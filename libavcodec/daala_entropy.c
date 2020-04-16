/*
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 *
 * This file incorporates work covered by the following copyright and
 * permission notice:
 *
 *     Copyright 2001-2015 Xiph.Org and contributors.
 *
 *     Redistribution and use in source and binary forms, with or without
 *     modification, are permitted provided that the following conditions
 *     are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 *     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *     ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *     A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
 *     OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *     EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *     PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 *     PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 *     LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 *     NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "libavutil/avassert.h"
#include "daala_entropy.h"
#include "daalatab.h"

#define DAALAENT_UINT_BITS      4
#define DAALAENT_MODEL_TAB      12
#define DAALAENT_BIT_ABUNDANCE  16384
#define DAALAENT_SAT(a,b)       ((a) - FFMIN(a,b))
#define DAALAENT_WSIZE          CHAR_BIT*(int)sizeof(ent_win)
#define DAALAENT_CDF_ACCESS(n)  &ff_daalaent_cdf_tab[((n)*((n) - 1) >> 1) - 1]

enum DaalaCDFType {
    CDF_NORM = 0,
    CDF_Q15,
    CDF_UNSCALED,
    CDF_DYADIC,
};

/* Expectation value log, outputs Q1 */
static av_always_inline int daalaent_log_ex(int ex_q16)
{
    int o, log = daalaent_log2(ex_q16);
    if (log < 15) {
        o = ex_q16*ex_q16 > 2 << 2*log;
    } else {
        int tmp = ex_q16 >> (log - 8);
        o = tmp*tmp > (1 << 15);
    }
    return FFMAX(0, 2*log - 33 + o);
}

/* Updates the context */
static av_always_inline void daalaent_fillup(DaalaEntropy *e)
{
    int i = DAALAENT_WSIZE - 9 - (e->count + 15);
    uint8_t *ebuf = e->ebuf, *buf = e->buf;
    for (; i >= 0 && buf < ebuf; i -= 8, buf++) {
        av_assert0(i <= DAALAENT_WSIZE - 8);
        e->diff |= (ent_win)buf[0] << i;
        e->count += 8;
    }
    if (buf >= ebuf) {
        e->eos_offset += DAALAENT_BIT_ABUNDANCE - e->count;
        e->count = DAALAENT_BIT_ABUNDANCE;
    }
    e->ebuf = ebuf;
    e->buf = buf;
}

/* Renormalizes, takes new range and diff as arguments */
static av_always_inline void daalaent_renormalize(DaalaEntropy *e, ent_win diff,
                                                  uint16_t range)
{
    int i = 16 - daalaent_log2(range);
    e->diff = diff << i;
    e->range = range << i;
    if ((e->count -= i) < 0)
        daalaent_fillup(e);
}

static av_always_inline void daalaent_enc_renormalize(DaalaEntropy *e, ent_win low,
                                                      uint32_t range)
{
    int d, c, s;
    c = e->count;
    av_assert0(range <= 65535U);
    d = 16 - daalaent_log2(range);
    s = c + d;
    if (s >= 0) {
        uint16_t *buf;
        uint32_t storage;
        uint32_t offs, m;
        buf = e->precarry_buf;
        storage = e->precarry_storage;
        offs = e->offset;
        if (offs + 2 > storage) {
            storage = 2*storage + 2;
            buf = av_realloc(buf, sizeof(*buf)*storage);
            if (buf == NULL) {
                e->err = -1;
                e->offset = 0;
                return;
            }
            e->precarry_buf = buf;
            e->precarry_storage = storage;
        }
        c += 16;
        m = (1 << c) - 1;
        if (s >= 8) {
            av_assert0(offs < storage);
            buf[offs++] = (uint16_t)(low >> c);
            low &= m;
            c -= 8;
            m >>= 8;
        }
        av_assert0(offs < storage);
        buf[offs++] = (uint16_t)(low >> c);
        s = c + d - 24;
        low &= m;
        e->offset = offs;
    }
    e->low   = low << d;
    e->range = range << d;
    e->count = s;
}

/* Decodes a bool from the bitstream, p ∈ (0, p_tot), p_tot ∈ [16384, 32768] */
int ff_daalaent_decode_bool(DaalaEntropy *e, uint32_t p, uint32_t p_tot)
{
    int rval;
    ent_win diff = e->range - p_tot;
    int64_t tmp = diff >= p_tot;
    av_assert0(e->diff >> (DAALAENT_WSIZE - 16) < e->range);
    p <<= tmp;
    p_tot <<= tmp;
    tmp  = DAALAENT_SAT(2*diff, p_tot);
    tmp  = p + FFMIN(p, tmp) + FFMIN(DAALAENT_SAT(p, tmp) >> 1, diff);
    diff = tmp << (DAALAENT_WSIZE - 16);
    rval = e->diff >= diff;
    diff = e->diff - (rval ? diff : 0);
    tmp  = rval ? e->range - tmp : tmp;
    daalaent_renormalize(e, diff, tmp);
    return rval;
}

void ff_daalaent_encode_bool(DaalaEntropy *e, int val, uint32_t p, uint32_t p_tot)
{
    int s;
    ent_win l;
    uint32_t r, v, d, g;
    av_assert0(0 < p);
    av_assert0(p < p_tot);
    av_assert0(16384 <= p_tot);
    av_assert0(p_tot <= 32768U);
    l = e->low;
    r = e->range;
    av_assert0(p_tot <= r);
    s = r - p_tot >= p_tot;
    p_tot <<= s;
    p <<= s;
    av_assert0(r - p_tot < p_tot);
    d = r - p_tot;
    g = DAALAENT_SAT(2*d, p_tot);
    v = p + FFMIN(p, g) + FFMIN(DAALAENT_SAT(p, g) >> 1, d);
    if (val)
        l += v;
    r = val ? r - v : v;
    daalaent_enc_renormalize(e, l, r);
    e->entropy -= log2((double)(val ? p_tot - p : p)/p_tot);
    e->nb_symbols++;
}

/*  Decodes raw bits from the bitstream, num ∈ [0, 25] */
int ff_daalaent_decode_bits(DaalaEntropy *e, int num)
{
    int avail = e->end_window_size;
    ent_win ret, win = e->end_window;
    if (avail < num) {
        uint8_t *erbuf = e->erbuf;
        av_assert0(avail <= DAALAENT_WSIZE - 8);
        do {
            if (erbuf <= e->rbuf) {
                e->eos_offset += DAALAENT_BIT_ABUNDANCE - avail;
                avail = DAALAENT_BIT_ABUNDANCE;
                break;
            }
            win |= (ent_win)*--erbuf << avail;
            avail += 8;
        } while (avail <= DAALAENT_WSIZE - 8);
        e->erbuf = erbuf;
    }
    ret = win & ((1 << num) - 1);
    win >>= num;
    avail -= num;
    e->end_window = win;
    e->end_window_size = avail;
    return ret;
}

/* Encode up to 25 bits to the bitstream */
void ff_daalaent_encode_bits(DaalaEntropy *e, uint32_t val, int n)
{
    int nend_bits = e->nend_bits;
    ent_win end_window = e->end_window;
    av_assert0(n <= 25);
    av_assert0(val < (uint32_t)1 << n);
    e->entropy += n;
    if (nend_bits + n > DAALAENT_WSIZE) {
        uint8_t *buf = e->buf;
        uint32_t storage;
        uint32_t end_offs;
        storage = e->storage;
        end_offs = e->end_offset;
        if (end_offs + (DAALAENT_WSIZE >> 3) >= storage) {
            uint8_t *new_buf;
            uint32_t new_storage;
            new_storage = 2*storage + (DAALAENT_WSIZE >> 3);
            new_buf = av_malloc(sizeof(*new_buf)*new_storage);
            if (!new_buf) {
                e->err = -1;
                e->end_offset = 0;
                return;
            }
            memcpy(new_buf + new_storage - end_offs,
                   buf + storage - end_offs, end_offs);
            storage = new_storage;
            free(buf);
            e->buf = buf = new_buf;
            e->storage = storage;
        }
        do {
            av_assert0(end_offs < storage);
            buf[storage - ++end_offs] = (uint8_t)end_window;
            end_window >>= 8;
            nend_bits -= 8;
        } while (nend_bits >= 8);
        e->end_offset = end_offs;
    }
    av_assert0(nend_bits + n <= DAALAENT_WSIZE);
    end_window |= (ent_win)val << nend_bits;
    nend_bits += n;
    e->end_window = end_window;
    e->nend_bits = nend_bits;
}

/* Decodes a symbol using a CDF table */
static int daalaent_decode_cdf(DaalaEntropy *e, const uint16_t *cdf,
                               int cdf_size, uint32_t p_tot,
                               enum DaalaCDFType type)
{
    int d, lim, g, scale, ret = 0;
    uint16_t range = e->range;
    ent_win diff = e->diff, u = 0, v = 0;
    const int cshift = DAALAENT_WSIZE - 16;
    const int cval = diff >> cshift;
    av_assert0(diff >> cshift < range); /* Probably the most important assert */
    if (type == CDF_UNSCALED) {
        p_tot = cdf[cdf_size - 1];
        av_assert0(2 <= p_tot && p_tot <= 32768);
        scale = 15 - daalaent_log2(p_tot - 1);
        p_tot <<= scale;
        av_assert0(p_tot <= range);
        if (range - p_tot >= p_tot) {
            p_tot <<= 1;
            scale++;
        }
        d = range - p_tot;
    } else if (type == CDF_Q15) {
        av_assert0(cdf[cdf_size - 1] == 32768);
        av_assert0(32768 <= range);
        d = range - 32768;
        p_tot = 32768;
        scale = 0;
    } else if (type == CDF_DYADIC) {
        av_assert0(cdf[cdf_size - 1] == 1 << p_tot);
        scale = 15 - p_tot;
        av_assert0(32768 <= range);
        d = range - 32768;
        p_tot = 32768;
    } else {
        p_tot = cdf[cdf_size - 1];
        av_assert0(16384 <= p_tot && p_tot <= 32768);
        av_assert0(p_tot <= range);
        scale = range - p_tot >= p_tot;
        p_tot <<= scale;
        d = range - p_tot;
    }
    g = DAALAENT_SAT(2*d, p_tot);
    lim = FFMAX(FFMAX(cval >> 1, cval - d), (2*cval + 1 - g)/3) >> scale;
    for (v = cdf[ret]; v <= lim; v = cdf[++ret])
        u = v;
    u <<= scale;
    v <<= scale;
    u = u + FFMIN(u, g) + FFMIN(DAALAENT_SAT(u, g) >> 1, d);
    v = v + FFMIN(v, g) + FFMIN(DAALAENT_SAT(v, g) >> 1, d);
    range = v - u;
    diff -= u << cshift;
    daalaent_renormalize(e, diff, range);
    return ret;
}

static void daalaent_encode_cdf(DaalaEntropy *e, int s, const uint16_t *cdf,
                                int nsyms, enum DaalaCDFType type)
{
    int scale;
    ent_win l;
    uint32_t fl, fh, ft, r, d, u, v, g;
    if (type == CDF_UNSCALED) {
        av_assert0(s >= 0);
        av_assert0(s < nsyms);
        fl = s > 0 ? cdf[s - 1] : 0;
        fh = cdf[s];
        ft = cdf[nsyms - 1];
        av_assert0(fl < fh);
        av_assert0(fh <= ft);
        av_assert0(2 <= ft);
        av_assert0(ft <= 32768U);
        scale = 15 - daalaent_log2(ft - 1);
        fl <<= scale;
        fh <<= scale;
        ft <<= scale;
    } else if (type == CDF_Q15) {
        av_assert0(s >= 0);
        av_assert0(s < nsyms);
        av_assert0(cdf[nsyms - 1] == 32768U);
        fl = s > 0 ? cdf[s - 1] : 0;
        fh = cdf[s];
        ft = 32768U;
    } else {
        return;
    }
    av_assert0(fl < fh);
    av_assert0(fh <= ft);
    av_assert0(16384 <= ft);
    av_assert0(ft <= 32768U);
    l = e->low;
    r = e->range;
    av_assert0(ft <= r);
    scale = r - ft >= ft;
    ft <<= scale;
    fl <<= scale;
    fh <<= scale;
    d = r - ft;
    av_assert0(d < ft);
    g = DAALAENT_SAT(2*d, ft);
    u = fl + FFMIN(fl, g) + FFMIN(DAALAENT_SAT(fl, g) >> 1, d);
    v = fh + FFMIN(fh, g) + FFMIN(DAALAENT_SAT(fh, g) >> 1, d);
    r = v - u;
    l += u;
    e->entropy -= log2((double)(fh - fl)/ft);
    e->nb_symbols++;
    daalaent_enc_renormalize(e, l, r);
}

/* Decodes a uint from the bitstream, num ∈ [2, 2^29] */
uint32_t ff_daalaent_decode_uint(DaalaEntropy *e, uint32_t num)
{
    av_assert0(num <= 1 << (25 + DAALAENT_UINT_BITS));
    if (num > 1 << DAALAENT_UINT_BITS) {
        int bit = daalaent_log2(--num) - DAALAENT_UINT_BITS;
        int adr = (num >> bit) + 1;
        int t = daalaent_decode_cdf(e, DAALAENT_CDF_ACCESS(adr), adr, 0, CDF_Q15);
        t = t << bit | ff_daalaent_decode_bits(e, bit);
        if (t <= num)
            return t;
        e->err = 1;
        return num;
    }
    return daalaent_decode_cdf(e, DAALAENT_CDF_ACCESS(num), num, 0, CDF_Q15);
}

/* Encodes a uint to the bitstream, val = value, num ∈ [2, 2^29] */
void ff_daalaent_encode_uint(DaalaEntropy *e, uint32_t val, uint32_t num)
{
    av_assert0(num <= (uint32_t)1 << (25 + DAALAENT_UINT_BITS));
    if (num > 1 << DAALAENT_UINT_BITS) {
        int bit = daalaent_log2(--num) - DAALAENT_UINT_BITS;
        int adr = (num >> bit) + 1;
        daalaent_encode_cdf(e, (int)(val >> bit), DAALAENT_CDF_ACCESS(adr), adr, CDF_Q15);
        ff_daalaent_encode_bits(e, val & (((uint32_t)1 << bit) - 1), bit);
        return;
    }
    daalaent_encode_cdf(e, (int)val, DAALAENT_CDF_ACCESS(num), (int)num, CDF_Q15);
}

/* Decodes a symbol using a CDF */
int ff_daalaent_decode_cdf_adapt(DaalaEntropy *e, DaalaCDF *c, int cdf_off, int n)
{
    int i;
    uint16_t *cdf = &c->cdf[cdf_off*c->y];
    const int rval = daalaent_decode_cdf(e, cdf, n, 0, CDF_UNSCALED);
    if (cdf[n - 1] + c->inc > 32767) {
        for (i = 0; i < n; i++)
            cdf[i] = (cdf[i] >> 1) + i + 1;
    }
    for (i = rval; i < n; i++)
        cdf[i] += c->inc;
    return rval;
}

/* Encodes a symbol using a CDF */
void ff_daalaent_encode_cdf_adapt(DaalaEntropy *e, DaalaCDF *c, int val,
                                  int cdf_off, int n)
{
    int i;
    uint16_t *cdf = &c->cdf[cdf_off*c->y];
    daalaent_encode_cdf(e, val, cdf, n, CDF_UNSCALED);
    if (cdf[n - 1] + c->inc > 32767) {
        for (i = 0; i < n; i++)
            cdf[i] = (cdf[i] >> 1) + i + 1;
    }
    for (i = val; i < n; i++)
        cdf[i] += c->inc;
}

/* "Special laplace decoder" */
int ff_daalaent_decode_laplace(DaalaEntropy *e, uint32_t decay, int max)
{
    const uint16_t *cdf;
    int pos, sym, max_shift, shift = 0, p_shift = 0;
    if (!max)
        return 0;
    while (((max >> shift) >= 15 || max == -1) && decay > 235) {
        decay = (decay*decay + 128) >> 8;
        shift++;
    }
    max_shift = max >> shift;
    decay = FFMAX(FFMIN(decay, 254), 2);
    cdf = ff_daalaent_cdf_exp_tab[(decay + 1) >> 1];
    do {
        const int shift_bound = max_shift > 0 && max_shift < 15;
        const int cdf_size = shift_bound ? max_shift + 1 : 16;
        const int cdf_type = shift_bound ? CDF_UNSCALED : CDF_Q15;
        sym = daalaent_decode_cdf(e, cdf, cdf_size, 0, cdf_type);
        p_shift += sym;
        max_shift -= 15;
    } while (sym >= 15 && max_shift);
    pos = shift ? (p_shift << shift) + ff_daalaent_decode_bits(e, shift) : p_shift;
    av_assert0(pos >> shift <= max >> shift || max == -1);
    if (max != -1 && pos > max) {
        pos = max;
        e->err = 1;
    }
    return pos;
}

/* Special" laplace encoder */
void ff_daalaent_encode_laplace(DaalaEntropy *e, int x, uint32_t decay, int max)
{
    int xs, ms, sym, shift = 0;
    const uint16_t *cdf;
    if (max == 0)
        return;
    while (((max >> shift) >= 15 || max == -1) && decay > 235) {
        decay = (decay*decay + 128) >> 8;
        shift++;
    }
    av_assert0(x <= max || max == -1);
    decay = FFMIN(decay, 254);
    decay = FFMAX(decay, 2);
    xs = x >> shift;
    ms = max >> shift;
    cdf = ff_daalaent_cdf_exp_tab[(decay + 1) >> 1];
    do {
        enum DaalaCDFType type = ms > 0 && ms < 15 ? CDF_UNSCALED : CDF_Q15;
        int ex = ms > 0 && ms < 15 ? ms + 1 : 16;
        sym = FFMIN(xs, 15);
        daalaent_encode_cdf(e, sym, cdf, ex, type);
        xs -= 15;
        ms -= 15;
    } while (sym >= 15 && ms != 0);
    if (shift)
        ff_daalaent_encode_bits(e, x & ((1 << shift) - 1), shift);
}

/* Used by the vector and delta laplace decoding functions for PVQ */
int ff_daalaent_decode_laplace_pvq(DaalaEntropy *e, uint32_t exp_v, int max_mod)
{
    int sym = 0, lsb = 0;
    const int shift = FFMAX(daalaent_log2(exp_v) - 11, 0);
    const int ex = (exp_v + (1 << shift >> 1)) >> shift;
    const int maxval = (max_mod + (1 << shift >> 1)) >> shift;
    const int decay = FFMIN(254, 256*ex/(ex + 256));
    const int offset = ff_daalaent_laplace_offset[(decay + 1) >> 1];
    if (maxval) {
        int i;
        uint16_t cdf[16];
        for (i = 0; i < 16; i++)
            cdf[i] = ff_daalaent_cdf_exp_tab[(decay + 1) >> 1][i] - offset;
        sym = daalaent_decode_cdf(e, cdf, FFMIN(maxval + 1, 16), 0, CDF_UNSCALED);
    }
    if (shift) {
        if (shift - !sym > 0)
            lsb = ff_daalaent_decode_bits(e, shift - !sym);
        lsb -= (!!sym << (shift - 1));
    }
    if (sym == 15) /* Tail */
        sym += ff_daalaent_decode_laplace(e, decay, maxval - 15);
    return (sym << shift) + lsb;
}

/* Expectation value is in Q16, will update the context as well */
int ff_daalaent_decode_generic(DaalaEntropy *e, DaalaCDF *c, int *ex, int max,
                               int integrate)
{
    int i, xenc, tmp, rval, lsb = 0, log_ex = daalaent_log_ex(*ex);
    const int shift = FFMAX(0, (log_ex - 5) >> 1);
    const int id = FFMIN(DAALAENT_MODEL_TAB - 1, log_ex);
    const int ms = (max + (1 << shift >> 1)) >> shift;
    int xs = (max == -1) ? 16 : FFMIN(ms + 1, 16);
    uint16_t *cdf = &c->cdf[id*c->y];
    if (!max)
        return 0;
    if ((xs = daalaent_decode_cdf(e, cdf, xs, 0, CDF_UNSCALED)) == 15) {
        int g = ((*ex >> 7) + (1 << shift >> 1)) >> shift;
        ent_win decay = av_clip(256*g / (g + 256), 2, 254);
        xs += ff_daalaent_decode_laplace(e, decay, (max == -1) ? -1 : ms - 15);
    }
    if (shift) {
        if (shift > !xs)
            lsb = ff_daalaent_decode_bits(e, shift - !xs);
        lsb -= !!xs << (shift - 1);
    }
    rval = (xs << shift) + lsb;
    if (cdf[15] + c->inc > 32767) {
        for (i = 0; i < 16; i++)
            cdf[i] = (cdf[i] >> 1) + i + 1;
    }
    xenc = FFMIN(15, xs);
    for (i = xenc; i < 16; i++)
        cdf[i] += c->inc;
    tmp = FFMIN(rval, 32767);
    *ex += ((tmp << 16) - *ex) >> integrate;
    return rval;
}

/* Inits an entropy decoding context */
void ff_daalaent_decode_init(DaalaEntropy *e, uint8_t *buf, int buf_size)
{
    e->rbuf = buf;
    e->erbuf = buf + buf_size;
    e->buf = buf;
    e->ebuf = buf + buf_size;
    e->err = 0;
    e->diff  = 0;
    e->range = 0x8000;
    e->count = -15;
    e->eos_offset = 10 - (DAALAENT_WSIZE - 8);
    e->end_window = 0;
    e->end_window_size = 0;
    daalaent_fillup(e);
}

/* Resets the encoding context */
void ff_daalaent_encode_reset(DaalaEntropy *e)
{
    e->end_offset = 0;
    e->end_window = 0;
    e->nend_bits = 0;
    e->offset = 0;
    e->low = 0;
    e->range = 0x8000;
    e->count = -9;
    e->err = 0;
    e->entropy = 0;
    e->nb_symbols = 0;
}

/* Inits an entropy encoding context with an initial size */
int ff_daalaent_encode_init(DaalaEntropy *e, uint32_t size)
{
    ff_daalaent_encode_reset(e);
    e->buf = av_malloc(sizeof(*e->buf)*size);
    e->storage = size;
    if (size > 0 && !e->buf)
        return AVERROR(ENOMEM);
    e->precarry_buf = av_malloc(sizeof(*e->precarry_buf)*size);
    e->precarry_storage = size;
    if (size > 0 && !e->precarry_buf) {
        av_free(&e->buf);
        return AVERROR(ENOMEM);
    }
    return 0;
}

void ff_daalaent_encode_efficiency(DaalaEntropy *e, void *avctx)
{
    uint32_t bits = daalaent_enc_bits_count(e) - 1;
    av_log(avctx, AV_LOG_WARNING, "overhead: %f%%\n", e->entropy);
    av_log(avctx, AV_LOG_WARNING, "efficiency: %f bits/symbol\n", (double)bits/e->nb_symbols);
}

static void daalaent_buf_free(void *opaque, uint8_t *data)
{
    av_free(opaque);
}

int ff_daalaent_encode_done(DaalaEntropy *enc, AVPacket *avpkt)
{
    uint8_t *out;
    uint32_t storage;
    uint16_t *buf;
    uint32_t offs;
    uint32_t end_offs;
    int nend_bits;
    if (enc->err)
        return 1;

    /* We output the minimum number of bits that ensures that the symbols encoded
       thus far will be decoded correctly regardless of the bits that follow */
    ent_win l = enc->low;
    uint32_t r = enc->range;
    int c = enc->count;
    int s = 9;
    ent_win m = 0x7FFF;
    ent_win e = (l + m) & ~m;

    while ((e | m) >= l + r) {
        s++;
        m >>= 1;
        e = (l + m) & ~m;
    }
    s += c;
    offs = enc->offset;
    buf = enc->precarry_buf;
    if (s > 0) {
        uint32_t n;
        storage = enc->precarry_storage;
        if (offs + ((s + 7) >> 3) > storage) {
            storage = storage*2 + ((s + 7) >> 3);
            buf = av_realloc(buf, sizeof(*buf)*storage);
            if (!buf) {
                enc->err = -1;
                return 1;
            }
            enc->precarry_buf = buf;
            enc->precarry_storage = storage;
        }
        n = (1 << (c + 16)) - 1;
        do {
            av_assert0(offs < storage);
            buf[offs++] = (uint16_t)(e >> (c + 16));
            e &= n;
            s -= 8;
            c -= 8;
            n >>= 8;
        } while (s > 0);
    }

    /* Make sure there's enough room for the entropy-coded bits and the raw
       bits */
    out = enc->buf;
    storage = enc->storage;
    end_offs = enc->end_offset;
    e = enc->end_window;
    nend_bits = enc->nend_bits;
    s = -s;
    c = FFMAX((nend_bits - s + 7) >> 3, 0);
    if (offs + end_offs + c > storage) {
        storage = offs + end_offs + c;
        out = av_realloc(out, sizeof(*out)*storage);
        if (!out) {
            enc->err = -1;
            return 1;
        }
        memmove(out + storage - end_offs, out + enc->storage - end_offs, end_offs*sizeof(*out));
        enc->buf = out;
        enc->storage = storage;
    }

    /* If we have buffered raw bits, flush them as well */
    while (nend_bits > s) {
        av_assert0(end_offs < storage);
        out[storage - ++end_offs] = (uint8_t)e;
        e >>= 8;
        nend_bits -= 8;
    }
    int bytes = offs + end_offs;

    /* Perform carry propagation */
    av_assert0(offs + end_offs <= storage);
    out = out + storage - (offs + end_offs);
    c = 0;
    end_offs = offs;
    while (offs-- > 0) {
        c = buf[offs] + c;
        out[offs] = (uint8_t)c;
        c >>= 8;
    }

    /* Add any remaining raw bits to the last byte
       There is guaranteed to be enough room, because nend_bits <= s */
    av_assert0(nend_bits <= 0 || end_offs > 0);
    if (nend_bits > 0)
        out[end_offs - 1] |= (uint8_t)e;

    /* Free entropy coded part we already copied */
    av_free(enc->precarry_buf);

    /* Transfer ownership of the buffer to the avpkt */
    avpkt->buf = av_buffer_create(out, bytes, daalaent_buf_free, enc->buf, 0);
    if (!avpkt->buf)
        return AVERROR(ENOMEM);

    avpkt->data = out;
    avpkt->size = bytes;

    return 0;
}
