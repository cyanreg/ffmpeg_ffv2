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

#ifndef AVCODEC_DAALAENTROPY_H
#define AVCODEC_DAALAENTROPY_H

#include "avcodec.h"

#define daalaent_log2(x) (int)(1 + ff_log2(x))

#if 1
typedef uint64_t ent_win;         /* Has to be able to express uint32_t */
#else
typedef uint32_t ent_win;
#endif

typedef struct DaalaCDF {
    uint16_t *cdf;
    int x, y, inc, inc_g, fir;
    int gen_mod;
} DaalaCDF;

typedef struct DaalaEntropy {
    uint8_t *buf, *ebuf;    /* Normal, ends before the raw bits start   */
    uint8_t *rbuf, *erbuf;  /* Raw, located at the end of the bitstream */
    ent_win diff, low, end_window;
    uint32_t offset, end_offset, storage, precarry_storage;
    uint16_t *precarry_buf, range;
    int16_t count;
    int nb_symbols, nend_bits, eos_offset, end_window_size, err;
    double entropy;
} DaalaEntropy;

/* Boolean value, p ∈ (0, p_tot), p_tot ∈ [16384, 32768] */
int  ff_daalaent_decode_bool(DaalaEntropy *e, uint32_t p, uint32_t p_tot);
void ff_daalaent_encode_bool(DaalaEntropy *e, int val, uint32_t p, uint32_t p_tot);

/* Rawbits, 0 to 25 */
int  ff_daalaent_decode_bits(DaalaEntropy *e, int num);
void ff_daalaent_encode_bits(DaalaEntropy *e, uint32_t val, int n);

/* Uint, num ∈ [2, 2^29] */
uint32_t ff_daalaent_decode_uint(DaalaEntropy *e, uint32_t num);
void     ff_daalaent_encode_uint(DaalaEntropy *e, uint32_t val, uint32_t num);

/* Decodes a symbol using a CDF */
int  ff_daalaent_decode_cdf_adapt(DaalaEntropy *e, DaalaCDF *c, int cdf_off, int n);
void ff_daalaent_encode_cdf_adapt(DaalaEntropy *e, DaalaCDF *c, int val, int cdf_off, int n);

/* "Special laplace decoder" */
int  ff_daalaent_decode_laplace(DaalaEntropy *e, uint32_t decay, int max);
void ff_daalaent_encode_laplace(DaalaEntropy *e, int x, uint32_t decay, int max);

/* Used by the vector and delta laplace decoding functions for PVQ */
int ff_daalaent_decode_laplace_pvq(DaalaEntropy *e, uint32_t exp_v, int max_mod);

/* Decodes an integer encoded using generic exponential probability decay */
int ff_daalaent_decode_generic(DaalaEntropy *e, DaalaCDF *c, int *ex, int max,
                               int integrate);



/* Inits an entropy decoding context */
void ff_daalaent_decode_init(DaalaEntropy *e, uint8_t *buf, int buf_size);

/* Resets an entropy encoding context (called on init) */
void ff_daalaent_encode_reset(DaalaEntropy *e);

/* Inits an entropy encoding context */
int ff_daalaent_encode_init(DaalaEntropy *e, uint32_t size);

/* Prints the efficiency of the encoder */
void ff_daalaent_encode_efficiency(DaalaEntropy *e, void *avctx);

/* Finalizes the buffer and frees resources */
int ff_daalaent_encode_done(DaalaEntropy *enc, AVPacket *avpkt);

/* Gets a single bit if !!cond and returns ±1 */
#define daalaent_dec_cphase(e, c)    ((c) ? (1 - 2*ff_daalaent_decode_bits(e, 1)) : 1)

/* Encodes a single bit depending on the sign */
#define daalaent_enc_cphase(e, c, v) ((c) ? ff_daalaent_encode_bits(e, (v) < 0, 1) : 0)

/* Number of bits read */
static av_always_inline int daalaent_bits_count(DaalaEntropy *e)
{
    return (((e->ebuf - e->erbuf) + (e->buf - e->rbuf))*8 -
            e->count - e->end_window_size + e->eos_offset);
}

/* Number of bits used during encoding */
static av_always_inline int daalaent_enc_bits_count(DaalaEntropy *e)
{
    return (e->offset + e->end_offset)*8 + e->count + e->nend_bits + 10;
}

static av_always_inline void daalaent_cdf_reset(DaalaCDF *s)
{
    int i, j;
    for (i = 0; i < s->x; i++)
        for (j = 0; j < s->y; j++)
            s->cdf[i*s->y + j] = s->inc_g*(j + s->gen_mod) + s->fir;
}

static inline int daalaent_cdf_alloc(DaalaCDF *s, int x, int y, int inc, int fir,
                                     int inc_shift, int gen_mod)
{
    s->x = x;
    s->y = y;
    s->inc = inc;
    s->gen_mod = gen_mod;
    s->inc_g = s->inc >> inc_shift;
    s->fir = !!fir || s->gen_mod ? fir : s->inc_g;
    s->cdf = av_malloc(x*y*sizeof(*s->cdf));
    if (!s->cdf)
        return 1;
    return 0;
}

#define daalaent_cdf_free(s) av_freep(&((s)->cdf))

#endif /* AVCODEC_DAALAENTROPY_H */
