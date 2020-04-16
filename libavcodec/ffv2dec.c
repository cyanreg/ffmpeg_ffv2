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
 */

#include "avcodec.h"
#include "internal.h"

#include "ffv2.h"
#include "daala_entropy.h"
#include "libavutil/pixdesc.h"
#include "libavutil/opt.h"

typedef struct FFV2DecCtx {
    AVClass *class;
    AVCodecContext *avctx;

    DaalaCDF subdiv_cdf;

    int qp;
    int num_sb_x;
    int num_sb_y;
    int planes;
    int bit_depth;

    FFV2DSP dsp;

    FFV2FCBuf buf[FFV2_REF_FRAMES];
} FFV2DecCtx;

static int alloc_coeff_buf(FFV2DecCtx *s, FFV2FCBuf *buf)
{
    int pad_w = FFALIGN(s->avctx->width  + 2*FFV2_PADDING, FFV2_SB_SIZE);
    int pad_h = FFALIGN(s->avctx->height + 2*FFV2_PADDING, FFV2_SB_SIZE);

    buf->sbs = av_mallocz(s->num_sb_x*sizeof(FFV2SB));
    for (int i = 0; i < s->num_sb_x; i++)
        buf->sbs[i] = av_mallocz(s->num_sb_y*sizeof(FFV2SB));

    for (int i = 0; i < s->planes; i++) {
        buf->pix_stride[i] = pad_w;
        buf->pix[i]  = av_calloc(buf->pix_stride[i]*pad_h, sizeof(dctcoef));
        memset(buf->pix[i], 0, buf->pix_stride[i]*pad_h*sizeof(dctcoef));
        buf->pix[i] += FFV2_PADDING*buf->pix_stride[i] + FFV2_PADDING;
    }

    return 0;
}

static void free_coeff_buf(FFV2DecCtx *s, FFV2FCBuf *buf)
{
    for (int i = 0; i < s->num_sb_x; i++)
        av_freep(&buf->sbs[i]);
    av_freep(&buf->sbs);

    for (int i = 0; i < s->planes; i++) {
        buf->pix[i] -= FFV2_PADDING*buf->pix_stride[i] + FFV2_PADDING;
        av_freep(&buf->pix[i]);
    }
}

static uint32_t decode_golomb(DaalaEntropy *e)
{
    int coeff = 1;

    while (!ff_daalaent_decode_bits(e, 1)) {
        coeff <<= 1;
        coeff |= ff_daalaent_decode_bits(e, 1);
    }

    return coeff - 1;
}

static void dequant_block(dctcoef *dst, int qp, int tx, DaalaEntropy *e)
{
    int i;
    int len = FFV2_IDX_TO_BS(FFV2_IDX_X(tx))*FFV2_IDX_TO_BS(FFV2_IDX_Y(tx));

    for (i = 0; i < len; i++) {
        dst[i] = decode_golomb(e);
        if (dst[i])
            dst[i] *= 1 - 2*ff_daalaent_decode_bits(e, 1);
        dst[i] *= qp;
    }
}

static int decode_block(FFV2DecCtx *s, DaalaEntropy *e, FFV2FCBuf *buf,
                        FFV2SB *sb, FFV2Block *blk, int o_x, int o_y,
                        enum FFV2TXSize s_x, enum FFV2TXSize s_y)
{
    enum FFV2TXType tx_type;
    dctcoef temp [FFV2_SB_SIZE*FFV2_SB_SIZE];
    dctcoef temp2[FFV2_SB_SIZE*FFV2_SB_SIZE];
    const int b_stride = FFV2_IDX_TO_BS(s_x);

    for (int p = 0; p < s->planes; p++) {
        blk->pix[p]   = sb->pix[p]   + o_y*buf->pix_stride[p]   + o_x;
        blk->coeff[p] = sb->coeff[p] + o_y*buf->coeff_stride[p] + o_x;
    }

    tx_type = ff_daalaent_decode_bits(e, 4);

    blk->qp   = s->qp;
    blk->type = FFV2_TX(s_x, s_y, tx_type);

    for (int p = 0; p < s->planes; p++) {
        dequant_block(temp, blk->qp, blk->type, e);
        s->dsp.coding_to_raster(temp2, b_stride, temp, blk->type);
        s->dsp.inv_tx(&s->dsp, blk->type, blk->pix[p], buf->pix_stride[p], temp2, b_stride);
    }

    return 0;
}

static int decode_block_rec(FFV2DecCtx *s, DaalaEntropy *e, FFV2FCBuf *buf,
                            FFV2SB *sb, int o_x, int o_y,
                            enum FFV2TXSize s_x, enum FFV2TXSize s_y)
{
    int cdf_idx = 0;
    int cdf_max = FFV2_SPLIT_NB;
    if ((s_x == FFV2_TX_4) && (s_y == FFV2_TX_4))
        goto decode;
    int split = ff_daalaent_decode_cdf_adapt(e, &s->subdiv_cdf, cdf_idx, cdf_max);
    switch (split) {
    case FFV2_SPLIT_END:
        goto decode;
    case FFV2_SPLIT_XY: {
        const int l_x = FFV2_IDX_TO_BS(--s_x);
        const int l_y = FFV2_IDX_TO_BS(--s_y);
        decode_block_rec(s, e, buf, sb, o_x +   0, o_y +   0, s_x, s_y);
        decode_block_rec(s, e, buf, sb, o_x + l_x, o_y +   0, s_x, s_y);
        decode_block_rec(s, e, buf, sb, o_x +   0, o_y + l_y, s_x, s_y);
        decode_block_rec(s, e, buf, sb, o_x + l_x, o_y + l_y, s_x, s_y);
        return 0;
    }
    case FFV2_SPLIT_Y: {
        const int l_y = FFV2_IDX_TO_BS(--s_y);
        decode_block_rec(s, e, buf, sb, o_x +   0, o_y +   0, s_x, s_y);
        decode_block_rec(s, e, buf, sb, o_x +   0, o_y + l_y, s_x, s_y);
        return 0;
    }
    case FFV2_SPLIT_X: {
        const int l_x = FFV2_IDX_TO_BS(--s_x);
        decode_block_rec(s, e, buf, sb, o_x +   0, o_y +   0, s_x, s_y);
        decode_block_rec(s, e, buf, sb, o_x + l_x, o_y +   0, s_x, s_y);
        return 0;
    }
    }

decode:
    return decode_block(s, e, buf, sb, &sb->block[o_x >> 2][o_y >> 2],
                        o_x, o_y, s_x, s_y);
}

//#define DEBUGGING

static void decode_sbs(FFV2DecCtx *s, FFV2FCBuf *buf, DaalaEntropy *e)
{
    ptrdiff_t offset_p[4] = { 0 };
    ptrdiff_t offset_c[4] = { 0 };

    for (int j = 0; j < s->num_sb_y; j++) {
        for (int i = 0; i < s->num_sb_x; i++) {
            FFV2SB *sb = &buf->sbs[i][j];
            memset(sb, 0, sizeof(FFV2SB));

            for (int p = 0; p < s->planes; p++) {
                sb->pix[p]   = buf->pix[p]   + offset_p[p] + i*FFV2_SB_SIZE;
                sb->coeff[p] = buf->coeff[p] + offset_c[p] + i*FFV2_SB_SIZE;
            }

            decode_block_rec(s, e, buf, sb, 0, 0, FFV2_TX_64, FFV2_TX_64);
        }
        for (int p = 0; p < s->planes; p++) {
            offset_p[p] += buf->pix_stride[p]  *FFV2_SB_SIZE;
            offset_c[p] += buf->coeff_stride[p]*FFV2_SB_SIZE;
        }
    }

    for (int j = 1; j < s->num_sb_y; j++) {
        for (int i = 0; i < s->num_sb_x; i++) {
            for (int p = 0; p < s->planes; p++) {
                FFV2SB *sb = &buf->sbs[i][j];
                s->dsp.lap_postfilter_ver(sb->pix[p], buf->pix_stride[p], 64, 32);
            }
        }
    }

    for (int j = 0; j < s->num_sb_y; j++) {
        for (int i = 1; i < s->num_sb_x; i++) {
            for (int p = 0; p < s->planes; p++) {
                FFV2SB *sb = &buf->sbs[i][j];
                s->dsp.lap_postfilter_hor(sb->pix[p], buf->pix_stride[p], 64, 32);
            }
        }
    }

#ifdef DEBUGGING
    for (int p = 0; p < s->planes; p++) {
        for (int j = 0; j < s->num_sb_y; j++) {
            for (int i = 0; i < s->num_sb_x; i++) {
                FFV2SB *sb = &buf->sbs[i][j];
                int32_t *src = sb->pix[p];
                for (int i = 0; i < 64; i++) {
                    src[i] = !p ? -2048 : 0;
                    src[i + 64*buf->pix_stride[p]] = src[i];
                    src[i*buf->pix_stride[p]] = src[i];
                    src[64 + i*buf->pix_stride[p]] = src[i];
                }
            }
        }
    }
#endif
}

static void decode_frame_header(FFV2DecCtx *s, DaalaEntropy *e)
{
    s->avctx->pix_fmt = ff_daalaent_decode_uint(e, AV_PIX_FMT_NB);
    s->qp = ff_daalaent_decode_uint(e, 2048);
}

#ifdef DEBUGGING
#include "libavutil/ffversion.h"
#include "libavutil/internal.h"
#include "libavutil/xga_font_data.h"
#include "libavutil/pixdesc.h"
#include "cga_data.h"
#include <sys/time.h>

static void print_debug_info(uint8_t *dst1, int linesize, int depth, const char *str)
{
    int i;
    for (i = 0; i < strlen(str); i++) {
        ff_draw_pc_font(dst1 + (i+1)*(depth > 8 ? 16 : 8), depth, linesize,
                        avpriv_cga_font, 8, str[i], 255, 1);
    }
}

#define PRINT_OSD_DEBUG(format, ...) \
    do { \
        snprintf(sbuf, sizeof(sbuf), format, ##__VA_ARGS__); \
        print_debug_info(dst1, frame->linesize[0], s->bit_depth, sbuf);    \
        av_log(avctx, AV_LOG_WARNING, "%s\n", sbuf); \
        dst1 += frame->linesize[0] * 10; \
    } while (0)

#endif

static int ffv2_decode_frame(AVCodecContext *avctx, void *data,
                             int *got_frame, AVPacket *avpkt)
{
    int ret;
    DaalaEntropy ent;
    AVFrame *frame = data;
    FFV2DecCtx *s = avctx->priv_data;

#ifdef DEBUGGING
    long long unsigned dec_time;
    struct timeval tv1, tv2;
    char sbuf[50];
    gettimeofday(&tv1, NULL);
#endif

    s->avctx = avctx;

    ff_daalaent_decode_init(&ent, avpkt->data, avpkt->size);

    daalaent_cdf_reset(&s->subdiv_cdf);
    decode_frame_header(s, &ent);

    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;

    s->bit_depth = av_pix_fmt_desc_get(avctx->pix_fmt)->comp[0].depth;
    s->planes = av_pix_fmt_count_planes(avctx->pix_fmt);

    s->num_sb_x = FFALIGN(s->avctx->width,  FFV2_SB_SIZE) / FFV2_SB_SIZE;
    s->num_sb_y = FFALIGN(s->avctx->height, FFV2_SB_SIZE) / FFV2_SB_SIZE;

    ff_ffv2dsp_init(&s->dsp, s->bit_depth);

    alloc_coeff_buf(s, &s->buf[0]);

    decode_sbs(s, &s->buf[0], &ent);

    for (int p = 0; p < s->planes; p++)
        s->dsp.coeff2ref(frame->data[p], frame->linesize[p],
                         s->buf[0].pix[p], s->buf[0].pix_stride[p],
                         s->avctx->width, s->avctx->height);

#ifdef DEBUGGING
    {
    uint8_t *dst1 = frame->data[0] + frame->linesize[0] * 8;
    gettimeofday(&tv2, NULL);
    dec_time = 1000*(tv2.tv_sec - tv1.tv_sec) + (tv2.tv_usec - tv1.tv_usec)/1000;
    PRINT_OSD_DEBUG("FFV2 rev: %s", FFMPEG_VERSION);
    PRINT_OSD_DEBUG("Frame size: %i x %i", s->avctx->width, s->avctx->height);
    PRINT_OSD_DEBUG("Superblocks: %i x %i", s->num_sb_x, s->num_sb_y);
    PRINT_OSD_DEBUG("Pixfmt: %s", av_get_pix_fmt_name(avctx->pix_fmt));
    PRINT_OSD_DEBUG("PTS: %li   DTS: %li", avpkt->pts, avpkt->dts);
    PRINT_OSD_DEBUG("Packet size: %0.2f kb", (avpkt->size)*0.001f);
    PRINT_OSD_DEBUG("Decoding time: %llu msec", dec_time);
    PRINT_OSD_DEBUG("Quantizer: %i", s->qp);
    }
#endif

    free_coeff_buf(s, &s->buf[0]);

    *got_frame = 1;

    return avpkt->size;
}

static av_cold int ffv2dec_init(AVCodecContext *avctx)
{
    FFV2DecCtx *s = avctx->priv_data;

    if (daalaent_cdf_alloc(&s->subdiv_cdf, 1, FFV2_SPLIT_NB, 128, 0, 2, 0))
        return AVERROR(ENOMEM);

    return 0;
}

static av_cold int ffv2dec_close(AVCodecContext *avctx)
{
    FFV2DecCtx *s = avctx->priv_data;

    daalaent_cdf_free(&s->subdiv_cdf);

    return 0;
}

#define FFV2DEC_FLAGS (AV_OPT_FLAG_DECODING_PARAM | AV_OPT_FLAG_VIDEO_PARAM)
static const AVOption ffv2dec_options[] = {
    { NULL }
};

static const AVClass ffv2dec_class = {
    .class_name = "FFV2 decoder",
    .category   = AV_CLASS_CATEGORY_DECODER,
    .option     = ffv2dec_options,
    .item_name  = av_default_item_name,
    .version    = LIBAVUTIL_VERSION_INT,
};

AVCodec ff_ffv2_decoder = {
    .name           = "ffv2",
    .priv_class     = &ffv2dec_class,
    .long_name      = NULL_IF_CONFIG_SMALL("FFv2"),
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = AV_CODEC_ID_FFV2,
    .init           = ffv2dec_init,
    .decode         = ffv2_decode_frame,
    .close          = ffv2dec_close,
    .priv_data_size = sizeof(FFV2DecCtx),
    .capabilities   = AV_CODEC_CAP_DR1,
    .caps_internal  = FF_CODEC_CAP_INIT_THREADSAFE |
                      FF_CODEC_CAP_INIT_CLEANUP,
};
