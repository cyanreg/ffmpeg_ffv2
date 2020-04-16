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
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"

#include "ffv2.h"
#include "daala_entropy.h"

#define REGENERATE_ZIGZAGS 0

typedef struct FFV2EncCtx {
    AVClass *class;
    AVCodecContext *avctx;

    DaalaCDF subdiv_cdf;

    int qp;
    int num_sb_x;
    int num_sb_y;
    int planes;
    int bit_depth;

#if REGENERATE_ZIGZAGS
    uint64_t num_f;
    double tx_score[5][5][64][64];
#endif

    FFV2DSP dsp;

    FFV2FCBuf buf[FFV2_REF_FRAMES];

    /* Options */
    uint8_t lossless;
} FFV2EncCtx;

static int alloc_coeff_buf(FFV2EncCtx *s, FFV2FCBuf *buf)
{
    int pad_w = FFALIGN(s->avctx->width  + 2*FFV2_PADDING, FFV2_SB_SIZE);
    int pad_h = FFALIGN(s->avctx->height + 2*FFV2_PADDING, FFV2_SB_SIZE);

    buf->sbs = av_mallocz(s->num_sb_x*sizeof(FFV2SB));
    for (int i = 0; i < s->num_sb_x; i++)
        buf->sbs[i] = av_mallocz(s->num_sb_y*sizeof(FFV2SB));

    buf->grid = av_mallocz(s->num_sb_x*s->num_sb_y*16*16*sizeof(*buf->grid));
    buf->grid_stride = s->num_sb_y*16;

    for (int i = 0; i < s->planes; i++) {
        buf->pix_stride[i] = pad_w;
        buf->pix[i]  = av_calloc(buf->pix_stride[i]*pad_h, sizeof(dctcoef));
        memset(buf->pix[i], 0, buf->pix_stride[i]*pad_h*sizeof(dctcoef));
        buf->pix[i] += FFV2_PADDING*buf->pix_stride[i] + FFV2_PADDING;
    }

    return 0;
}

static void rec_free_block(FFV2Block *blk)
{
    if (!blk)
        return;
    for (int i = 0; i < 4; i++) {
        rec_free_block(blk->subdiv[i]);
        av_freep(&blk->subdiv[i]);
    }
}

static void free_coeff_buf(FFV2EncCtx *s, FFV2FCBuf *buf)
{
    for (int j = 0; j < s->num_sb_y; j++)
        for (int i = 0; i < s->num_sb_x; i++)
            rec_free_block(&buf->sbs[i][j].base);

    for (int i = 0; i < s->num_sb_x; i++)
        av_freep(&buf->sbs[i]);
    av_freep(&buf->sbs);

    av_freep(&buf->grid);

    for (int i = 0; i < s->planes; i++) {
        buf->pix[i] -= FFV2_PADDING*buf->pix_stride[i] + FFV2_PADDING;
        av_freep(&buf->pix[i]);
    }
}

static void encode_golomb(DaalaEntropy *e, uint32_t val)
{
    int topbit = 1, maxval = 1;

    if (!val++)
        goto end;

    while (val > maxval) {
        topbit <<= 1;
        maxval <<= 1;
        maxval |=  1;
    }

    for (int i = ff_log2(topbit) - 1; i >= 0; i--)
        ff_daalaent_encode_bits(e, !!(val & (1 << i)) << 1, 2);

end:
    ff_daalaent_encode_bits(e, 1, 1);
}

static void quant_block(dctcoef *src, int qp, int tx, DaalaEntropy *e)
{
    int i;
    int len = FFV2_IDX_TO_BS(FFV2_IDX_X(tx))*FFV2_IDX_TO_BS(FFV2_IDX_Y(tx));

    for (i = 0; i < len; i++) {
        int coeff = src[i]/qp;
        encode_golomb(e, FFABS(coeff));
        if (coeff)
            ff_daalaent_encode_bits(e, coeff < 0, 1);
    }
}

static int encode_block(FFV2EncCtx *s, DaalaEntropy *e, FFV2FCBuf *buf,
                        FFV2Block *blk)
{
    dctcoef temp [FFV2_SB_SIZE*FFV2_SB_SIZE];
    dctcoef temp2[FFV2_SB_SIZE*FFV2_SB_SIZE];
    const int b_stride = FFV2_IDX_TO_BS(FFV2_IDX_X(blk->type));

    ff_daalaent_encode_bits(e, FFV2_IDX_TX(blk->type), 4);

    for (int p = 0; p < s->planes; p++) {
        s->dsp.fwd_tx(&s->dsp, blk->type, temp, b_stride, blk->pix[p], buf->pix_stride[p]);
        s->dsp.raster_to_coding(temp2, temp, b_stride, blk->type);
        quant_block(temp2, blk->qp, blk->type, e);
    }

    return 0;
}

#define GET_SPLIT_TYPE(b)                                                                \
    ((b->subdiv[0] && b->subdiv[1] && b->subdiv[2] && b->subdiv[3])    ? FFV2_SPLIT_XY : \
    ((b->subdiv[0] && b->subdiv[2]) || (b->subdiv[1] && b->subdiv[3])) ? FFV2_SPLIT_Y :  \
    ((b->subdiv[0] && b->subdiv[1]) || (b->subdiv[2] && b->subdiv[3])) ? FFV2_SPLIT_X :  \
                                                                         FFV2_SPLIT_END)

static int encode_block_rec(FFV2EncCtx *s, DaalaEntropy *e, FFV2FCBuf *buf,
                            FFV2Block *blk)
{
    const int cdf_idx = 0;
    const int cdf_max = FFV2_SPLIT_NB;
    const int split = GET_SPLIT_TYPE(blk);
    if (!split && (FFV2_IDX_X(blk->type) == FFV2_TX_4) && (FFV2_IDX_Y(blk->type) == FFV2_TX_4))
        goto encode;
    ff_daalaent_encode_cdf_adapt(e, &s->subdiv_cdf, split, cdf_idx, cdf_max);
    switch (split) {
    case FFV2_SPLIT_END:
        goto encode;
    case FFV2_SPLIT_XY: {
        encode_block_rec(s, e, buf, blk->subdiv[0]);
        encode_block_rec(s, e, buf, blk->subdiv[1]);
        encode_block_rec(s, e, buf, blk->subdiv[2]);
        encode_block_rec(s, e, buf, blk->subdiv[3]);
        return 0;
    }
    case FFV2_SPLIT_Y: {
        encode_block_rec(s, e, buf, blk->subdiv[0] ? blk->subdiv[0] : blk->subdiv[1]);
        encode_block_rec(s, e, buf, blk->subdiv[2] ? blk->subdiv[2] : blk->subdiv[3]);
        return 0;
    }
    case FFV2_SPLIT_X: {
        encode_block_rec(s, e, buf, blk->subdiv[0] ? blk->subdiv[0] : blk->subdiv[2]);
        encode_block_rec(s, e, buf, blk->subdiv[1] ? blk->subdiv[1] : blk->subdiv[3]);
        return 0;
    }
    }

encode:
    return encode_block(s, e, buf, blk);
}

#if REGENERATE_ZIGZAGS
#include "libavutil/qsort.h"

typedef struct CoeffEntry {
    double val;
    int x_pos;
    int y_pos;
} CoeffEntry;

static int coeff_cmp(const void *a, const void *b)
{
    const CoeffEntry *aa = a;
    const CoeffEntry *bb = b;
    return bb->val - aa->val;
}
#endif

#include "libavutil/time.h"

static int decode_block_rec(FFV2EncCtx *s, DaalaEntropy *e, FFV2FCBuf *buf,
                            FFV2SB *sb, FFV2Block *blk, int o_x, int o_y,
                            enum FFV2TXSize s_x, enum FFV2TXSize s_y, int d)
{
    int split = FFV2_SPLIT_END;

    rec_free_block(blk);

    if ((s_x == FFV2_TX_4) || (s_y == FFV2_TX_4))
        goto decode;

    srand(av_gettime());

    //split = trunc((rand()/(float)RAND_MAX)*4);

    switch (split) {
    case FFV2_SPLIT_END:
        goto decode;
    case FFV2_SPLIT_XY: {
        const int l_x = FFV2_IDX_TO_BS(--s_x);
        const int l_y = FFV2_IDX_TO_BS(--s_y);
        blk->subdiv[0] = av_mallocz(sizeof(FFV2Block));
        blk->subdiv[1] = av_mallocz(sizeof(FFV2Block));
        blk->subdiv[2] = av_mallocz(sizeof(FFV2Block));
        blk->subdiv[3] = av_mallocz(sizeof(FFV2Block));
        decode_block_rec(s, e, buf, sb, blk->subdiv[0], o_x +   0, o_y +   0, s_x, s_y, d + 1);
        decode_block_rec(s, e, buf, sb, blk->subdiv[1], o_x + l_x, o_y +   0, s_x, s_y, d + 1);
        decode_block_rec(s, e, buf, sb, blk->subdiv[2], o_x +   0, o_y + l_y, s_x, s_y, d + 1);
        decode_block_rec(s, e, buf, sb, blk->subdiv[3], o_x + l_x, o_y + l_y, s_x, s_y, d + 1);
        return 0;
    }
    case FFV2_SPLIT_Y: {
        const int l_y = FFV2_IDX_TO_BS(--s_y);
        blk->subdiv[0] = av_mallocz(sizeof(FFV2Block));
        blk->subdiv[2] = av_mallocz(sizeof(FFV2Block));
        decode_block_rec(s, e, buf, sb, blk->subdiv[0], o_x +   0, o_y +   0, s_x, s_y, d + 1);
        decode_block_rec(s, e, buf, sb, blk->subdiv[2], o_x +   0, o_y + l_y, s_x, s_y, d + 1);
        return 0;
    }
    case FFV2_SPLIT_X: {
        const int l_x = FFV2_IDX_TO_BS(--s_x);
        blk->subdiv[0] = av_mallocz(sizeof(FFV2Block));
        blk->subdiv[1] = av_mallocz(sizeof(FFV2Block));
        decode_block_rec(s, e, buf, sb, blk->subdiv[0], o_x +   0, o_y +   0, s_x, s_y, d + 1);
        decode_block_rec(s, e, buf, sb, blk->subdiv[1], o_x + l_x, o_y +   0, s_x, s_y, d + 1);
        return 0;
    }
    }

decode:
    ffv2_splat_onto_grid(buf, blk, (sb->x + o_x) >> 2, (sb->y + o_y) >> 2);

    for (int p = 0; p < s->planes; p++) {
        blk->pix[p]   = sb->pix[p]   + o_y*buf->pix_stride[p]   + o_x;
        blk->coeff[p] = sb->coeff[p] + o_y*buf->coeff_stride[p] + o_x;
    }

    blk->qp   = s->qp;
    blk->type = FFV2_TX(s_x, s_y, FFV2_TX_DCT);

    return 0;
}

static void prefilter_sb_blocks_hor(FFV2EncCtx *s, FFV2FCBuf *buf, FFV2SB *sb)
{
    int fs = 0;
    int size = 0;
    for (int x = 0; x < 16; x += size) {
        FFV2Block *blk = ffv2_get_from_grid(buf, sb->x + x, sb->y);
        size = FFV2_IDX_TO_BS(FFV2_IDX_X(blk->type)) >> 2;
        fs = size;

        for (int p = 0; p < s->planes; p++)
            s->dsp.lap_prefilter_hor(blk->pix[p], buf->pix_stride[p], size << 2, fs << 2);
    }
}

static void prefilter_block(FFV2EncCtx *s, FFV2FCBuf *buf)
{
    for (int j = 0; j < s->num_sb_y; j++) {
        for (int i = 1; i < s->num_sb_x; i++) {
            for (int p = 0; p < s->planes; p++) {
                FFV2SB *sb = &buf->sbs[i][j];
                s->dsp.lap_prefilter_hor(sb->pix[p], buf->pix_stride[p], 64, 32);
            }
        }
    }

    for (int j = 1; j < s->num_sb_y; j++) {
        for (int i = 0; i < s->num_sb_x; i++) {
            for (int p = 0; p < s->planes; p++) {
                FFV2SB *sb = &buf->sbs[i][j];
                s->dsp.lap_prefilter_ver(sb->pix[p], buf->pix_stride[p], 64, 32);
            }
        }
    }
}

static void rdo_sbs(FFV2EncCtx *s, FFV2FCBuf *buf, DaalaEntropy *e)
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

            sb->x = i;
            sb->y = j;

            decode_block_rec(s, e, buf, sb, &sb->base, 0, 0, FFV2_TX_64, FFV2_TX_64, 0);
        }
        for (int p = 0; p < s->planes; p++) {
            offset_p[p] += buf->pix_stride[p]  *FFV2_SB_SIZE;
            offset_c[p] += buf->coeff_stride[p]*FFV2_SB_SIZE;
        }
    }

    prefilter_block(s, buf);

#if REGENERATE_ZIGZAGS
    for (int j = 0; j < s->num_sb_y; j++) {
        for (int i = 0; i < s->num_sb_x; i++) {
            for (int p = 0; p < s->planes; p++) {
                for (int x = 0; x < FFV2_TX_SIZE_NB; x++) {
                    for (int y = 0; y < FFV2_TX_SIZE_NB; y++) {
                        FFV2SB *sb = &buf->sbs[i][j];
                        int tx = FFV2_TX(x, y, FFV2_TX_DCT);
                        const int b_stride = FFV2_IDX_TO_BS(FFV2_IDX_X(tx));
                        const int y_len = FFV2_IDX_TO_BS(FFV2_IDX_Y(tx));
                        dctcoef temp [FFV2_SB_SIZE*FFV2_SB_SIZE] = { 0 };
                        CoeffEntry lul[FFV2_SB_SIZE*FFV2_SB_SIZE] = { { 0 } };

                        s->dsp.fwd_tx(&s->dsp, tx, temp, b_stride, sb->pix[p], buf->pix_stride[p]);

                        for (int i = 0; i < y_len; i++) {
                            for (int j = 0; j < b_stride; j++) {
                                CoeffEntry *l = &lul[i*b_stride + j];
                                l->x_pos = j;
                                l->y_pos = i;
                                l->val = FFABS(temp[i*b_stride + j]);
                            }
                        }
                        AV_QSORT(lul, y_len*b_stride, CoeffEntry, coeff_cmp);
                        for (int i = 0; i < y_len; i++) {
                            for (int j = 0; j < b_stride; j++) {
                                int num = i*b_stride + j;
                                CoeffEntry *l = &lul[num];
                                double points = 1/(double)num;
                                s->tx_score[x][y][l->x_pos][l->y_pos] += points;
                            }
                        }
                        s->num_f++;
                    }
                }
            }
        }
    }
#endif
}

static void encode_sbs(FFV2EncCtx *s, FFV2FCBuf *buf, DaalaEntropy *e)
{
    for (int j = 0; j < s->num_sb_y; j++) {
        for (int i = 0; i < s->num_sb_x; i++) {
            FFV2SB *sb = &buf->sbs[i][j];
            encode_block_rec(s, e, buf, &sb->base);
        }
    }
}

static void encode_frame_header(FFV2EncCtx *s, DaalaEntropy *e)
{
    ff_daalaent_encode_uint(e, s->avctx->pix_fmt, AV_PIX_FMT_NB);
    ff_daalaent_encode_uint(e, s->qp, 2048);
}

static int ffv2_encode_frame(AVCodecContext *avctx, AVPacket *avpkt,
                             const AVFrame *frame, int *got_packet)
{
    int ret;
    DaalaEntropy ent;
    FFV2EncCtx *s = avctx->priv_data;

    s->qp = 1024;

    daalaent_cdf_reset(&s->subdiv_cdf);

    if ((ret = ff_daalaent_encode_init(&ent, 0)))
        return ret;

    alloc_coeff_buf(s, &s->buf[0]);

    for (int p = 0; p < s->planes; p++)
        s->dsp.ref2coeff(s->buf[0].pix[p], s->buf[0].pix_stride[p],
                         frame->data[p], frame->linesize[p],
                         s->avctx->width, s->avctx->height);

    rdo_sbs(s, &s->buf[0], &ent);

#if !REGENERATE_ZIGZAGS
    encode_frame_header(s, &ent);
    encode_sbs(s, &s->buf[0], &ent);
#endif

    if ((ret = ff_daalaent_encode_done(&ent, avpkt)))
        return ret;

    free_coeff_buf(s, &s->buf[0]);

    av_log(avctx, AV_LOG_WARNING, "Packet size = %f kib\n", avpkt->size/1024.0f);

    *got_packet = 1;

    return 0;
}

static av_cold int ffv2enc_init(AVCodecContext *avctx)
{
    FFV2EncCtx *s = avctx->priv_data;

    s->avctx = avctx;
    s->bit_depth = av_pix_fmt_desc_get(avctx->pix_fmt)->comp[0].depth;
    s->planes = av_pix_fmt_count_planes(avctx->pix_fmt);

    s->num_sb_x = FFALIGN(s->avctx->width,  FFV2_SB_SIZE) / FFV2_SB_SIZE;
    s->num_sb_y = FFALIGN(s->avctx->height, FFV2_SB_SIZE) / FFV2_SB_SIZE;

    if (daalaent_cdf_alloc(&s->subdiv_cdf, 1, FFV2_SPLIT_NB, 128, 0, 2, 0))
        return AVERROR(ENOMEM);

    if (ff_ffv2dsp_init(&s->dsp, s->bit_depth))
        return AVERROR_UNKNOWN;

    return 0;
}

static av_cold int ffv2enc_close(AVCodecContext *avctx)
{
    FFV2EncCtx *s = avctx->priv_data;

    daalaent_cdf_free(&s->subdiv_cdf);

#if REGENERATE_ZIGZAGS
    FILE *out = fopen("libavcodec/zigzags.h", "w+");
    for (int x = 0; x < FFV2_TX_SIZE_NB; x++) {
        for (int y = 0; y < FFV2_TX_SIZE_NB; y++) {
            const int b_stride = FFV2_IDX_TO_BS(x);
            const int y_len = FFV2_IDX_TO_BS(y);

            int count = 0;

            CoeffEntry lul[FFV2_SB_SIZE*FFV2_SB_SIZE] = { { 0 } };
            for (int i = 0; i < y_len; i++) {
                for (int j = 0; j < b_stride; j++) {
                    CoeffEntry *l = &lul[i*b_stride + j];
                    l->x_pos = j;
                    l->y_pos = i;
                    l->val = lrintf(s->tx_score[x][y][j][i]);
                }
            }
            AV_QSORT(lul, y_len*b_stride, CoeffEntry, coeff_cmp);

            int min_a_x, min_a_y;
            if (x != y) {
                min_a_x = x > y ? FFV2_IDX_TO_BS(x - 1) : FFV2_IDX_TO_BS(x);
                min_a_y = y > x ? FFV2_IDX_TO_BS(y - 1) : FFV2_IDX_TO_BS(y);
            } else {
                min_a_x = min_a_y = FFV2_IDX_TO_BS(x - 1);
            }
            if (!x && !y)
                min_a_x = min_a_y = 0;

            fprintf(out, "static const FFV2BlockLayout layout_freq_%ix%i = {\n", FFV2_IDX_TO_BS(x), FFV2_IDX_TO_BS(y));
            fprintf(out, "    %i,\n", b_stride*y_len - min_a_x*min_a_y);
            fprintf(out, "    {\n        ");

            for (int i = 0; i < y_len; i++) {
                for (int j = 0; j < b_stride; j++) {
                    CoeffEntry *l = &lul[i*b_stride + j];

                    if (!((l->x_pos >= min_a_x) || (l->y_pos >= min_a_y)))
                        continue;

                    fprintf(out, "{ %i, %i }, ", l->x_pos, l->y_pos);

                    count++;
                    if (count > 8) {
                        fprintf(out, "\n        ");
                        count = 0;
                    }
                }   
            }
            
            fprintf(out, "\n    },\n");
            fprintf(out, "}; /* Over %lu frames */\n\n", s->num_f);
        }
    }
    fclose(out);
#endif

    return 0;
}

#define FFV2ENC_FLAGS (AV_OPT_FLAG_ENCODING_PARAM | AV_OPT_FLAG_VIDEO_PARAM)
static const AVOption ffv2enc_options[] = {
    {"ffv2_lossless", "Lossless encoding", offsetof(FFV2EncCtx, lossless), AV_OPT_TYPE_BOOL, {.i64 = 0}, 0, 1, FFV2ENC_FLAGS, "lossless"},
    {NULL}
};

static const AVClass ffv2enc_class = {
    .class_name = "FFv2 encoder",
    .category   = AV_CLASS_CATEGORY_ENCODER,
    .option     = ffv2enc_options,
    .item_name  = av_default_item_name,
    .version    = LIBAVUTIL_VERSION_INT,
};

static const enum AVPixelFormat allowed_pix_fmts[] = {
    AV_PIX_FMT_GBRP, AV_PIX_FMT_GBRP10, AV_PIX_FMT_GBRP12,
    AV_PIX_FMT_YUV444P, AV_PIX_FMT_YUV444P10, AV_PIX_FMT_YUV444P12,
    AV_PIX_FMT_GRAY8,
    AV_PIX_FMT_NONE
};

AVCodec ff_ffv2_encoder = {
    .name           = "ffv2",
    .long_name      = NULL_IF_CONFIG_SMALL("FFv2"),
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = AV_CODEC_ID_FFV2,
    .init           = ffv2enc_init,
    .encode2        = ffv2_encode_frame,
    .close          = ffv2enc_close,
    .priv_data_size = sizeof(FFV2EncCtx),
    .priv_class     = &ffv2enc_class,
    .pix_fmts       = allowed_pix_fmts,
    .capabilities   = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_EXPERIMENTAL,
    .caps_internal  = FF_CODEC_CAP_INIT_THREADSAFE |
                      FF_CODEC_CAP_INIT_CLEANUP,
};
