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

#define dctcoef int32_t

#include "daalatab.h"

#define FFV2_SB_SIZE 64
#define FFV2_PADDING 128
#define FFV2_REF_FRAMES 16

#define FFV2_TX_MIN  4

enum FFV2LapFlags {
    LAP_FILTER_SKIP_RIGHT  = 1 << 0,
    LAP_FILTER_SKIP_BOTTOM = 1 << 1,
    LAP_FILTER_DO_TOP      = 1 << 2,
    LAP_FILTER_DO_LEFT     = 1 << 3,
};

enum FFV2TXSize {
    FFV2_TX_4 = 0,
    FFV2_TX_8,
    FFV2_TX_16,
    FFV2_TX_32,
    FFV2_TX_64,

    FFV2_TX_SIZE_NB,
};

enum FFV2TXType {
    FFV2_TX_DCT = 0,
    FFV2_TX_DST,
    FFV2_TX_HAAR,

    FFV2_TX_TYPE_NB,
};

enum FFV2SplitType {
    FFV2_SPLIT_END = 0,
    FFV2_SPLIT_XY, /* Square split */
    FFV2_SPLIT_Y, /* Rectangular split in 2 partitions along the Y axis */
    FFV2_SPLIT_X, /* Rectangular split in 2 partitions along the X axis */

    FFV2_SPLIT_NB,
};

typedef struct FFV2Block {
    dctcoef *pix[4];
    dctcoef *coeff[4];

    uint32_t type;
    int qp;

    struct FFV2Block *subdiv[4];
} FFV2Block;

typedef struct FFV2SB {
    dctcoef *pix[4];
    dctcoef *coeff[4];

    int x, y;
    FFV2Block base;

    FFV2Block block[16][16];
} FFV2SB;

#define FFV2_TX_BITS (FFV2_TX_SIZE_NB - av_log2(FFV2_TX_MIN))

#define FFV2_BS_TO_IDX(SIZE) (av_log2(SIZE) - av_log2(FFV2_TX_MIN))
#define FFV2_IDX_TO_BS(IDX)  (1 << ((IDX) + av_log2(FFV2_TX_MIN)))

#define FFV2_TX(X_IDX, Y_IDX, TX) \
    ((X_IDX << 0*FFV2_TX_BITS) | (Y_IDX << 1*FFV2_TX_BITS) | (TX << 2*FFV2_TX_BITS))

#define FFV2_IDX_X(TX) (((TX) >> 0*FFV2_TX_BITS) & ((1 << FFV2_TX_BITS) - 1))
#define FFV2_IDX_Y(TX) (((TX) >> 1*FFV2_TX_BITS) & ((1 << FFV2_TX_BITS) - 1))
#define FFV2_IDX_TX(TX) ((TX) >> 2*FFV2_TX_BITS)

#define FFV2_MAX_TX FFV2_IDX_TO_BS(FFV2_TX_SIZE_NB - 1)

typedef struct FFV2FCBuf {
    dctcoef *pix[4];
    ptrdiff_t pix_stride[4];

    dctcoef *coeff[4];
    ptrdiff_t coeff_stride[4];

    FFV2SB **sbs;
    FFV2Block **grid;
    ptrdiff_t grid_stride;
} FFV2FCBuf;

static inline void ffv2_splat_onto_grid(FFV2FCBuf *buf, FFV2Block *blk,
                                        int ox, int oy)
{
    const int sx = FFV2_IDX_TO_BS(FFV2_IDX_X(blk->type)) >> 2;
    const int sy = FFV2_IDX_TO_BS(FFV2_IDX_X(blk->type)) >> 2;
    for (int y = oy; y < (oy + sy); y++)
        for (int x = ox; x < (ox + sx); x++)
            buf->grid[y*buf->grid_stride + x] = blk;
}

static inline FFV2Block *ffv2_get_from_grid(FFV2FCBuf *buf, int ox, int oy)
{
    return buf->grid[oy*buf->grid_stride + ox];
}

void ffv2_num_bands(int tx, int *bands_start, int *num_bands);

typedef struct FFV2DSP {
    void (*lap_prefilter_hor) (dctcoef *src, ptrdiff_t stride, int len, int lap_radius);
    void (*lap_postfilter_hor)(dctcoef *src, ptrdiff_t stride, int len, int lap_radius);

    void (*lap_prefilter_ver) (dctcoef *src, ptrdiff_t stride, int len, int lap_radius);
    void (*lap_postfilter_ver)(dctcoef *src, ptrdiff_t stride, int len, int lap_radius);

    void (*raster_to_coding)(dctcoef *dst, const dctcoef *src, int src_stride, int tx);
    void (*coding_to_raster)(dctcoef *dst, int dst_stride, const dctcoef *src, int tx);

    void (*ref2coeff)(dctcoef *dst, int dst_stride, const uint8_t *src,
                      ptrdiff_t src_stride, int src_width, int src_height);

    void (*coeff2ref)(uint8_t *dst, ptrdiff_t dst_stride, const dctcoef *src,
                      ptrdiff_t src_stride, int src_width, int src_height);

    void (*fwd_tx)(struct FFV2DSP *ctx, int tx, dctcoef *dst, int dst_stride, const dctcoef *src, int src_stride);
    void (*inv_tx)(struct FFV2DSP *ctx, int tx, dctcoef *dst, int dst_stride, const dctcoef *src, int src_stride);

    void (*fwd_tx_1d[FFV2_TX_SIZE_NB][FFV2_TX_TYPE_NB])(dctcoef y[], const dctcoef *x, int xstride);
    void (*inv_tx_1d[FFV2_TX_SIZE_NB][FFV2_TX_TYPE_NB])(dctcoef *x, int xstride, const dctcoef y[]);
} FFV2DSP;

int ff_ffv2dsp_init(FFV2DSP *ctx, int depth);
