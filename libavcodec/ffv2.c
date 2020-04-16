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

#include "ffv2.h"
#include "daalatab.h"
#include "libavutil/intreadwrite.h"

#define READ_BYTE(val) (*(val))
#define WRITE_BYTE(x, val) ((*(x)) = (val))

#define REF_2_COEFFS(READ_FN, DEPTH)                                                   \
static void ref_2_coeffs_ ##DEPTH (dctcoef *dst, int dst_stride,                       \
                                   const uint8_t *src, ptrdiff_t src_stride,           \
                                   int src_width, int src_height)                      \
{                                                                                      \
    const int bpp = 1 + (DEPTH > 8);                                                   \
    for (int i = 0; i < src_height; i++) {                                             \
        for (int j = 0; j < src_width; j++)                                            \
            dst[j] = ((int32_t)READ_FN(&src[j * bpp]) << (12 - DEPTH)) - 2048;         \
        src += src_stride;                                                             \
        dst += dst_stride;                                                             \
    }                                                                                  \
}

#define COEFFS_2_REF(WRITE_FN, DEPTH)                                                  \
static void coeffs_2_ref_ ##DEPTH (uint8_t *dst, ptrdiff_t dst_stride,                 \
                                   const dctcoef *src, ptrdiff_t src_stride,           \
                                   int src_width, int src_height)                      \
{                                                                                      \
    const int bpp = 1 + (DEPTH > 8);                                                   \
    for (int i = 0; i < src_height; i++) {                                             \
        for (int j = 0; j < src_width; j++)                                            \
            WRITE_FN(&dst[j * bpp], (src[j] + 2048) >> (12 - DEPTH));                  \
        src += src_stride;                                                             \
        dst += dst_stride;                                                             \
    }                                                                                  \
}

REF_2_COEFFS(READ_BYTE,  8);
REF_2_COEFFS(AV_RN16,   10);
REF_2_COEFFS(AV_RN16,   12);

COEFFS_2_REF(WRITE_BYTE, 8);
COEFFS_2_REF(AV_WN16,   10);
COEFFS_2_REF(AV_WN16,   12);

static void raster_to_coding(dctcoef *dst, const dctcoef *src,
                             int src_stride, int tx)
{
    int offset = 0;
    int bs_x = 0, bsize_x = FFV2_IDX_X(tx);
    int bs_y = 0, bsize_y = FFV2_IDX_Y(tx);

    while (1) {
        const FFV2BlockLayout *layout = ffv2_partition_layout_freq[bs_x][bs_y];
        for (int i = 0; i < layout->zigzag_len; i++)
            dst[offset + i] = src[layout->zigzag[i][1]*src_stride + layout->zigzag[i][0]];
        offset += layout->zigzag_len;
        if ((bs_x == bsize_x) && (bs_y == bsize_y))
            break;
        bs_x = FFMIN(bs_x + 1, bsize_x);
        bs_y = FFMIN(bs_y + 1, bsize_y);
    }
}

static void coding_to_raster(dctcoef *dst, int dst_stride,
                             const dctcoef *src, int tx)
{
    int offset = 0;
    int bs_x = 0, bsize_x = FFV2_IDX_X(tx);
    int bs_y = 0, bsize_y = FFV2_IDX_Y(tx);

    while (1) {
        const FFV2BlockLayout *layout = ffv2_partition_layout_freq[bs_x][bs_y];
        for (int i = 0; i < layout->zigzag_len; i++)
            dst[layout->zigzag[i][1]*dst_stride + layout->zigzag[i][0]] = src[offset + i];
        offset += layout->zigzag_len;
        if ((bs_x == bsize_x) && (bs_y == bsize_y))
            break;
        bs_x = FFMIN(bs_x + 1, bsize_x);
        bs_y = FFMIN(bs_y + 1, bsize_y);
    }
}

#if 1
#  define OD_DCT_OVERFLOW_CHECK(val, scale, offset, idx) \
  do { \
    assert((offset) >= 0); \
    if ((scale) > 0) { \
      if ((val) < INT_MIN/(scale)) { \
        av_log(NULL, AV_LOG_ERROR, "Overflow %2i: 0x%08X*0x%08X < INT_MIN\n", \
         (idx), (val), (scale)); \
      } \
      if ((val) > (INT_MAX - (offset))/(scale)) { \
        av_log(NULL, AV_LOG_ERROR, "Overflow %2i: 0x%08X*0x%04X + 0x%04X > INT_MAX\n", \
         (idx), (val), (scale), (offset)); \
      } \
    } \
    else if ((scale) < 0) { \
      if ((val) > (INT_MIN)/(scale)) { \
        av_log(NULL, AV_LOG_ERROR, "Overflow %2i: 0x%08X*-0x%04X < INT_MIN\n", \
         (idx), (val), (scale)); \
      } \
      if ((val) < (INT_MAX - (offset))/(scale)) { \
        av_log(NULL, AV_LOG_ERROR, "Overflow %2i: 0x%08X*-0x%08X + 0x%04X > INT_MAX\n", \
         (idx), (val), (scale), (offset)); \
      } \
    } \
  } \
  while(0)
# else
#  define OD_DCT_OVERFLOW_CHECK(val, scale, offset, idx)
# endif


/* 0, 2, 6, 14, 30 more than a given lap size */

static dctcoef lap_filt_params_4[4] = {
    85, 75, -15, 33,
};

static dctcoef lap_filt_params_8[10] = {
    93, 72, 73, 78, -28, -23, -10, 50, 37, 23,
};

static dctcoef lap_filt_params_16[22] = {
    94, 71, 68, 68, 68, 69, 70, 73, -32, -37, -36, -32, -26, -17, -7, 56, 49, 45,
    40, 34, 26, 15,
};

static dctcoef lap_filt_params_32[46] = {
    91, 70, 68, 67, 67, 67, 67, 66, 66, 67, 67, 66, 67, 67, 67, 70, -32, -41, -42,
    -41, -40, -38, -36, -34, -32, -29, -24, -19, -14, -9, -5, 58, 52, 50, 48, 45,
    43, 40, 38, 35, 32, 29, 24, 18, 13, 8,
};

static dctcoef lap_filt_params_64[94] = {
    91, 91, 70, 70, 68, 68, 67, 67, 67, 67, 67, 67, 67, 67, 66, 66, 66, 66, 67,
    67, 67, 67, 66, 66, 67, 67, 67, 67, 67, 67, 70, 70, -32, -32, -41, -41, -42,
    -42, -41, -41, -40, -40, -38, -38, -36, -36, -34, -34, -32, -32, -29, -29,
    -24, -24, -19, -19, -14, -14, -9, -9, -5, -5, 58, 58, 52, 52, 50, 50, 48, 48,
    45, 45, 43, 43, 40, 40, 38, 38, 35, 35, 32, 32, 29, 29, 24, 24, 18, 18, 13,
    13, 8, 8, 2, 2,
};

#define LAP_FILTER_PAIR(SIZE)                                                     \
static void fwd_lap_filter_ ##SIZE(dctcoef y[SIZE], const dctcoef x[SIZE]) {      \
    LOCAL_ALIGNED_32(dctcoef, t, [SIZE]);                                         \
                                                                                  \
    for (int i = 0; i < SIZE/2; i++)                                              \
        t[SIZE - 1 - i] = x[i] - x[SIZE - 1 - i];                                 \
    for (int i = 0; i < SIZE/2; i++)                                              \
        t[SIZE/2 - 1 - i] = x[SIZE/2 - 1 - i] - (t[SIZE/2 + i] >> 1);             \
                                                                                  \
    for (int i = SIZE/2; i < SIZE; i++) {                                         \
        OD_DCT_OVERFLOW_CHECK(t[i], lap_filt_params_ ##SIZE[i - SIZE/2], 0, i);   \
        t[i] = (t[i]*lap_filt_params_ ##SIZE[i - SIZE/2]) >> 6;                   \
        /* Equivalent to if (t[i] > 0) t[i]++; */                                 \
        t[i] += -t[i] >> ((sizeof(dctcoef) << 3) - 1) & 1;                        \
    }                                                                             \
                                                                                  \
    for (int i = SIZE - 1; i > SIZE/2; i--) {                                     \
        OD_DCT_OVERFLOW_CHECK(t[i - 1], lap_filt_params_ ##SIZE[i - 1], 32, i);   \
        t[i - 0] += (t[i - 1]*lap_filt_params_ ##SIZE[i - 1] + 32) >> 6;          \
        OD_DCT_OVERFLOW_CHECK(t[i], lap_filt_params_ ##SIZE[i + SIZE/2 - 2],      \
                              32, i);                                             \
        t[i - 1] += (t[i - 0]*lap_filt_params_ ##SIZE[i + SIZE/2 - 2] + 32) >> 6; \
    }                                                                             \
                                                                                  \
    for (int i = 0; i < SIZE/2; i++) {                                            \
        t[i] += t[SIZE - 1 - i] >> 1;                                             \
        y[i] = (dctcoef)t[i];                                                     \
    }                                                                             \
                                                                                  \
    for (int i = 0; i < SIZE/2; i++)                                              \
        y[SIZE/2 + i] = (dctcoef)(t[SIZE/2 - 1 - i] - t[SIZE/2 + i]);             \
}                                                                                 \
                                                                                  \
static void inv_lap_filter_ ##SIZE(dctcoef x[SIZE], const dctcoef y[SIZE]) {      \
    LOCAL_ALIGNED_32(dctcoef, t, [SIZE]);                                         \
                                                                                  \
    for (int i = 0; i < SIZE/2; i++)                                              \
        t[SIZE - 1 - i] = x[i] - x[SIZE - 1 - i];                                 \
    for (int i = 0; i < SIZE/2; i++)                                              \
        t[SIZE/2 - 1 - i] = x[SIZE/2 - 1 - i] - (t[SIZE/2 + i] >> 1);             \
                                                                                  \
    for (int i = SIZE/2; i < (SIZE - 1); i++) {                                   \
        t[i + 0] -= (t[i + 1]*lap_filt_params_ ##SIZE[i + SIZE/2 - 1] + 32) >> 6; \
        t[i + 1] -= (t[i + 0]*lap_filt_params_ ##SIZE[i             ] + 32) >> 6; \
    }                                                                             \
                                                                                  \
    for (int i = SIZE - 1; i >= SIZE/2; i--) /* TODO: fix per-sample divide! */   \
        t[i] = (t[i] << 6) / lap_filt_params_ ##SIZE[i - SIZE/2];                 \
                                                                                  \
    for (int i = 0; i < SIZE/2; i++) {                                            \
        t[i] += t[SIZE - 1 - i] >> 1;                                             \
        x[i] = (dctcoef)t[i];                                                     \
    }                                                                             \
                                                                                  \
    for (int i = SIZE/2; i < SIZE; i++)                                           \
        x[i] = (dctcoef)(t[SIZE - 1 - i] - t[i]);                                 \
}

LAP_FILTER_PAIR(4)
LAP_FILTER_PAIR(8)
LAP_FILTER_PAIR(16)
LAP_FILTER_PAIR(32)
LAP_FILTER_PAIR(64)

static void (*lap_pre_filters[])(dctcoef *, const dctcoef *) = {
    NULL,
    NULL,
    fwd_lap_filter_4,
    fwd_lap_filter_8,
    fwd_lap_filter_16,
    fwd_lap_filter_32,
    fwd_lap_filter_64,
};

static void (*lap_post_filters[])(dctcoef *, const dctcoef *) = {
    NULL,
    NULL,
    inv_lap_filter_4,
    inv_lap_filter_8,
    inv_lap_filter_16,
    inv_lap_filter_32,
    inv_lap_filter_64,
};

#define FILTER_HORIZONTAL(FILT, BSIZE, SRC, STRIDE)                            \
    do {                                                                       \
        for (int i = 0; i < BSIZE; i++)                                        \
            FILT((SRC) + i*STRIDE, (SRC) + i*STRIDE);                          \
    } while (0)

#define FILTER_VERTICAL(FILT, BSIZE, FSIZE, SRC, STRIDE)                       \
    do {                                                                       \
        LOCAL_ALIGNED_32(dctcoef, temp, [64]);                                 \
        for (int i = 0; i < BSIZE; i++) {                                      \
            for (int j = 0; j < FSIZE; j++)                                    \
                temp[j] = (SRC)[j*STRIDE + i];                                 \
            FILT(temp, temp);                                                  \
            for (int j = 0; j < FSIZE; j++)                                    \
                (SRC)[j*STRIDE + i] = temp[j];                                 \
        }                                                                      \
    } while (0)

static void lap_prefilter_hor(dctcoef *src, ptrdiff_t stride, int len, int lap_radius)
{
    void (*filt_h)(dctcoef *, const dctcoef *) = lap_pre_filters[av_log2(lap_radius)];
    src -= lap_radius >> 1;
    FILTER_HORIZONTAL(filt_h, len, src, stride);
}

static void lap_postfilter_hor(dctcoef *src, ptrdiff_t stride, int len, int lap_radius)
{
    void (*filt_h)(dctcoef *, const dctcoef *) = lap_post_filters[av_log2(lap_radius)];
    src -= lap_radius >> 1;
    FILTER_HORIZONTAL(filt_h, len, src, stride);
}

static void lap_prefilter_ver(dctcoef *src, ptrdiff_t stride, int len, int lap_radius)
{
    void (*filt_v)(dctcoef *, const dctcoef *) = lap_pre_filters[av_log2(lap_radius)];
    src -= (lap_radius >> 1)*stride;
    FILTER_VERTICAL(filt_v, len, lap_radius, src, stride);
}

static void lap_postfilter_ver(dctcoef *src, ptrdiff_t stride, int len, int lap_radius)
{
    void (*filt_v)(dctcoef *, const dctcoef *) = lap_post_filters[av_log2(lap_radius)];
    src -= (lap_radius >> 1)*stride;
    FILTER_VERTICAL(filt_v, len, lap_radius, src, stride);
}

#define OD_RSHIFT1(_a) (((_a) + ((_a) < 0)) >> 1)

/* Embedded 2-point orthonormal Type-II fDCT. */
#define OD_FDCT_2(t0, t1) \
do { \
    /* 13573/32768 ~= Tan[pi/8] ~= 0.414213562373095 */ \
    OD_DCT_OVERFLOW_CHECK(t1, 13573, 16384, 100); \
    t0 -= (t1*13573 + 16384) >> 15; \
    /* 5793/8192 ~= Sin[pi/4] ~= 0.707106781186547 */ \
    OD_DCT_OVERFLOW_CHECK(t0, 5793, 4096, 101); \
    t1 += (t0*5793 + 4096) >> 13; \
    /* 3393/8192 ~= Tan[pi/8] ~= 0.414213562373095 */ \
    OD_DCT_OVERFLOW_CHECK(t1, 3393, 4096, 102); \
    t0 -= (t1*3393 + 4096) >> 13; \
} while (0)

/* Embedded 2-point orthonormal Type-II iDCT. */
#define OD_IDCT_2(t0, t1) \
do { \
    /* 3393/8192 ~= Tan[pi/8] ~= 0.414213562373095 */ \
    t0 += (t1*3393 + 4096) >> 13; \
    /* 5793/8192 ~= Sin[pi/4] ~= 0.707106781186547 */ \
    t1 -= (t0*5793 + 4096) >> 13; \
    /* 13573/32768 ~= Tan[pi/8] ~= 0.414213562373095 */ \
    t0 += (t1*13573 + 16384) >> 15; \
} while (0)

/* Embedded 2-point asymmetric Type-II fDCT. */
#define OD_FDCT_2_ASYM(p0, p1, p1h) \
do { \
    p0 += p1h; \
    p1 = p0 - p1; \
} while (0)

/* Embedded 2-point asymmetric Type-II iDCT. */
#define OD_IDCT_2_ASYM(p0, p1, p1h) \
do { \
    p1 = p0 - p1; \
    p1h = OD_RSHIFT1(p1); \
    p0 -= p1h; \
} while (0)

/* Embedded 2-point orthonormal Type-IV fDST. */
#define OD_FDST_2(t0, t1) \
do { \
    /* 10947/16384 ~= Tan[3*Pi/16] ~= 0.668178637919299 */ \
    OD_DCT_OVERFLOW_CHECK(t1, 10947, 8192, 103); \
    t0 -= (t1*10947 + 8192) >> 14; \
    /* 473/512 ~= Sin[3*Pi/8] ~= 0.923879532511287 */ \
    OD_DCT_OVERFLOW_CHECK(t0, 473, 256, 104); \
    t1 += (t0*473 + 256) >> 9; \
    /* 10947/16384 ~= Tan[3*Pi/16] ~= 0.668178637919299 */ \
    OD_DCT_OVERFLOW_CHECK(t1, 10947, 8192, 105); \
    t0 -= (t1*10947 + 8192) >> 14; \
} while (0)

/* Embedded 2-point orthonormal Type-IV iDST. */
#define OD_IDST_2(t0, t1) \
do { \
    /* 10947/16384 ~= Tan[3*Pi/16]) ~= 0.668178637919299 */ \
    t0 += (t1*10947 + 8192) >> 14; \
    /* 473/512 ~= Sin[3*Pi/8] ~= 0.923879532511287 */ \
    t1 -= (t0*473 + 256) >> 9; \
    /* 10947/16384 ~= Tan[3*Pi/16] ~= 0.668178637919299 */ \
    t0 += (t1*10947 + 8192) >> 14; \
} while (0)

/* Embedded 2-point asymmetric Type-IV fDST. */
#define OD_FDST_2_ASYM(p0, p1) \
do { \
    /* 11507/16384 ~= 4*Sin[Pi/8] - 2*Tan[Pi/8] ~= 0.702306604714169 */ \
    OD_DCT_OVERFLOW_CHECK(p1, 11507, 8192, 187); \
    p0 -= (p1*11507 + 8192) >> 14; \
    /* 669/1024 ~= Cos[Pi/8]/Sqrt[2] ~= 0.653281482438188 */ \
    OD_DCT_OVERFLOW_CHECK(p0, 669, 512, 188); \
    p1 += (p0*669 + 512) >> 10; \
    /* 4573/4096 ~= 4*Sin[Pi/8] - Tan[Pi/8] ~= 1.11652016708726 */ \
    OD_DCT_OVERFLOW_CHECK(p1, 4573, 2048, 189); \
    p0 -= (p1*4573 + 2048) >> 12; \
} while (0)

/* Embedded 2-point asymmetric Type-IV iDST. */ \
#define OD_IDST_2_ASYM(p0, p1) \
do { \
    /* 4573/4096 ~= 4*Sin[Pi/8] - Tan[Pi/8] ~= 1.11652016708726 */ \
    p0 += (p1*4573 + 2048) >> 12; \
    /* 669/1024 ~= Cos[Pi/8]/Sqrt[2] ~= 0.653281482438188 */ \
    p1 -= (p0*669 + 512) >> 10; \
    /* 11507/16384 ~= 4*Sin[Pi/8] - 2*Tan[Pi/8] ~= 0.702306604714169 */ \
    p0 += (p1*11507 + 8192) >> 14; \
} while (0)

/* Embedded 4-point orthonormal Type-II fDCT. */
#define OD_FDCT_4(q0, q2, q1, q3) \
do { \
    dctcoef q2h; \
    dctcoef q3h; \
    q3 = q0 - q3; \
    q3h = OD_RSHIFT1(q3); \
    q0 -= q3h; \
    q2 += q1; \
    q2h = OD_RSHIFT1(q2); \
    q1 = q2h - q1; \
    OD_FDCT_2_ASYM(q0, q2, q2h); \
    OD_FDST_2_ASYM(q3, q1); \
} while (0)

/* Embedded 4-point orthonormal Type-II iDCT. */
#define OD_IDCT_4(q0, q2, q1, q3) \
do { \
    dctcoef q1h; \
    dctcoef q3h; \
    OD_IDST_2_ASYM(q3, q2); \
    OD_IDCT_2_ASYM(q0, q1, q1h); \
    q3h = OD_RSHIFT1(q3); \
    q0 += q3h; \
    q3 = q0 - q3; \
    q2 = q1h - q2; \
    q1 -= q2; \
} while (0)

/* Embedded 4-point asymmetric Type-II fDCT. */
#define OD_FDCT_4_ASYM(t0, t2, t2h, t1, t3, t3h) \
do { \
    t0 += t3h; \
    t3 = t0 - t3; \
    t1 = t2h - t1; \
    t2 = t1 - t2; \
    OD_FDCT_2(t0, t2); \
    OD_FDST_2(t3, t1); \
} while (0)

/* Embedded 4-point asymmetric Type-II iDCT. */
#define OD_IDCT_4_ASYM(t0, t2, t1, t1h, t3, t3h) \
do { \
    OD_IDST_2(t3, t2); \
    OD_IDCT_2(t0, t1); \
    t1 = t2 - t1; \
    t1h = OD_RSHIFT1(t1); \
    t2 = t1h - t2; \
    t3 = t0 - t3; \
    t3h = OD_RSHIFT1(t3); \
    t0 -= t3h; \
} while (0)

/* Embedded 4-point orthonormal Type-IV fDST. */
#define OD_FDST_4(q0, q2, q1, q3) \
do { \
    dctcoef q0h; \
    dctcoef q1h; \
    /* 13573/32768 ~= Tan[Pi/8] ~= 0.414213562373095 */ \
    OD_DCT_OVERFLOW_CHECK(q1, 13573, 16384, 190); \
    q2 += (q1*13573 + 16384) >> 15; \
    /* 5793/8192 ~= Sin[Pi/4] ~= 0.707106781186547 */ \
    OD_DCT_OVERFLOW_CHECK(q2, 5793, 4096, 191); \
    q1 -= (q2*5793 + 4096) >> 13; \
    /* 3393/8192 ~= Tan[Pi/8] ~= 0.414213562373095 */ \
    OD_DCT_OVERFLOW_CHECK(q1, 3393, 4096, 192); \
    q2 += (q1*3393 + 4096) >> 13; \
    q0 += q2; \
    q0h = OD_RSHIFT1(q0); \
    q2 = q0h - q2; \
    q1 += q3; \
    q1h = OD_RSHIFT1(q1); \
    q3 -= q1h; \
    /* 537/1024 ~= (1/Sqrt[2] - Cos[3*Pi/16]/2)/Sin[3*Pi/16] ~=
        0.524455699240090 */ \
    OD_DCT_OVERFLOW_CHECK(q1, 537, 512, 193); \
    q2 -= (q1*537 + 512) >> 10; \
    /* 1609/2048 ~= Sqrt[2]*Sin[3*Pi/16] ~= 0.785694958387102 */ \
    OD_DCT_OVERFLOW_CHECK(q2, 1609, 1024, 194); \
    q1 += (q2*1609 + 1024) >> 11; \
    /* 7335/32768 ~= (1/Sqrt[2] - Cos[3*Pi/16])/Sin[3*Pi/16] ~=
        0.223847182092655 */ \
    OD_DCT_OVERFLOW_CHECK(q1, 7335, 16384, 195); \
    q2 += (q1*7335 + 16384) >> 15; \
    /* 5091/8192 ~= (1/Sqrt[2] - Cos[7*Pi/16]/2)/Sin[7*Pi/16] ~=
        0.6215036383171189 */ \
    OD_DCT_OVERFLOW_CHECK(q0, 5091, 4096, 196); \
    q3 += (q0*5091 + 4096) >> 13; \
    /* 5681/4096 ~= Sqrt[2]*Sin[7*Pi/16] ~= 1.38703984532215 */ \
    OD_DCT_OVERFLOW_CHECK(q3, 5681, 2048, 197); \
    q0 -= (q3*5681 + 2048) >> 12; \
    /* 4277/8192 ~= (1/Sqrt[2] - Cos[7*Pi/16])/Sin[7*Pi/16] ~=
        0.52204745462729 */ \
    OD_DCT_OVERFLOW_CHECK(q0, 4277, 4096, 198); \
    q3 += (q0*4277 + 4096) >> 13; \
} while (0)

/* Embedded 4-point orthonormal Type-IV iDST. */
#define OD_IDST_4(q0, q2, q1, q3) \
do { \
    dctcoef q0h; \
    dctcoef q2h; \
    /* 4277/8192 ~= (1/Sqrt[2] - Cos[7*Pi/16])/Sin[7*Pi/16] ~=
        0.52204745462729 */ \
    q3 -= (q0*4277 + 4096) >> 13; \
    /* 5681/4096 ~= Sqrt[2]*Sin[7*Pi/16] ~= 1.38703984532215 */ \
    q0 += (q3*5681 + 2048) >> 12; \
    /* 5091/8192 ~= (1/Sqrt[2] - Cos[7*Pi/16]/2)/Sin[7*Pi/16] ~=
        0.6215036383171189 */ \
    q3 -= (q0*5091 + 4096) >> 13; \
    /* 7335/32768 ~= (1/Sqrt[2] - Cos[3*Pi/16])/Sin[3*Pi/16] ~=
        0.223847182092655 */ \
    q1 -= (q2*7335 + 16384) >> 15; \
    /* 1609/2048 ~= Sqrt[2]*Sin[3*Pi/16] ~= 0.785694958387102 */ \
    q2 -= (q1*1609 + 1024) >> 11; \
    /* 537/1024 ~= (1/Sqrt[2] - Cos[3*Pi/16]/2)/Sin[3*Pi/16] ~=
        0.524455699240090 */ \
    q1 += (q2*537 + 512) >> 10; \
    q2h = OD_RSHIFT1(q2); \
    q3 += q2h; \
    q2 -= q3; \
    q0h = OD_RSHIFT1(q0); \
    q1 = q0h - q1; \
    q0 -= q1; \
    /* 3393/8192 ~= Tan[Pi/8] ~= 0.414213562373095 */ \
    q1 -= (q2*3393 + 4096) >> 13; \
    /* 5793/8192 ~= Sin[Pi/4] ~= 0.707106781186547 */ \
    q2 += (q1*5793 + 4096) >> 13; \
    /* 13573/32768 ~= Tan[Pi/8] ~= 0.414213562373095 */ \
    q1 -= (q2*13573 + 16384) >> 15; \
} while (0)

/* Embedded 4-point asymmetric Type-IV fDST. */
#define OD_FDST_4_ASYM(t0, t0h, t2, t1, t3) \
do { \
    /* 7489/8192 ~= Tan[Pi/8] + Tan[Pi/4]/2 ~= 0.914213562373095 */ \
    OD_DCT_OVERFLOW_CHECK(t1, 7489, 4096, 106); \
    t2 -= (t1*7489 + 4096) >> 13; \
    /* 11585/16384 ~= Sin[Pi/4] ~= 0.707106781186548 */ \
    OD_DCT_OVERFLOW_CHECK(t1, 11585, 8192, 107); \
    t1 += (t2*11585 + 8192) >> 14; \
    /* -19195/32768 ~= Tan[Pi/8] - Tan[Pi/4] ~= -0.585786437626905 */ \
    OD_DCT_OVERFLOW_CHECK(t1, 19195, 16384, 108); \
    t2 += (t1*19195 + 16384) >> 15; \
    t3 += OD_RSHIFT1(t2); \
    t2 -= t3; \
    t1 = t0h - t1; \
    t0 -= t1; \
    /* 6723/8192 ~= Tan[7*Pi/32] ~= 0.820678790828660 */ \
    OD_DCT_OVERFLOW_CHECK(t0, 6723, 4096, 109); \
    t3 += (t0*6723 + 4096) >> 13; \
    /* 8035/8192 ~= Sin[7*Pi/16] ~= 0.980785280403230 */ \
    OD_DCT_OVERFLOW_CHECK(t3, 8035, 4096, 110); \
    t0 -= (t3*8035 + 4096) >> 13; \
    /* 6723/8192 ~= Tan[7*Pi/32] ~= 0.820678790828660 */ \
    OD_DCT_OVERFLOW_CHECK(t0, 6723, 4096, 111); \
    t3 += (t0*6723 + 4096) >> 13; \
    /* 8757/16384 ~= Tan[5*Pi/32] ~= 0.534511135950792 */ \
    OD_DCT_OVERFLOW_CHECK(t1, 8757, 8192, 112); \
    t2 += (t1*8757 + 8192) >> 14; \
    /* 6811/8192 ~= Sin[5*Pi/16] ~= 0.831469612302545 */ \
    OD_DCT_OVERFLOW_CHECK(t2, 6811, 4096, 113); \
    t1 -= (t2*6811 + 4096) >> 13; \
    /* 8757/16384 ~= Tan[5*Pi/32] ~= 0.534511135950792 */ \
    OD_DCT_OVERFLOW_CHECK(t1, 8757, 8192, 114); \
    t2 += (t1*8757 + 8192) >> 14; \
} while (0)

/* Embedded 4-point asymmetric Type-IV iDST. */
#define OD_IDST_4_ASYM(t0, t0h, t2, t1, t3) \
do { \
    /* 8757/16384 ~= Tan[5*Pi/32] ~= 0.534511135950792 */ \
    t1 -= (t2*8757 + 8192) >> 14; \
    /* 6811/8192 ~= Sin[5*Pi/16] ~= 0.831469612302545 */ \
    t2 += (t1*6811 + 4096) >> 13; \
    /* 8757/16384 ~= Tan[5*Pi/32] ~= 0.534511135950792 */ \
    t1 -= (t2*8757 + 8192) >> 14; \
    /* 6723/8192 ~= Tan[7*Pi/32] ~= 0.820678790828660 */ \
    t3 -= (t0*6723 + 4096) >> 13; \
    /* 8035/8192 ~= Sin[7*Pi/16] ~= 0.980785280403230 */ \
    t0 += (t3*8035 + 4096) >> 13; \
    /* 6723/8192 ~= Tan[7*Pi/32] ~= 0.820678790828660 */ \
    t3 -= (t0*6723 + 4096) >> 13; \
    t0 += t2; \
    t0h = OD_RSHIFT1(t0); \
    t2 = t0h - t2; \
    t1 += t3; \
    t3 -= OD_RSHIFT1(t1); \
    /* -19195/32768 ~= Tan[Pi/8] - Tan[Pi/4] ~= -0.585786437626905 */ \
    t1 -= (t2*19195 + 16384) >> 15; \
    /* 11585/16384 ~= Sin[Pi/4] ~= 0.707106781186548 */ \
    t2 -= (t1*11585 + 8192) >> 14; \
    /* 7489/8192 ~= Tan[Pi/8] + Tan[Pi/4]/2 ~= 0.914213562373095 */ \
    t1 += (t2*7489 + 4096) >> 13; \
} while (0)

/* Embedded 8-point orthonormal Type-II fDCT. */
#define OD_FDCT_8(r0, r4, r2, r6, r1, r5, r3, r7) \
do { \
    dctcoef r4h; \
    dctcoef r5h; \
    dctcoef r6h; \
    dctcoef r7h; \
    r7 = r0 - r7; \
    r7h = OD_RSHIFT1(r7); \
    r0 -= r7h; \
    r6 += r1; \
    r6h = OD_RSHIFT1(r6); \
    r1 = r6h - r1; \
    r5 = r2 - r5; \
    r5h = OD_RSHIFT1(r5); \
    r2 -= r5h; \
    r4 += r3; \
    r4h = OD_RSHIFT1(r4); \
    r3 = r4h - r3; \
    OD_FDCT_4_ASYM(r0, r4, r4h, r2, r6, r6h); \
    OD_FDST_4_ASYM(r7, r7h, r3, r5, r1); \
} while (0)

/* Embedded 8-point orthonormal Type-II iDCT. */
#define OD_IDCT_8(r0, r4, r2, r6, r1, r5, r3, r7) \
do { \
    dctcoef r1h; \
    dctcoef r3h; \
    dctcoef r5h; \
    dctcoef r7h; \
    OD_IDST_4_ASYM(r7, r7h, r5, r6, r4); \
    OD_IDCT_4_ASYM(r0, r2, r1, r1h, r3, r3h); \
    r0 += r7h; \
    r7 = r0 - r7; \
    r6 = r1h - r6; \
    r1 -= r6; \
    r5h = OD_RSHIFT1(r5); \
    r2 += r5h; \
    r5 = r2 - r5; \
    r4 = r3h - r4; \
    r3 -= r4; \
} while (0)

/* Embedded 8-point asymmetric Type-II fDCT. */
#define OD_FDCT_8_ASYM(r0, r4, r4h, r2, r6, r6h, r1, r5, r5h, r3, r7, r7h) \
do { \
    r0 += r7h; \
    r7 = r0 - r7; \
    r1 = r6h - r1; \
    r6 -= r1; \
    r2 += r5h; \
    r5 = r2 - r5; \
    r3 = r4h - r3; \
    r4 -= r3; \
    OD_FDCT_4(r0, r4, r2, r6); \
    OD_FDST_4(r7, r3, r5, r1); \
} while (0)

/* Embedded 8-point asymmetric Type-II iDCT. */
#define OD_IDCT_8_ASYM(r0, r4, r2, r6, r1, r1h, r5, r5h, r3, r3h, r7, r7h) \
do { \
    OD_IDST_4(r7, r5, r6, r4); \
    OD_IDCT_4(r0, r2, r1, r3); \
    r7 = r0 - r7; \
    r7h = OD_RSHIFT1(r7); \
    r0 -= r7h; \
    r1 += r6; \
    r1h = OD_RSHIFT1(r1); \
    r6 = r1h - r6; \
    r5 = r2 - r5; \
    r5h = OD_RSHIFT1(r5); \
    r2 -= r5h; \
    r3 += r4; \
    r3h = OD_RSHIFT1(r3); \
    r4 = r3h - r4; \
} while (0)

/* Embedded 8-point orthonormal Type-IV fDST. */
#define OD_FDST_8(t0, t4, t2, t6, t1, t5, t3, t7) \
do { \
    dctcoef t0h; \
    dctcoef t2h; \
    dctcoef t5h; \
    dctcoef t7h; \
    /* 13573/32768 ~= Tan[Pi/8] ~= 0.414213562373095 */ \
    OD_DCT_OVERFLOW_CHECK(t1, 13573, 16384, 115); \
    t6 -= (t1*13573 + 16384) >> 15; \
    /* 11585/16384 ~= Sin[Pi/4] ~= 0.707106781186547 */ \
    OD_DCT_OVERFLOW_CHECK(t6, 11585, 8192, 116); \
    t1 += (t6*11585 + 8192) >> 14; \
    /* 13573/32768 ~= Tan[Pi/8] ~= 0.414213562373095 */ \
    OD_DCT_OVERFLOW_CHECK(t1, 13573, 16384, 117); \
    t6 -= (t1*13573 + 16384) >> 15; \
    /* 21895/32768 ~= Tan[3*Pi/16] ~= 0.668178637919299 */ \
    OD_DCT_OVERFLOW_CHECK(t2, 21895, 16384, 118); \
    t5 -= (t2*21895 + 16384) >> 15; \
    /* 15137/16384 ~= Sin[3*Pi/8] ~= 0.923879532511287 */ \
    OD_DCT_OVERFLOW_CHECK(t5, 15137, 8192, 119); \
    t2 += (t5*15137 + 8192) >> 14; \
    /* 10947/16384 ~= Tan[3*Pi/16] ~= 0.668178637919299 */ \
    OD_DCT_OVERFLOW_CHECK(t2, 10947, 8192, 120); \
    t5 -= (t2*10947 + 8192) >> 14; \
    /* 3259/16384 ~= Tan[Pi/16] ~= 0.198912367379658 */ \
    OD_DCT_OVERFLOW_CHECK(t3, 3259, 8192, 121); \
    t4 -= (t3*3259 + 8192) >> 14; \
    /* 3135/8192 ~= Sin[Pi/8] ~= 0.382683432365090 */ \
    OD_DCT_OVERFLOW_CHECK(t4, 3135, 4096, 122); \
    t3 += (t4*3135 + 4096) >> 13; \
    /* 3259/16384 ~= Tan[Pi/16] ~= 0.198912367379658 */ \
    OD_DCT_OVERFLOW_CHECK(t3, 3259, 8192, 123); \
    t4 -= (t3*3259 + 8192) >> 14; \
    t7 += t1; \
    t7h = OD_RSHIFT1(t7); \
    t1 -= t7h; \
    t2 = t3 - t2; \
    t2h = OD_RSHIFT1(t2); \
    t3 -= t2h; \
    t0 -= t6; \
    t0h = OD_RSHIFT1(t0); \
    t6 += t0h; \
    t5 = t4 - t5; \
    t5h = OD_RSHIFT1(t5); \
    t4 -= t5h; \
    t1 += t5h; \
    t5 = t1 - t5; \
    t4 += t0h; \
    t0 -= t4; \
    t6 -= t2h; \
    t2 += t6; \
    t3 -= t7h; \
    t7 += t3; \
    /* TODO: Can we move this into another operation */ \
    t7 = -t7; \
    /* 7425/8192 ~= Tan[15*Pi/64] ~= 0.906347169019147 */ \
    OD_DCT_OVERFLOW_CHECK(t7, 7425, 4096, 124); \
    t0 -= (t7*7425 + 4096) >> 13; \
    /* 8153/8192 ~= Sin[15*Pi/32] ~= 0.995184726672197 */ \
    OD_DCT_OVERFLOW_CHECK(t0, 8153, 4096, 125); \
    t7 += (t0*8153 + 4096) >> 13; \
    /* 7425/8192 ~= Tan[15*Pi/64] ~= 0.906347169019147 */ \
    OD_DCT_OVERFLOW_CHECK(t7, 7425, 4096, 126); \
    t0 -= (t7*7425 + 4096) >> 13; \
    /* 4861/32768 ~= Tan[3*Pi/64] ~= 0.148335987538347 */ \
    OD_DCT_OVERFLOW_CHECK(t1, 4861, 16384, 127); \
    t6 -= (t1*4861 + 16384) >> 15; \
    /* 1189/4096 ~= Sin[3*Pi/32] ~= 0.290284677254462 */ \
    OD_DCT_OVERFLOW_CHECK(t6, 1189, 2048, 128); \
    t1 += (t6*1189 + 2048) >> 12; \
    /* 4861/32768 ~= Tan[3*Pi/64] ~= 0.148335987538347 */ \
    OD_DCT_OVERFLOW_CHECK(t1, 4861, 16384, 129); \
    t6 -= (t1*4861 + 16384) >> 15; \
    /* 2455/4096 ~= Tan[11*Pi/64] ~= 0.599376933681924 */ \
    OD_DCT_OVERFLOW_CHECK(t5, 2455, 2048, 130); \
    t2 -= (t5*2455 + 2048) >> 12; \
    /* 7225/8192 ~= Sin[11*Pi/32] ~= 0.881921264348355 */ \
    OD_DCT_OVERFLOW_CHECK(t2, 7225, 4096, 131); \
    t5 += (t2*7225 + 4096) >> 13; \
    /* 2455/4096 ~= Tan[11*Pi/64] ~= 0.599376933681924 */ \
    OD_DCT_OVERFLOW_CHECK(t5, 2455, 2048, 132); \
    t2 -= (t5*2455 + 2048) >> 12; \
    /* 11725/32768 ~= Tan[7*Pi/64] ~= 0.357805721314524 */ \
    OD_DCT_OVERFLOW_CHECK(t3, 11725, 16384, 133); \
    t4 -= (t3*11725 + 16384) >> 15; \
    /* 5197/8192 ~= Sin[7*Pi/32] ~= 0.634393284163645 */ \
    OD_DCT_OVERFLOW_CHECK(t4, 5197, 4096, 134); \
    t3 += (t4*5197 + 4096) >> 13; \
    /* 11725/32768 ~= Tan[7*Pi/64] ~= 0.357805721314524 */ \
    OD_DCT_OVERFLOW_CHECK(t3, 11725, 16384, 135); \
    t4 -= (t3*11725 + 16384) >> 15; \
} while (0)

/* Embedded 8-point orthonormal Type-IV iDST. */
#define OD_IDST_8(t0, t4, t2, t6, t1, t5, t3, t7) \
do { \
    dctcoef t0h; \
    dctcoef t2h; \
    dctcoef t5h_; \
    dctcoef t7h_; \
    /* 11725/32768 ~= Tan[7*Pi/64] ~= 0.357805721314524 */ \
    t1 += (t6*11725 + 16384) >> 15; \
    /* 5197/8192 ~= Sin[7*Pi/32] ~= 0.634393284163645 */ \
    t6 -= (t1*5197 + 4096) >> 13; \
    /* 11725/32768 ~= Tan[7*Pi/64] ~= 0.357805721314524 */ \
    t1 += (t6*11725 + 16384) >> 15; \
    /* 2455/4096 ~= Tan[11*Pi/64] ~= 0.599376933681924 */ \
    t2 += (t5*2455 + 2048) >> 12; \
    /* 7225/8192 ~= Sin[11*Pi/32] ~= 0.881921264348355 */ \
    t5 -= (t2*7225 + 4096) >> 13; \
    /* 2455/4096 ~= Tan[11*Pi/64] ~= 0.599376933681924 */ \
    t2 += (t5*2455 + 2048) >> 12; \
    /* 4861/32768 ~= Tan[3*Pi/64] ~= 0.148335987538347 */ \
    t3 += (t4*4861 + 16384) >> 15; \
    /* 1189/4096 ~= Sin[3*Pi/32] ~= 0.290284677254462 */ \
    t4 -= (t3*1189 + 2048) >> 12; \
    /* 4861/32768 ~= Tan[3*Pi/64] ~= 0.148335987538347 */ \
    t3 += (t4*4861 + 16384) >> 15; \
    /* 7425/8192 ~= Tan[15*Pi/64] ~= 0.906347169019147 */ \
    t0 += (t7*7425 + 4096) >> 13; \
    /* 8153/8192 ~= Sin[15*Pi/32] ~= 0.995184726672197 */ \
    t7 -= (t0*8153 + 4096) >> 13; \
    /* 7425/8192 ~= Tan[15*Pi/64] ~= 0.906347169019147 */ \
    t0 += (t7*7425 + 4096) >> 13; \
    /* TODO: Can we move this into another operation */ \
    t7 = -t7; \
    t7 -= t6; \
    t7h_ = OD_RSHIFT1(t7); \
    t6 += t7h_; \
    t2 -= t3; \
    t2h = OD_RSHIFT1(t2); \
    t3 += t2h; \
    t0 += t1; \
    t0h = OD_RSHIFT1(t0); \
    t1 -= t0h; \
    t5 = t4 - t5; \
    t5h_ = OD_RSHIFT1(t5); \
    t4 -= t5h_; \
    t1 += t5h_; \
    t5 = t1 - t5; \
    t3 -= t0h; \
    t0 += t3; \
    t6 += t2h; \
    t2 = t6 - t2; \
    t4 += t7h_; \
    t7 -= t4; \
    /* 3259/16384 ~= Tan[Pi/16] ~= 0.198912367379658 */ \
    t1 += (t6*3259 + 8192) >> 14; \
    /* 3135/8192 ~= Sin[Pi/8] ~= 0.382683432365090 */ \
    t6 -= (t1*3135 + 4096) >> 13; \
    /* 3259/16384 ~= Tan[Pi/16] ~= 0.198912367379658 */ \
    t1 += (t6*3259 + 8192) >> 14; \
    /* 10947/16384 ~= Tan[3*Pi/16] ~= 0.668178637919299 */ \
    t5 += (t2*10947 + 8192) >> 14; \
    /* 15137/16384 ~= Sin[3*Pi/8] ~= 0.923879532511287 */ \
    t2 -= (t5*15137 + 8192) >> 14; \
    /* 21895/32768 ~= Tan[3*Pi/16] ~= 0.668178637919299 */ \
    t5 += (t2*21895 + 16384) >> 15; \
    /* 13573/32768 ~= Tan[Pi/8] ~= 0.414213562373095 */ \
    t3 += (t4*13573 + 16384) >> 15; \
    /* 11585/16384 ~= Sin[Pi/4] ~= 0.707106781186547 */ \
    t4 -= (t3*11585 + 8192) >> 14; \
    /* 13573/32768 ~= Tan[Pi/8] ~= 0.414213562373095 */ \
    t3 += (t4*13573 + 16384) >> 15; \
} while (0)

/* Embedded 8-point asymmetric Type-IV fDST. */ \
/* TODO: Rewrite this so that t0h can be passed in. */
#define OD_FDST_8_ASYM(t0, t4, t2, t6, t1, t5, t3, t7) \
do { \
    dctcoef t0h; \
    dctcoef t2h; \
    dctcoef t5h; \
    dctcoef t7h; \
    /* 1035/2048 ~= (Sqrt[2] - Cos[7*Pi/32])/(2*Sin[7*Pi/32]) ~=
        0.505367194937830 */ \
    OD_DCT_OVERFLOW_CHECK(t1, 1035, 1024, 199); \
    t6 += (t1*1035 + 1024) >> 11; \
    /* 3675/4096 ~= Sqrt[2]*Sin[7*Pi/32] ~= 0.897167586342636 */ \
    OD_DCT_OVERFLOW_CHECK(t6, 3675, 2048, 200); \
    t1 -= (t6*3675 + 2048) >> 12; \
    /* 851/8192 ~= (Cos[7*Pi/32] - 1/Sqrt[2])/Sin[7*Pi/32] ~=
        0.103884567856159 */ \
    OD_DCT_OVERFLOW_CHECK(t1, 851, 4096, 201); \
    t6 -= (t1*851 + 4096) >> 13; \
    /* 4379/8192 ~= (Sqrt[2] - Sin[5*Pi/32])/(2*Cos[5*Pi/32]) ~=
        0.534524375168421 */ \
    OD_DCT_OVERFLOW_CHECK(t2, 4379, 4096, 202); \
    t5 += (t2*4379 + 4096) >> 13; \
    /* 10217/8192 ~= Sqrt[2]*Cos[5*Pi/32] ~= 1.24722501298667 */ \
    OD_DCT_OVERFLOW_CHECK(t5, 10217, 4096, 203); \
    t2 -= (t5*10217 + 4096) >> 13; \
    /* 4379/16384 ~= (1/Sqrt[2] - Sin[5*Pi/32])/Cos[5*Pi/32] ~=
        0.267268807193026 */ \
    OD_DCT_OVERFLOW_CHECK(t2, 4379, 8192, 204); \
    t5 += (t2*4379 + 8192) >> 14; \
    /* 12905/16384 ~= (Sqrt[2] - Cos[3*Pi/32])/(2*Sin[3*Pi/32]) ~=
        0.787628942329675 */ \
    OD_DCT_OVERFLOW_CHECK(t3, 12905, 8192, 205); \
    t4 += (t3*12905 + 8192) >> 14; \
    /* 3363/8192 ~= Sqrt[2]*Sin[3*Pi/32] ~= 0.410524527522357 */ \
    OD_DCT_OVERFLOW_CHECK(t4, 3363, 4096, 206); \
    t3 -= (t4*3363 + 4096) >> 13; \
    /* 3525/4096 ~= (Cos[3*Pi/32] - 1/Sqrt[2])/Sin[3*Pi/32] ~=
        0.860650162139486 */ \
    OD_DCT_OVERFLOW_CHECK(t3, 3525, 2048, 207); \
    t4 -= (t3*3525 + 2048) >> 12; \
    /* 5417/8192 ~= (Sqrt[2] - Sin[Pi/32])/(2*Cos[Pi/32]) ~=
        0.661282466846517 */ \
    OD_DCT_OVERFLOW_CHECK(t0, 5417, 4096, 208); \
    t7 += (t0*5417 + 4096) >> 13; \
    /* 5765/4096 ~= Sqrt[2]*Cos[Pi/32] ~= 1.40740373752638 */ \
    OD_DCT_OVERFLOW_CHECK(t7, 5765, 2048, 209); \
    t0 -= (t7*5765 + 2048) >> 12; \
    /* 2507/4096 ~= (1/Sqrt[2] - Sin[Pi/32])/Cos[Pi/32] ~=
        0.612036765167935 */ \
    OD_DCT_OVERFLOW_CHECK(t0, 2507, 2048, 210); \
    t7 += (t0*2507 + 2048) >> 12; \
    t0 += t1; \
    t0h = OD_RSHIFT1(t0); \
    t1 -= t0h; \
    t2 -= t3; \
    t2h = OD_RSHIFT1(t2); \
    t3 += t2h; \
    t5 -= t4; \
    t5h = OD_RSHIFT1(t5); \
    t4 += t5h; \
    t7 += t6; \
    t7h = OD_RSHIFT1(t7); \
    t6 = t7h - t6; \
    t4 = t7h - t4; \
    t7 -= t4; \
    t1 += t5h; \
    t5 = t1 - t5; \
    t6 += t2h; \
    t2 = t6 - t2; \
    t3 -= t0h; \
    t0 += t3; \
    /* 3259/16384 ~= Tan[Pi/16] ~= 0.198912367379658 */ \
    OD_DCT_OVERFLOW_CHECK(t6, 3259, 8192, 211); \
    t1 += (t6*3259 + 8192) >> 14; \
    /* 3135/8192 ~= Sin[Pi/8] ~= 0.382683432365090 */ \
    OD_DCT_OVERFLOW_CHECK(t1, 3135, 4096, 212); \
    t6 -= (t1*3135 + 4096) >> 13; \
    /* 3259/16384 ~= Tan[Pi/16] ~= 0.198912367379658 */ \
    OD_DCT_OVERFLOW_CHECK(t6, 3259, 8192, 213); \
    t1 += (t6*3259 + 8192) >> 14; \
    /* 2737/4096 ~= Tan[3*Pi/16] ~= 0.668178637919299 */ \
    OD_DCT_OVERFLOW_CHECK(t2, 2737, 2048, 214); \
    t5 += (t2*2737 + 2048) >> 12; \
    /* 473/512 ~= Sin[3*Pi/8] ~= 0.923879532511287 */ \
    OD_DCT_OVERFLOW_CHECK(t5, 473, 256, 215); \
    t2 -= (t5*473 + 256) >> 9; \
    /* 2737/4096 ~= Tan[3*Pi/16] ~= 0.668178637919299 */ \
    OD_DCT_OVERFLOW_CHECK(t2, 2737, 2048, 216); \
    t5 += (t2*2737 + 2048) >> 12; \
    /* 3393/8192 ~= Tan[Pi/8] ~= 0.414213562373095 */ \
    OD_DCT_OVERFLOW_CHECK(t4, 3393, 4096, 217); \
    t3 += (t4*3393 + 4096) >> 13; \
    /* 5793/8192 ~= Sin[Pi/4] ~= 0.707106781186547 */ \
    OD_DCT_OVERFLOW_CHECK(t3, 5793, 4096, 218); \
    t4 -= (t3*5793 + 4096) >> 13; \
    /* 3393/8192 ~= Tan[Pi/8] ~= 0.414213562373095 */ \
    OD_DCT_OVERFLOW_CHECK(t4, 3393, 4096, 219); \
    t3 += (t4*3393 + 4096) >> 13; \
} while (0)

/* Embedded 8-point asymmetric Type-IV iDST. */
#define OD_IDST_8_ASYM(t0, t4, t2, t6, t1, t5, t3, t7) \
do { \
    dctcoef t0h; \
    dctcoef t2h; \
    dctcoef t5h__; \
    dctcoef t7h__; \
    /* 3393/8192 ~= Tan[Pi/8] ~= 0.414213562373095 */ \
    t6 -= (t1*3393 + 4096) >> 13; \
    /* 5793/8192 ~= Sin[Pi/4] ~= 0.707106781186547 */ \
    t1 += (t6*5793 + 4096) >> 13; \
    /* 3393/8192 ~= Tan[Pi/8] ~= 0.414213562373095 */ \
    t6 -= (t1*3393 + 4096) >> 13; \
    /* 2737/4096 ~= Tan[3*Pi/16] ~= 0.668178637919299 */ \
    t5 -= (t2*2737 + 2048) >> 12; \
    /* 473/512 ~= Sin[3*Pi/8] ~= 0.923879532511287 */ \
    t2 += (t5*473 + 256) >> 9; \
    /* 2737/4096 ~= Tan[3*Pi/16] ~= 0.668178637919299 */ \
    t5 -= (t2*2737 + 2048) >> 12; \
    /* 3259/16384 ~= Tan[Pi/16] ~= 0.198912367379658 */ \
    t4 -= (t3*3259 + 8192) >> 14; \
    /* 3135/8192 ~= Sin[Pi/8] ~= 0.382683432365090 */ \
    t3 += (t4*3135 + 4096) >> 13; \
    /* 3259/16384 ~= Tan[Pi/16] ~= 0.198912367379658 */ \
    t4 -= (t3*3259 + 8192) >> 14; \
    t0 -= t6; \
    t0h = OD_RSHIFT1(t0); \
    t6 += t0h; \
    t2 = t3 - t2; \
    t2h = OD_RSHIFT1(t2); \
    t3 -= t2h; \
    t5 = t4 - t5; \
    t5h__ = OD_RSHIFT1(t5); \
    t4 -= t5h__; \
    t7 += t1; \
    t7h__ = OD_RSHIFT1(t7); \
    t1 = t7h__ - t1; \
    t3 = t7h__ - t3; \
    t7 -= t3; \
    t1 -= t5h__; \
    t5 += t1; \
    t6 -= t2h; \
    t2 += t6; \
    t4 += t0h; \
    t0 -= t4; \
    /* 2507/4096 ~= (1/Sqrt[2] - Sin[Pi/32])/Cos[Pi/32] ~=
        0.612036765167935 */ \
    t7 -= (t0*2507 + 2048) >> 12; \
    /* 5765/4096 ~= Sqrt[2]*Cos[Pi/32] ~= 1.40740373752638 */ \
    t0 += (t7*5765 + 2048) >> 12; \
    /* 5417/8192 ~= (Sqrt[2] - Sin[Pi/32])/(2*Cos[Pi/32]) ~=
        0.661282466846517 */ \
    t7 -= (t0*5417 + 4096) >> 13; \
    /* 3525/4096 ~= (Cos[3*Pi/32] - 1/Sqrt[2])/Sin[3*Pi/32] ~=
        0.860650162139486 */ \
    t1 += (t6*3525 + 2048) >> 12; \
    /* 3363/8192 ~= Sqrt[2]*Sin[3*Pi/32] ~= 0.410524527522357 */ \
    t6 += (t1*3363 + 4096) >> 13; \
    /* 12905/16384 ~= (1/Sqrt[2] - Cos[3*Pi/32]/1)/Sin[3*Pi/32] ~=
        0.787628942329675 */ \
    t1 -= (t6*12905 + 8192) >> 14; \
    /* 4379/16384 ~= (1/Sqrt[2] - Sin[5*Pi/32])/Cos[5*Pi/32] ~=
        0.267268807193026 */ \
    t5 -= (t2*4379 + 8192) >> 14; \
    /* 10217/8192 ~= Sqrt[2]*Cos[5*Pi/32] ~= 1.24722501298667 */ \
    t2 += (t5*10217 + 4096) >> 13; \
    /* 4379/8192 ~= (Sqrt[2] - Sin[5*Pi/32])/(2*Cos[5*Pi/32]) ~=
        0.534524375168421 */ \
    t5 -= (t2*4379 + 4096) >> 13; \
    /* 851/8192 ~= (Cos[7*Pi/32] - 1/Sqrt[2])/Sin[7*Pi/32] ~=
        0.103884567856159 */ \
    t3 += (t4*851 + 4096) >> 13; \
    /* 3675/4096 ~= Sqrt[2]*Sin[7*Pi/32] ~= 0.897167586342636 */ \
    t4 += (t3*3675 + 2048) >> 12; \
    /* 1035/2048 ~= (Sqrt[2] - Cos[7*Pi/32])/(2*Sin[7*Pi/32]) ~=
        0.505367194937830 */ \
    t3 -= (t4*1035 + 1024) >> 11; \
} while (0)

/* Embedded 16-point orthonormal Type-II fDCT. */
#define OD_FDCT_16(s0, s8, s4, sc, s2, sa, s6, se, \
                   s1, s9, s5, sd, s3, sb, s7, sf) \
do { \
    dctcoef s8h; \
    dctcoef sah; \
    dctcoef sch; \
    dctcoef seh; \
    dctcoef sfh; \
    sf = s0 - sf; \
    sfh = OD_RSHIFT1(sf); \
    s0 -= sfh; \
    se += s1; \
    seh = OD_RSHIFT1(se); \
    s1 = seh - s1; \
    sd = s2 - sd; \
    s2 -= OD_RSHIFT1(sd); \
    sc += s3; \
    sch = OD_RSHIFT1(sc); \
    s3 = sch - s3; \
    sb = s4 - sb; \
    s4 -= OD_RSHIFT1(sb); \
    sa += s5; \
    sah = OD_RSHIFT1(sa); \
    s5 = sah - s5; \
    s9 = s6 - s9; \
    s6 -= OD_RSHIFT1(s9); \
    s8 += s7; \
    s8h = OD_RSHIFT1(s8); \
    s7 = s8h - s7; \
    OD_FDCT_8_ASYM(s0, s8, s8h, s4, sc, sch, s2, sa, sah, s6, se, seh); \
    OD_FDST_8_ASYM(sf, s7, sb, s3, sd, s5, s9, s1); \
} while (0)

/* Embedded 16-point orthonormal Type-II iDCT. */
#define OD_IDCT_16(s0, s8, s4, sc, s2, sa, s6, se, \
                   s1, s9, s5, sd, s3, sb, s7, sf) \
do { \
    dctcoef s1h; \
    dctcoef s3h; \
    dctcoef s5h; \
    dctcoef s7h; \
    dctcoef sfh; \
    OD_IDST_8_ASYM(sf, sb, sd, s9, se, sa, sc, s8); \
    OD_IDCT_8_ASYM(s0, s4, s2, s6, s1, s1h, s5, s5h, s3, s3h, s7, s7h); \
    sfh = OD_RSHIFT1(sf); \
    s0 += sfh; \
    sf = s0 - sf; \
    se = s1h - se; \
    s1 -= se; \
    s2 += OD_RSHIFT1(sd); \
    sd = s2 - sd; \
    sc = s3h - sc; \
    s3 -= sc; \
    s4 += OD_RSHIFT1(sb); \
    sb = s4 - sb; \
    sa = s5h - sa; \
    s5 -= sa; \
    s6 += OD_RSHIFT1(s9); \
    s9 = s6 - s9; \
    s8 = s7h - s8; \
    s7 -= s8; \
} while (0)

/* Embedded 16-point asymmetric Type-II fDCT. */
#define OD_FDCT_16_ASYM(t0, t8, t8h, t4, tc, tch, t2, ta, tah, t6, te, teh, \
                        t1, t9, t9h, t5, td, tdh, t3, tb, tbh, t7, tf, tfh) \
do { \
    t0 += tfh; \
    tf = t0 - tf; \
    t1 -= teh; \
    te += t1; \
    t2 += tdh; \
    td = t2 - td; \
    t3 -= tch; \
    tc += t3; \
    t4 += tbh; \
    tb = t4 - tb; \
    t5 -= tah; \
    ta += t5; \
    t6 += t9h; \
    t9 = t6 - t9; \
    t7 -= t8h; \
    t8 += t7; \
    OD_FDCT_8(t0, t8, t4, tc, t2, ta, t6, te); \
    OD_FDST_8(tf, t7, tb, t3, td, t5, t9, t1); \
} while (0)

/* Embedded 16-point asymmetric Type-II iDCT. */
#define OD_IDCT_16_ASYM(t0, t8, t4, tc, t2, ta, t6, te, t1, t1h, t9, t9h, t5, t5h, \
                        td, tdh, t3, t3h, tb, tbh, t7, t7h, tf, tfh) \
do { \
    OD_IDST_8(tf, tb, td, t9, te, ta, tc, t8); \
    OD_IDCT_8(t0, t4, t2, t6, t1, t5, t3, t7); \
    t1 -= te; \
    t1h = OD_RSHIFT1(t1); \
    te += t1h; \
    t9 = t6 - t9; \
    t9h = OD_RSHIFT1(t9); \
    t6 -= t9h; \
    t5 -= ta; \
    t5h = OD_RSHIFT1(t5); \
    ta += t5h; \
    td = t2 - td; \
    tdh = OD_RSHIFT1(td); \
    t2 -= tdh; \
    t3 -= tc; \
    t3h = OD_RSHIFT1(t3); \
    tc += t3h; \
    tb = t4 - tb; \
    tbh = OD_RSHIFT1(tb); \
    t4 -= tbh; \
    t7 -= t8; \
    t7h = OD_RSHIFT1(t7); \
    t8 += t7h; \
    tf = t0 - tf; \
    tfh = OD_RSHIFT1(tf); \
    t0 -= tfh; \
} while (0)

/* Embedded 16-point orthonormal Type-IV fDST. */
#define OD_FDST_16(s0, s8, s4, sc, s2, sa, s6, se, \
                   s1, s9, s5, sd, s3, sb, s7, sf) \
do { \
    dctcoef s0h; \
    dctcoef s2h; \
    dctcoef sdh; \
    dctcoef sfh; \
    /* 13573/32768 ~= Tan[Pi/8] ~= 0.414213562373095 */ \
    OD_DCT_OVERFLOW_CHECK(s3, 13573, 16384, 220); \
    s1 += (se*13573 + 16384) >> 15; \
    /* 11585/16384 ~= Sin[Pi/4] ~= 0.707106781186547 */ \
    OD_DCT_OVERFLOW_CHECK(s1, 11585, 8192, 221); \
    se -= (s1*11585 + 8192) >> 14; \
    /* 13573/32768 ~= Tan[Pi/8] ~= 0.414213562373095 */ \
    OD_DCT_OVERFLOW_CHECK(s3, 13573, 16384, 222); \
    s1 += (se*13573 + 16384) >> 15; \
    /* 21895/32768 ~= Tan[3*Pi/16] ~= 0.668178637919299 */ \
    OD_DCT_OVERFLOW_CHECK(s2, 21895, 16384, 223); \
    sd += (s2*21895 + 16384) >> 15; \
    /* 15137/16384 ~= Sin[3*Pi/8] ~= 0.923879532511287 */ \
    OD_DCT_OVERFLOW_CHECK(sd, 15137, 16384, 224); \
    s2 -= (sd*15137 + 8192) >> 14; \
    /* 21895/32768 ~= Tan[3*Pi/16] ~= 0.668178637919299 */ \
    OD_DCT_OVERFLOW_CHECK(s2, 21895, 16384, 225); \
    sd += (s2*21895 + 16384) >> 15; \
    /* 3259/16384 ~= Tan[Pi/16] ~= 0.198912367379658 */ \
    OD_DCT_OVERFLOW_CHECK(s3, 3259, 8192, 226); \
    sc += (s3*3259 + 8192) >> 14; \
    /* 3135/8192 ~= Sin[Pi/8] ~= 0.382683432365090 */ \
    OD_DCT_OVERFLOW_CHECK(sc, 3135, 4096, 227); \
    s3 -= (sc*3135 + 4096) >> 13; \
    /* 3259/16384 ~= Tan[Pi/16] ~= 0.198912367379658 */ \
    OD_DCT_OVERFLOW_CHECK(s3, 3259, 8192, 228); \
    sc += (s3*3259 + 8192) >> 14; \
    /* 13573/32768 ~= Tan[Pi/8] ~= 0.414213562373095 */ \
    OD_DCT_OVERFLOW_CHECK(s5, 13573, 16384, 229); \
    sa += (s5*13573 + 16384) >> 15; \
    /* 11585/16384 ~= Sin[Pi/4] ~= 0.707106781186547 */ \
    OD_DCT_OVERFLOW_CHECK(sa, 11585, 8192, 230); \
    s5 -= (sa*11585 + 8192) >> 14; \
    /* 13573/32768 ~= Tan[Pi/8] ~= 0.414213562373095 */ \
    OD_DCT_OVERFLOW_CHECK(s5, 13573, 16384, 231); \
    sa += (s5*13573 + 16384) >> 15; \
    /* 13573/32768 ~= Tan[pi/8] ~= 0.414213562373095 */ \
    OD_DCT_OVERFLOW_CHECK(s9, 13573, 16384, 232); \
    s6 += (s9*13573 + 16384) >> 15; \
    /* 11585/16384 ~= Sin[pi/4] ~= 0.707106781186547 */ \
    OD_DCT_OVERFLOW_CHECK(s6, 11585, 8192, 233); \
    s9 -= (s6*11585 + 8192) >> 14; \
    /* 13573/32768 ~= Tan[pi/8] ~= 0.414213562373095 */ \
    OD_DCT_OVERFLOW_CHECK(s9, 13573, 16384, 234); \
    s6 += (s9*13573 + 16384) >> 15; \
    sf += se; \
    sfh = OD_RSHIFT1(sf); \
    se = sfh - se; \
    s0 += s1; \
    s0h = OD_RSHIFT1(s0); \
    s1 = s0h - s1; \
    s2 = s3 - s2; \
    s2h = OD_RSHIFT1(s2); \
    s3 -= s2h; \
    sd -= sc; \
    sdh = OD_RSHIFT1(sd); \
    sc += sdh; \
    sa = s4 - sa; \
    s4 -= OD_RSHIFT1(sa); \
    s5 += sb; \
    sb = OD_RSHIFT1(s5) - sb; \
    s8 += s6; \
    s6 -= OD_RSHIFT1(s8); \
    s7 = s9 - s7; \
    s9 -= OD_RSHIFT1(s7); \
    /* 6723/8192 ~= Tan[7*Pi/32] ~= 0.820678790828660 */ \
    OD_DCT_OVERFLOW_CHECK(sb, 6723, 4096, 235); \
    s4 += (sb*6723 + 4096) >> 13; \
    /* 16069/16384 ~= Sin[7*Pi/16] ~= 0.980785280403230 */ \
    OD_DCT_OVERFLOW_CHECK(s4, 16069, 8192, 236); \
    sb -= (s4*16069 + 8192) >> 14; \
    /* 6723/8192 ~= Tan[7*Pi/32]) ~= 0.820678790828660 */ \
    OD_DCT_OVERFLOW_CHECK(sb, 6723, 4096, 237); \
    s4 += (sb*6723 + 4096) >> 13; \
    /* 8757/16384 ~= Tan[5*Pi/32]) ~= 0.534511135950792 */ \
    OD_DCT_OVERFLOW_CHECK(s5, 8757, 8192, 238); \
    sa += (s5*8757 + 8192) >> 14; \
    /* 6811/8192 ~= Sin[5*Pi/16] ~= 0.831469612302545 */ \
    OD_DCT_OVERFLOW_CHECK(sa, 6811, 4096, 239); \
    s5 -= (sa*6811 + 4096) >> 13; \
    /* 8757/16384 ~= Tan[5*Pi/32] ~= 0.534511135950792 */ \
    OD_DCT_OVERFLOW_CHECK(s5, 8757, 8192, 240); \
    sa += (s5*8757 + 8192) >> 14; \
    /* 2485/8192 ~= Tan[3*Pi/32] ~= 0.303346683607342 */ \
    OD_DCT_OVERFLOW_CHECK(s9, 2485, 4096, 241); \
    s6 += (s9*2485 + 4096) >> 13; \
    /* 4551/8192 ~= Sin[3*Pi/16] ~= 0.555570233019602 */ \
    OD_DCT_OVERFLOW_CHECK(s6, 4551, 4096, 242); \
    s9 -= (s6*4551 + 4096) >> 13; \
    /* 2485/8192 ~= Tan[3*Pi/32] ~= 0.303346683607342 */ \
    OD_DCT_OVERFLOW_CHECK(s9, 2485, 4096, 243); \
    s6 += (s9*2485 + 4096) >> 13; \
    /* 3227/32768 ~= Tan[Pi/32] ~= 0.09849140335716425 */ \
    OD_DCT_OVERFLOW_CHECK(s8, 3227, 16384, 244); \
    s7 += (s8*3227 + 16384) >> 15; \
    /* 6393/32768 ~= Sin[Pi/16] ~= 0.19509032201612825 */ \
    OD_DCT_OVERFLOW_CHECK(s7, 6393, 16384, 245); \
    s8 -= (s7*6393 + 16384) >> 15; \
    /* 3227/32768 ~= Tan[Pi/32] ~= 0.09849140335716425 */ \
    OD_DCT_OVERFLOW_CHECK(s8, 3227, 16384, 246); \
    s7 += (s8*3227 + 16384) >> 15; \
    s1 -= s2h; \
    s2 += s1; \
    se += sdh; \
    sd = se - sd; \
    s3 += sfh; \
    sf -= s3; \
    sc = s0h - sc; \
    s0 -= sc; \
    sb += OD_RSHIFT1(s8); \
    s8 = sb - s8; \
    s4 += OD_RSHIFT1(s7); \
    s7 -= s4; \
    s6 += OD_RSHIFT1(s5); \
    s5 = s6 - s5; \
    s9 -= OD_RSHIFT1(sa); \
    sa += s9; \
    s8 += s0; \
    s0 -= OD_RSHIFT1(s8); \
    sf += s7; \
    s7 = OD_RSHIFT1(sf) - s7; \
    s1 -= s6; \
    s6 += OD_RSHIFT1(s1); \
    s9 += se; \
    se = OD_RSHIFT1(s9) - se; \
    s2 += sa; \
    sa = OD_RSHIFT1(s2) - sa; \
    s5 += sd; \
    sd -= OD_RSHIFT1(s5); \
    s4 = sc - s4; \
    sc -= OD_RSHIFT1(s4); \
    s3 -= sb; \
    sb += OD_RSHIFT1(s3); \
    /* 2799/4096 ~= (1/Sqrt[2] - Cos[31*Pi/64]/2)/Sin[31*Pi/64] ~=
        0.6833961245841154 */ \
    OD_DCT_OVERFLOW_CHECK(sf, 2799, 2048, 247); \
    s0 -= (sf*2799 + 2048) >> 12; \
    /* 2893/2048 ~= Sqrt[2]*Sin[31*Pi/64] ~= 1.4125100802019777 */ \
    OD_DCT_OVERFLOW_CHECK(s0, 2893, 1024, 248); \
    sf += (s0*2893 + 1024) >> 11; \
    /* 5397/8192 ~= (Cos[Pi/4] - Cos[31*Pi/64])/Sin[31*Pi/64] ~=
        0.6588326996993819 */ \
    OD_DCT_OVERFLOW_CHECK(sf, 5397, 4096, 249); \
    s0 -= (sf*5397 + 4096) >> 13; \
    /* 41/64 ~= (1/Sqrt[2] - Cos[29*Pi/64]/2)/Sin[29*Pi/64] ~=
        0.6406758931036793 */ \
    OD_DCT_OVERFLOW_CHECK(s1, 41, 32, 250); \
    se += (s1*41 + 32) >> 6; \
    /* 2865/2048 ~= Sqrt[2]*Sin[29*Pi/64] ~= 1.3989068359730783 */ \
    OD_DCT_OVERFLOW_CHECK(se, 2865, 1024, 251); \
    s1 -= (se*2865 + 1024) >> 11; \
    /* 4641/8192 ~= (1/Sqrt[2] - Cos[29*Pi/64])/Sin[29*Pi/64] ~=
        0.5665078993345056 */ \
    OD_DCT_OVERFLOW_CHECK(s1, 4641, 4096, 252); \
    se += (s1*4641 + 4096) >> 13; \
    /* 2473/4096 ~= (1/Sqrt[2] - Cos[27*Pi/64]/2)/Sin[27*Pi/64] ~=
        0.603709096285651 */ \
    OD_DCT_OVERFLOW_CHECK(s2, 2473, 2048, 253); \
    sd += (s2*2473 + 2048) >> 12; \
    /* 5619/4096 ~= Sqrt[2]*Sin[27*Pi/64] ~= 1.371831354193494 */ \
    OD_DCT_OVERFLOW_CHECK(sd, 5619, 2048, 254); \
    s2 -= (sd*5619 + 2048) >> 12; \
    /* 7839/16384 ~= (1/Sqrt[2] - Cos[27*Pi/64])/Sin[27*Pi/64] ~=
        0.47846561618999817 */ \
    OD_DCT_OVERFLOW_CHECK(s2, 7839, 8192, 255); \
    sd += (s2*7839 + 8192) >> 14; \
    /* 5747/8192 ~= (1/Sqrt[2] - Cos[7*Pi/64]/2)/Sin[7*Pi/64] ~=
        0.7015193429405162 */ \
    OD_DCT_OVERFLOW_CHECK(s3, 5747, 4096, 256); \
    sc -= (s3*5747 + 4096) >> 13; \
    /* 3903/8192 ~= Sqrt[2]*Sin[7*Pi/64] ~= 0.47643419969316125 */ \
    OD_DCT_OVERFLOW_CHECK(sc, 3903, 4096, 257); \
    s3 += (sc*3903 + 4096) >> 13; \
    /* 5701/8192 ~= (1/Sqrt[2] - Cos[7*Pi/64])/Sin[7*Pi/64] ~=
        0.6958870433047222 */ \
    OD_DCT_OVERFLOW_CHECK(s3, 5701, 4096, 258); \
    sc += (s3*5701 + 4096) >> 13; \
    /* 4471/8192 ~= (1/Sqrt[2] - Cos[23*Pi/64]/2)/Sin[23*Pi/64] ~=
        0.5457246432276498 */ \
    OD_DCT_OVERFLOW_CHECK(s4, 4471, 4096, 259); \
    sb += (s4*4471 + 4096) >> 13; \
    /* 1309/1024 ~= Sqrt[2]*Sin[23*Pi/64] ~= 1.278433918575241 */ \
    OD_DCT_OVERFLOW_CHECK(sb, 1309, 512, 260); \
    s4 -= (sb*1309 + 512) >> 10; \
    /* 5067/16384 ~= (1/Sqrt[2] - Cos[23*Pi/64])/Sin[23*Pi/64] ~=
        0.30924225528198984 */ \
    OD_DCT_OVERFLOW_CHECK(s4, 5067, 8192, 261); \
    sb += (s4*5067 + 8192) >> 14; \
    /* 2217/4096 ~= (1/Sqrt[2] - Cos[11*Pi/64]/2)/Sin[11*Pi/64] ~=
        0.5412195895259334 */ \
    OD_DCT_OVERFLOW_CHECK(s5, 2217, 2048, 262); \
    sa -= (s5*2217 + 2048) >> 12; \
    /* 1489/2048 ~= Sqrt[2]*Sin[11*Pi/64] ~= 0.72705107329128 */ \
    OD_DCT_OVERFLOW_CHECK(sa, 1489, 1024, 263); \
    s5 += (sa*1489 + 1024) >> 11; \
    /* 75/256 ~= (1/Sqrt[2] - Cos[11*Pi/64])/Sin[11*Pi/64] ~=
        0.2929800132658202 */ \
    OD_DCT_OVERFLOW_CHECK(s5, 75, 128, 264); \
    sa += (s5*75 + 128) >> 8; \
    /* 2087/4096 ~= (1/Sqrt[2] - Cos[19*Pi/64]/2)/Sin[19*Pi/64] ~=
        0.5095285002941893 */ \
    OD_DCT_OVERFLOW_CHECK(s9, 2087, 2048, 265); \
    s6 -= (s9*2087 + 2048) >> 12; \
    /* 4653/4096 ~= Sqrt[2]*Sin[19*Pi/64] ~= 1.1359069844201428 */ \
    OD_DCT_OVERFLOW_CHECK(s6, 4653, 2048, 266); \
    s9 += (s6*4653 + 2048) >> 12; \
    /* 4545/32768 ~= (1/Sqrt[2] - Cos[19*Pi/64])/Sin[19*Pi/64] ~=
        0.13870322715817154 */ \
    OD_DCT_OVERFLOW_CHECK(s9, 4545, 16384, 267); \
    s6 -= (s9*4545 + 16384) >> 15; \
    /* 2053/4096 ~= (1/Sqrt[2] - Cos[15*Pi/64]/2)/Sin[15*Pi/64] ~=
        0.5012683042634027 */ \
    OD_DCT_OVERFLOW_CHECK(s8, 2053, 2048, 268); \
    s7 += (s8*2053 + 2048) >> 12; \
    /* 1945/2048 ~= Sqrt[2]*Sin[15*Pi/64] ~= 0.9497277818777543 */ \
    OD_DCT_OVERFLOW_CHECK(s7, 1945, 1024, 269); \
    s8 -= (s7*1945 + 1024) >> 11; \
    /* 1651/32768 ~= (1/Sqrt[2] - Cos[15*Pi/64])/Sin[15*Pi/64] ~=
        0.05039668360333519 */ \
    OD_DCT_OVERFLOW_CHECK(s8, 1651, 16384, 270); \
    s7 -= (s8*1651 + 16384) >> 15; \
} while (0)

/* Embedded 16-point orthonormal Type-IV iDST. */
#define OD_IDST_16(s0, s8, s4, sc, s2, sa, s6, se, \
                   s1, s9, s5, sd, s3, sb, s7, sf) \
do { \
    dctcoef s0h; \
    dctcoef s4h; \
    dctcoef sbh; \
    dctcoef sfh; \
    /* 1651/32768 ~= (1/Sqrt[2] - Cos[15*Pi/64])/Sin[15*Pi/64] ~=
        0.05039668360333519 */ \
    se += (s1*1651 + 16384) >> 15; \
    /* 1945/2048 ~= Sqrt[2]*Sin[15*Pi/64] ~= 0.9497277818777543 */ \
    s1 += (se*1945 + 1024) >> 11; \
    /* 2053/4096 ~= (1/Sqrt[2] - Cos[15*Pi/64]/2)/Sin[15*Pi/64] ~=
        0.5012683042634027 */ \
    se -= (s1*2053 + 2048) >> 12; \
    /* 4545/32768 ~= (1/Sqrt[2] - Cos[19*Pi/64])/Sin[19*Pi/64] ~=
        0.13870322715817154 */ \
    s6 += (s9*4545 + 16384) >> 15; \
    /* 4653/32768 ~= Sqrt[2]*Sin[19*Pi/64] ~= 1.1359069844201428 */ \
    s9 -= (s6*4653 + 2048) >> 12; \
    /* 2087/4096 ~= (1/Sqrt[2] - Cos[19*Pi/64]/2)/Sin[19*Pi/64] ~=
        0.5095285002941893 */ \
    s6 += (s9*2087 + 2048) >> 12; \
    /* 75/256 ~= (1/Sqrt[2] - Cos[11*Pi/64])/Sin[11*Pi/64] ~=
        0.2929800132658202 */ \
    s5 -= (sa*75 + 128) >> 8; \
    /* 1489/2048 ~= Sqrt[2]*Sin[11*Pi/64] ~= 0.72705107329128 */ \
    sa -= (s5*1489 + 1024) >> 11; \
    /* 2217/4096 ~= (1/Sqrt[2] - Cos[11*Pi/64]/2)/Sin[11*Pi/64] ~=
        0.5412195895259334 */ \
    s5 += (sa*2217 + 2048) >> 12; \
    /* 5067/16384 ~= (1/Sqrt[2] - Cos[23*Pi/64])/Sin[23*Pi/64] ~=
        0.30924225528198984 */ \
    sd -= (s2*5067 + 8192) >> 14; \
    /* 1309/1024 ~= Sqrt[2]*Sin[23*Pi/64] ~= 1.278433918575241 */ \
    s2 += (sd*1309 + 512) >> 10; \
    /* 4471/8192 ~= (1/Sqrt[2] - Cos[23*Pi/64]/2)/Sin[23*Pi/64] ~=
        0.5457246432276498 */ \
    sd -= (s2*4471 + 4096) >> 13; \
    /* 5701/8192 ~= (1/Sqrt[2] - Cos[7*Pi/64])/Sin[7*Pi/64] ~=
        0.6958870433047222 */ \
    s3 -= (sc*5701 + 4096) >> 13; \
    /* 3903/8192 ~= Sqrt[2]*Sin[7*Pi/64] ~= 0.47643419969316125 */ \
    sc -= (s3*3903 + 4096) >> 13; \
    /* 5747/8192 ~= (1/Sqrt[2] - Cos[7*Pi/64]/2)/Sin[7*Pi/64] ~=
        0.7015193429405162 */ \
    s3 += (sc*5747 + 4096) >> 13; \
    /* 7839/16384 ~= (1/Sqrt[2] - Cos[27*Pi/64])/Sin[27*Pi/64] ~=
        0.47846561618999817 */ \
    sb -= (s4*7839 + 8192) >> 14; \
    /* 5619/4096 ~= Sqrt[2]*Sin[27*Pi/64] ~= 1.371831354193494 */ \
    s4 += (sb*5619 + 2048) >> 12; \
    /* 2473/4096 ~= (1/Sqrt[2] - Cos[27*Pi/64]/2)/Sin[27*Pi/64] ~=
        0.603709096285651 */ \
    sb -= (s4*2473 + 2048) >> 12; \
    /* 4641/8192 ~= (1/Sqrt[2] - Cos[29*Pi/64])/Sin[29*Pi/64] ~=
        0.5665078993345056 */ \
    s7 -= (s8*4641 + 4096) >> 13; \
    /* 2865/2048 ~= Sqrt[2]*Sin[29*Pi/64] ~= 1.3989068359730783 */ \
    s8 += (s7*2865 + 1024) >> 11; \
    /* 41/64 ~= (1/Sqrt[2] - Cos[29*Pi/64]/2)/Sin[29*Pi/64] ~=
        0.6406758931036793 */ \
    s7 -= (s8*41 + 32) >> 6; \
    /* 5397/8192 ~= (Cos[Pi/4] - Cos[31*Pi/64])/Sin[31*Pi/64] ~=
        0.6588326996993819 */ \
    s0 += (sf*5397 + 4096) >> 13; \
    /* 2893/2048 ~= Sqrt[2]*Sin[31*Pi/64] ~= 1.4125100802019777 */ \
    sf -= (s0*2893 + 1024) >> 11; \
    /* 2799/4096 ~= (1/Sqrt[2] - Cos[31*Pi/64]/2)/Sin[31*Pi/64] ~=
        0.6833961245841154 */ \
    s0 += (sf*2799 + 2048) >> 12; \
    sd -= OD_RSHIFT1(sc); \
    sc += sd; \
    s3 += OD_RSHIFT1(s2); \
    s2 = s3 - s2; \
    sb += OD_RSHIFT1(sa); \
    sa -= sb; \
    s5 = OD_RSHIFT1(s4) - s5; \
    s4 -= s5; \
    s7 = OD_RSHIFT1(s9) - s7; \
    s9 -= s7; \
    s6 -= OD_RSHIFT1(s8); \
    s8 += s6; \
    se = OD_RSHIFT1(sf) - se; \
    sf -= se; \
    s0 += OD_RSHIFT1(s1); \
    s1 -= s0; \
    s5 -= s9; \
    s9 += OD_RSHIFT1(s5); \
    sa = s6 - sa; \
    s6 -= OD_RSHIFT1(sa); \
    se += s2; \
    s2 -= OD_RSHIFT1(se); \
    s1 = sd - s1; \
    sd -= OD_RSHIFT1(s1); \
    s0 += s3; \
    s0h = OD_RSHIFT1(s0); \
    s3 = s0h - s3; \
    sf += sc; \
    sfh = OD_RSHIFT1(sf); \
    sc -= sfh; \
    sb = s7 - sb; \
    sbh = OD_RSHIFT1(sb); \
    s7 -= sbh; \
    s4 -= s8; \
    s4h = OD_RSHIFT1(s4); \
    s8 += s4h; \
    /* 3227/32768 ~= Tan[Pi/32] ~= 0.09849140335716425 */ \
    se -= (s1*3227 + 16384) >> 15; \
    /* 6393/32768 ~= Sin[Pi/16] ~= 0.19509032201612825 */ \
    s1 += (se*6393 + 16384) >> 15; \
    /* 3227/32768 ~= Tan[Pi/32] ~= 0.09849140335716425 */ \
    se -= (s1*3227 + 16384) >> 15; \
    /* 2485/8192 ~= Tan[3*Pi/32] ~= 0.303346683607342 */ \
    s6 -= (s9*2485 + 4096) >> 13; \
    /* 4551/8192 ~= Sin[3*Pi/16] ~= 0.555570233019602 */ \
    s9 += (s6*4551 + 4096) >> 13; \
    /* 2485/8192 ~= Tan[3*Pi/32] ~= 0.303346683607342 */ \
    s6 -= (s9*2485 + 4096) >> 13; \
    /* 8757/16384 ~= Tan[5*Pi/32] ~= 0.534511135950792 */ \
    s5 -= (sa*8757 + 8192) >> 14; \
    /* 6811/8192 ~= Sin[5*Pi/16] ~= 0.831469612302545 */ \
    sa += (s5*6811 + 4096) >> 13; \
    /* 8757/16384 ~= Tan[5*Pi/32]) ~= 0.534511135950792 */ \
    s5 -= (sa*8757 + 8192) >> 14; \
    /* 6723/8192 ~= Tan[7*Pi/32]) ~= 0.820678790828660 */ \
    s2 -= (sd*6723 + 4096) >> 13; \
    /* 16069/16384 ~= Sin[7*Pi/16] ~= 0.980785280403230 */ \
    sd += (s2*16069 + 8192) >> 14; \
    /* 6723/8192 ~= Tan[7*Pi/32] ~= 0.820678790828660 */ \
    s2 -= (sd*6723 + 4096) >> 13; \
    s9 += OD_RSHIFT1(se); \
    se = s9 - se; \
    s6 += OD_RSHIFT1(s1); \
    s1 -= s6; \
    sd = OD_RSHIFT1(sa) - sd; \
    sa -= sd; \
    s2 += OD_RSHIFT1(s5); \
    s5 = s2 - s5; \
    s3 -= sbh; \
    sb += s3; \
    sc += s4h; \
    s4 = sc - s4; \
    s8 = s0h - s8; \
    s0 -= s8; \
    s7 = sfh - s7; \
    sf -= s7; \
    /* 13573/32768 ~= Tan[pi/8] ~= 0.414213562373095 */ \
    s6 -= (s9*13573 + 16384) >> 15; \
    /* 11585/16384 ~= Sin[pi/4] ~= 0.707106781186547 */ \
    s9 += (s6*11585 + 8192) >> 14; \
    /* 13573/32768 ~= Tan[pi/8] ~= 0.414213562373095 */ \
    s6 -= (s9*13573 + 16384) >> 15; \
    /* 13573/32768 ~= Tan[pi/8] ~= 0.414213562373095 */ \
    s5 -= (sa*13573 + 16384) >> 15; \
    /* 11585/16384 ~= Sin[pi/4] ~= 0.707106781186547 */ \
    sa += (s5*11585 + 8192) >> 14; \
    /* 13573/32768 ~= Tan[pi/8] ~= 0.414213562373095 */ \
    s5 -= (sa*13573 + 16384) >> 15; \
    /* 3259/16384 ~= Tan[Pi/16] ~= 0.198912367379658 */ \
    s3 -= (sc*3259 + 8192) >> 14; \
    /* 3135/8192 ~= Sin[Pi/8] ~= 0.382683432365090 */ \
    sc += (s3*3135 + 4096) >> 13; \
    /* 3259/16384 ~= Tan[Pi/16] ~= 0.198912367379658 */ \
    s3 -= (sc*3259 + 8192) >> 14; \
    /* 21895/32768 ~= Tan[3*Pi/16] ~= 0.668178637919299 */ \
    sb -= (s4*21895 + 16384) >> 15; \
    /* 15137/16384 ~= Sin[3*Pi/8] ~= 0.923879532511287 */ \
    s4 += (sb*15137 + 8192) >> 14; \
    /* 21895/32768 ~= Tan[3*Pi/16] ~= 0.668178637919299 */ \
    sb -= (s4*21895 + 16384) >> 15; \
    /* 13573/32768 ~= Tan[pi/8] ~= 0.414213562373095 */ \
    s8 -= (s7*13573 + 16384) >> 15; \
    /* 11585/16384 ~= Sin[pi/4] ~= 0.707106781186547 */ \
    s7 += (s8*11585 + 8192) >> 14; \
    /* 13573/32768 ~= Tan[pi/8] ~= 0.414213562373095 */ \
    s8 -= (s7*13573 + 16384) >> 15; \
} while (0)

/* Embedded 16-point asymmetric Type-IV fDST. */
/* TODO: rewrite this to match OD_FDST_16. */
#define OD_FDST_16_ASYM(t0, t0h, t8, t4, t4h, tc, t2, ta, t6, te, \
                        t1, t9, t5, td, t3, tb, t7, t7h, tf) \
do { \
    dctcoef t2h; \
    dctcoef t3h; \
    dctcoef t6h; \
    dctcoef t8h; \
    dctcoef t9h; \
    dctcoef tch; \
    dctcoef tdh; \
    /* TODO: Can we move these into another operation */ \
    t8 = -t8; \
    t9 = -t9; \
    ta = -ta; \
    tb = -tb; \
    td = -td; \
    /* 13573/16384 ~= 2*Tan[Pi/8] ~= 0.828427124746190 */ \
    OD_DCT_OVERFLOW_CHECK(te, 13573, 8192, 136); \
    t1 -= (te*13573 + 8192) >> 14; \
    /* 11585/32768 ~= Sin[Pi/4]/2 ~= 0.353553390593274 */ \
    OD_DCT_OVERFLOW_CHECK(t1, 11585, 16384, 137); \
    te += (t1*11585 + 16384) >> 15; \
    /* 13573/16384 ~= 2*Tan[Pi/8] ~= 0.828427124746190 */ \
    OD_DCT_OVERFLOW_CHECK(te, 13573, 8192, 138); \
    t1 -= (te*13573 + 8192) >> 14; \
    /* 4161/16384 ~= Tan[3*Pi/16] - Tan[Pi/8] ~= 0.253965075546204 */ \
    OD_DCT_OVERFLOW_CHECK(td, 4161, 8192, 139); \
    t2 += (td*4161 + 8192) >> 14; \
    /* 15137/16384 ~= Sin[3*Pi/8] ~= 0.923879532511287 */ \
    OD_DCT_OVERFLOW_CHECK(t2, 15137, 8192, 140); \
    td -= (t2*15137 + 8192) >> 14; \
    /* 14341/16384 ~= Tan[3*Pi/16] + Tan[Pi/8]/2 ~= 0.875285419105846 */ \
    OD_DCT_OVERFLOW_CHECK(td, 14341, 8192, 141); \
    t2 += (td*14341 + 8192) >> 14; \
    /* 14341/16384 ~= Tan[3*Pi/16] + Tan[Pi/8]/2 ~= 0.875285419105846 */ \
    OD_DCT_OVERFLOW_CHECK(t3, 14341, 8192, 142); \
    tc -= (t3*14341 + 8192) >> 14; \
    /* 15137/16384 ~= Sin[3*Pi/8] ~= 0.923879532511287 */ \
    OD_DCT_OVERFLOW_CHECK(tc, 15137, 8192, 143); \
    t3 += (tc*15137 + 8192) >> 14; \
    /* 4161/16384 ~= Tan[3*Pi/16] - Tan[Pi/8] ~= 0.253965075546204 */ \
    OD_DCT_OVERFLOW_CHECK(t3, 4161, 8192, 144); \
    tc -= (t3*4161 + 8192) >> 14; \
    te = t0h - te; \
    t0 -= te; \
    tf = OD_RSHIFT1(t1) - tf; \
    t1 -= tf; \
    /* TODO: Can we move this into another operation */ \
    tc = -tc; \
    t2 = OD_RSHIFT1(tc) - t2; \
    tc -= t2; \
    t3 = OD_RSHIFT1(td) - t3; \
    td = t3 - td; \
    /* 7489/8192 ~= Tan[Pi/8] + Tan[Pi/4]/2 ~= 0.914213562373095 */ \
    OD_DCT_OVERFLOW_CHECK(t6, 7489, 4096, 145); \
    t9 -= (t6*7489 + 4096) >> 13; \
    /* 11585/16384 ~= Sin[Pi/4] ~= 0.707106781186548 */ \
    OD_DCT_OVERFLOW_CHECK(t9, 11585, 8192, 146); \
    t6 += (t9*11585 + 8192) >> 14; \
    /* -19195/32768 ~= Tan[Pi/8] - Tan[Pi/4] ~= -0.585786437626905 */ \
    OD_DCT_OVERFLOW_CHECK(t6, 19195, 16384, 147); \
    t9 += (t6*19195 + 16384) >> 15; \
    t8 += OD_RSHIFT1(t9); \
    t9 -= t8; \
    t6 = t7h - t6; \
    t7 -= t6; \
    /* 6723/8192 ~= Tan[7*Pi/32] ~= 0.820678790828660 */ \
    OD_DCT_OVERFLOW_CHECK(t7, 6723, 4096, 148); \
    t8 += (t7*6723 + 4096) >> 13; \
    /* 16069/16384 ~= Sin[7*Pi/16] ~= 0.980785280403230 */ \
    OD_DCT_OVERFLOW_CHECK(t8, 16069, 8192, 149); \
    t7 -= (t8*16069 + 8192) >> 14; \
    /* 6723/8192 ~= Tan[7*Pi/32]) ~= 0.820678790828660 */ \
    OD_DCT_OVERFLOW_CHECK(t7, 6723, 4096, 150); \
    t8 += (t7*6723 + 4096) >> 13; \
    /* 17515/32768 ~= Tan[5*Pi/32]) ~= 0.534511135950792 */ \
    OD_DCT_OVERFLOW_CHECK(t6, 17515, 16384, 151); \
    t9 += (t6*17515 + 16384) >> 15; \
    /* 13623/16384 ~= Sin[5*Pi/16] ~= 0.831469612302545 */ \
    OD_DCT_OVERFLOW_CHECK(t9, 13623, 8192, 152); \
    t6 -= (t9*13623 + 8192) >> 14; \
    /* 17515/32768 ~= Tan[5*Pi/32] ~= 0.534511135950792 */ \
    OD_DCT_OVERFLOW_CHECK(t6, 17515, 16384, 153); \
    t9 += (t6*17515 + 16384) >> 15; \
    /* 13573/16384 ~= 2*Tan[Pi/8] ~= 0.828427124746190 */ \
    OD_DCT_OVERFLOW_CHECK(ta, 13573, 8192, 154); \
    t5 += (ta*13573 + 8192) >> 14; \
    /* 11585/32768 ~= Sin[Pi/4]/2 ~= 0.353553390593274 */ \
    OD_DCT_OVERFLOW_CHECK(t5, 11585, 16384, 155); \
    ta -= (t5*11585 + 16384) >> 15; \
    /* 13573/16384 ~= 2*Tan[Pi/8] ~= 0.828427124746190 */ \
    OD_DCT_OVERFLOW_CHECK(ta, 13573, 8192, 156); \
    t5 += (ta*13573 + 8192) >> 14; \
    tb += OD_RSHIFT1(t5); \
    t5 = tb - t5; \
    ta += t4h; \
    t4 -= ta; \
    /* 2485/8192 ~= Tan[3*Pi/32] ~= 0.303346683607342 */ \
    OD_DCT_OVERFLOW_CHECK(t5, 2485, 4096, 157); \
    ta += (t5*2485 + 4096) >> 13; \
    /* 18205/32768 ~= Sin[3*Pi/16] ~= 0.555570233019602 */ \
    OD_DCT_OVERFLOW_CHECK(ta, 18205, 16384, 158); \
    t5 -= (ta*18205 + 16384) >> 15; \
    /* 2485/8192 ~= Tan[3*Pi/32] ~= 0.303346683607342 */ \
    OD_DCT_OVERFLOW_CHECK(t5, 2485, 4096, 159); \
    ta += (t5*2485 + 4096) >> 13; \
    /* 6723/8192 ~= Tan[7*Pi/32] ~= 0.820678790828660 */ \
    OD_DCT_OVERFLOW_CHECK(t4, 6723, 4096, 160); \
    tb -= (t4*6723 + 4096) >> 13; \
    /* 16069/16384 ~= Sin[7*Pi/16] ~= 0.980785280403230 */ \
    OD_DCT_OVERFLOW_CHECK(tb, 16069, 8192, 161); \
    t4 += (tb*16069 + 8192) >> 14; \
    /* 6723/8192 ~= Tan[7*Pi/32] ~= 0.820678790828660 */ \
    OD_DCT_OVERFLOW_CHECK(t4, 6723, 4096, 162); \
    tb -= (t4*6723 + 4096) >> 13; \
    /* TODO: Can we move this into another operation */ \
    t5 = -t5; \
    tc -= tf; \
    tch = OD_RSHIFT1(tc); \
    tf += tch; \
    t3 += t0; \
    t3h = OD_RSHIFT1(t3); \
    t0 -= t3h; \
    td -= t1; \
    tdh = OD_RSHIFT1(td); \
    t1 += tdh; \
    t2 += te; \
    t2h = OD_RSHIFT1(t2); \
    te -= t2h; \
    t8 += t4; \
    t8h = OD_RSHIFT1(t8); \
    t4 = t8h - t4; \
    t7 = tb - t7; \
    t7h = OD_RSHIFT1(t7); \
    tb = t7h - tb; \
    t6 -= ta; \
    t6h = OD_RSHIFT1(t6); \
    ta += t6h; \
    t9 = t5 - t9; \
    t9h = OD_RSHIFT1(t9); \
    t5 -= t9h; \
    t0 -= t7h; \
    t7 += t0; \
    tf += t8h; \
    t8 -= tf; \
    te -= t6h; \
    t6 += te; \
    t1 += t9h; \
    t9 -= t1; \
    tb -= tch; \
    tc += tb; \
    t4 += t3h; \
    t3 -= t4; \
    ta -= tdh; \
    td += ta; \
    t5 = t2h - t5; \
    t2 -= t5; \
    /* TODO: Can we move these into another operation */ \
    t8 = -t8; \
    t9 = -t9; \
    ta = -ta; \
    tb = -tb; \
    tc = -tc; \
    td = -td; \
    tf = -tf; \
    /* 7799/8192 ~= Tan[31*Pi/128] ~= 0.952079146700925 */ \
    OD_DCT_OVERFLOW_CHECK(tf, 7799, 4096, 163); \
    t0 -= (tf*7799 + 4096) >> 13; \
    /* 4091/4096 ~= Sin[31*Pi/64] ~= 0.998795456205172 */ \
    OD_DCT_OVERFLOW_CHECK(t0, 4091, 2048, 164); \
    tf += (t0*4091 + 2048) >> 12; \
    /* 7799/8192 ~= Tan[31*Pi/128] ~= 0.952079146700925 */ \
    OD_DCT_OVERFLOW_CHECK(tf, 7799, 4096, 165); \
    t0 -= (tf*7799 + 4096) >> 13; \
    /* 2417/32768 ~= Tan[3*Pi/128] ~= 0.0737644315224493 */ \
    OD_DCT_OVERFLOW_CHECK(te, 2417, 16384, 166); \
    t1 += (te*2417 + 16384) >> 15; \
    /* 601/4096 ~= Sin[3*Pi/64] ~= 0.146730474455362 */ \
    OD_DCT_OVERFLOW_CHECK(t1, 601, 2048, 167); \
    te -= (t1*601 + 2048) >> 12; \
    /* 2417/32768 ~= Tan[3*Pi/128] ~= 0.0737644315224493 */ \
    OD_DCT_OVERFLOW_CHECK(te, 2417, 16384, 168); \
    t1 += (te*2417 + 16384) >> 15; \
    /* 14525/32768 ~= Tan[17*Pi/128] ~= 0.443269513890864 */ \
    OD_DCT_OVERFLOW_CHECK(t8, 14525, 16384, 169); \
    t7 -= (t8*14525 + 16384) >> 15; \
    /* 3035/4096 ~= Sin[17*Pi/64] ~= 0.740951125354959 */ \
    OD_DCT_OVERFLOW_CHECK(t7, 3035, 2048, 170); \
    t8 += (t7*3035 + 2048) >> 12; \
    /* 7263/16384 ~= Tan[17*Pi/128] ~= 0.443269513890864 */ \
    OD_DCT_OVERFLOW_CHECK(t8, 7263, 8192, 171); \
    t7 -= (t8*7263 + 8192) >> 14; \
    /* 6393/8192 ~= Tan[27*Pi/128] ~= 0.780407659653944 */ \
    OD_DCT_OVERFLOW_CHECK(td, 6393, 4096, 172); \
    t2 -= (td*6393 + 4096) >> 13; \
    /* 3973/4096 ~= Sin[27*Pi/64] ~= 0.970031253194544 */ \
    OD_DCT_OVERFLOW_CHECK(t2, 3973, 2048, 173); \
    td += (t2*3973 + 2048) >> 12; \
    /* 6393/8192 ~= Tan[27*Pi/128] ~= 0.780407659653944 */ \
    OD_DCT_OVERFLOW_CHECK(td, 6393, 4096, 174); \
    t2 -= (td*6393 + 4096) >> 13; \
    /* 9281/16384 ~= Tan[21*Pi/128] ~= 0.566493002730344 */ \
    OD_DCT_OVERFLOW_CHECK(ta, 9281, 8192, 175); \
    t5 -= (ta*9281 + 8192) >> 14; \
    /* 7027/8192 ~= Sin[21*Pi/64] ~= 0.857728610000272 */ \
    OD_DCT_OVERFLOW_CHECK(t5, 7027, 4096, 176); \
    ta += (t5*7027 + 4096) >> 13; \
    /* 9281/16384 ~= Tan[21*Pi/128] ~= 0.566493002730344 */ \
    OD_DCT_OVERFLOW_CHECK(ta, 9281, 8192, 177); \
    t5 -= (ta*9281 + 8192) >> 14; \
    /* 11539/16384 ~= Tan[25*Pi/128] ~= 0.704279460865044 */ \
    OD_DCT_OVERFLOW_CHECK(tc, 11539, 8192, 178); \
    t3 -= (tc*11539 + 8192) >> 14; \
    /* 7713/8192 ~= Sin[25*Pi/64] ~= 0.941544065183021 */ \
    OD_DCT_OVERFLOW_CHECK(t3, 7713, 4096, 179); \
    tc += (t3*7713 + 4096) >> 13; \
    /* 11539/16384 ~= Tan[25*Pi/128] ~= 0.704279460865044 */ \
    OD_DCT_OVERFLOW_CHECK(tc, 11539, 8192, 180); \
    t3 -= (tc*11539 + 8192) >> 14; \
    /* 10375/16384 ~= Tan[23*Pi/128] ~= 0.633243016177569 */ \
    OD_DCT_OVERFLOW_CHECK(tb, 10375, 8192, 181); \
    t4 -= (tb*10375 + 8192) >> 14; \
    /* 7405/8192 ~= Sin[23*Pi/64] ~= 0.903989293123443 */ \
    OD_DCT_OVERFLOW_CHECK(t4, 7405, 4096, 182); \
    tb += (t4*7405 + 4096) >> 13; \
    /* 10375/16384 ~= Tan[23*Pi/128] ~= 0.633243016177569 */ \
    OD_DCT_OVERFLOW_CHECK(tb, 10375, 8192, 183); \
    t4 -= (tb*10375 + 8192) >> 14; \
    /* 8247/16384 ~= Tan[19*Pi/128] ~= 0.503357699799294 */ \
    OD_DCT_OVERFLOW_CHECK(t9, 8247, 8192, 184); \
    t6 -= (t9*8247 + 8192) >> 14; \
    /* 1645/2048 ~= Sin[19*Pi/64] ~= 0.803207531480645 */ \
    OD_DCT_OVERFLOW_CHECK(t6, 1645, 1024, 185); \
    t9 += (t6*1645 + 1024) >> 11; \
    /* 8247/16384 ~= Tan[19*Pi/128] ~= 0.503357699799294 */ \
    OD_DCT_OVERFLOW_CHECK(t9, 8247, 8192, 186); \
    t6 -= (t9*8247 + 8192) >> 14; \
} while (0)

/* Embedded 16-point asymmetric Type-IV iDST. */
#define OD_IDST_16_ASYM(t0, t0h, t8, t4, tc, t2, t2h, ta, t6, te, teh, \
                        t1, t9, t5, td, t3, tb, t7, tf) \
do { \
    dctcoef t1h_; \
    dctcoef t3h_; \
    dctcoef t4h; \
    dctcoef t6h; \
    dctcoef t9h_; \
    dctcoef tbh_; \
    dctcoef tch; \
    /* 8247/16384 ~= Tan[19*Pi/128] ~= 0.503357699799294 */ \
    t6 += (t9*8247 + 8192) >> 14; \
    /* 1645/2048 ~= Sin[19*Pi/64] ~= 0.803207531480645 */ \
    t9 -= (t6*1645 + 1024) >> 11; \
    /* 8247/16384 ~= Tan[19*Pi/128] ~= 0.503357699799294 */ \
    t6 += (t9*8247 + 8192) >> 14; \
    /* 10375/16384 ~= Tan[23*Pi/128] ~= 0.633243016177569 */ \
    t2 += (td*10375 + 8192) >> 14; \
    /* 7405/8192 ~= Sin[23*Pi/64] ~= 0.903989293123443 */ \
    td -= (t2*7405 + 4096) >> 13; \
    /* 10375/16384 ~= Tan[23*Pi/128] ~= 0.633243016177569 */ \
    t2 += (td*10375 + 8192) >> 14; \
    /* 11539/16384 ~= Tan[25*Pi/128] ~= 0.704279460865044 */ \
    tc += (t3*11539 + 8192) >> 14; \
    /* 7713/8192 ~= Sin[25*Pi/64] ~= 0.941544065183021 */ \
    t3 -= (tc*7713 + 4096) >> 13; \
    /* 11539/16384 ~= Tan[25*Pi/128] ~= 0.704279460865044 */ \
    tc += (t3*11539 + 8192) >> 14; \
    /* 9281/16384 ~= Tan[21*Pi/128] ~= 0.566493002730344 */ \
    ta += (t5*9281 + 8192) >> 14; \
    /* 7027/8192 ~= Sin[21*Pi/64] ~= 0.857728610000272 */ \
    t5 -= (ta*7027 + 4096) >> 13; \
    /* 9281/16384 ~= Tan[21*Pi/128] ~= 0.566493002730344 */ \
    ta += (t5*9281 + 8192) >> 14; \
    /* 6393/8192 ~= Tan[27*Pi/128] ~= 0.780407659653944 */ \
    t4 += (tb*6393 + 4096) >> 13; \
    /* 3973/4096 ~= Sin[27*Pi/64] ~= 0.970031253194544 */ \
    tb -= (t4*3973 + 2048) >> 12; \
    /* 6393/8192 ~= Tan[27*Pi/128] ~= 0.780407659653944 */ \
    t4 += (tb*6393 + 4096) >> 13; \
    /* 7263/16384 ~= Tan[17*Pi/128] ~= 0.443269513890864 */ \
    te += (t1*7263 + 8192) >> 14; \
    /* 3035/4096 ~= Sin[17*Pi/64] ~= 0.740951125354959 */ \
    t1 -= (te*3035 + 2048) >> 12; \
    /* 14525/32768 ~= Tan[17*Pi/128] ~= 0.443269513890864 */ \
    te += (t1*14525 + 16384) >> 15; \
    /* 2417/32768 ~= Tan[3*Pi/128] ~= 0.0737644315224493 */ \
    t8 -= (t7*2417 + 16384) >> 15; \
    /* 601/4096 ~= Sin[3*Pi/64] ~= 0.146730474455362 */ \
    t7 += (t8*601 + 2048) >> 12; \
    /* 2417/32768 ~= Tan[3*Pi/128] ~= 0.0737644315224493 */ \
    t8 -= (t7*2417 + 16384) >> 15; \
    /* 7799/8192 ~= Tan[31*Pi/128] ~= 0.952079146700925 */ \
    t0 += (tf*7799 + 4096) >> 13; \
    /* 4091/4096 ~= Sin[31*Pi/64] ~= 0.998795456205172 */ \
    tf -= (t0*4091 + 2048) >> 12; \
    /* 7799/8192 ~= Tan[31*Pi/128] ~= 0.952079146700925 */ \
    t0 += (tf*7799 + 4096) >> 13; \
    /* TODO: Can we move these into another operation */ \
    t1 = -t1; \
    t3 = -t3; \
    t5 = -t5; \
    t9 = -t9; \
    tb = -tb; \
    td = -td; \
    tf = -tf; \
    t4 += ta; \
    t4h = OD_RSHIFT1(t4); \
    ta = t4h - ta; \
    tb -= t5; \
    tbh_ = OD_RSHIFT1(tb); \
    t5 += tbh_; \
    tc += t2; \
    tch = OD_RSHIFT1(tc); \
    t2 -= tch; \
    t3 -= td; \
    t3h_ = OD_RSHIFT1(t3); \
    td += t3h_; \
    t9 += t8; \
    t9h_ = OD_RSHIFT1(t9); \
    t8 -= t9h_; \
    t6 -= t7; \
    t6h = OD_RSHIFT1(t6); \
    t7 += t6h; \
    t1 += tf; \
    t1h_ = OD_RSHIFT1(t1); \
    tf -= t1h_; \
    te -= t0; \
    teh = OD_RSHIFT1(te); \
    t0 += teh; \
    ta += t9h_; \
    t9 = ta - t9; \
    t5 -= t6h; \
    t6 += t5; \
    td = teh - td; \
    te = td - te; \
    t2 = t1h_ - t2; \
    t1 -= t2; \
    t7 += t4h; \
    t4 -= t7; \
    t8 -= tbh_; \
    tb += t8; \
    t0 += tch; \
    tc -= t0; \
    tf -= t3h_; \
    t3 += tf; \
    /* TODO: Can we move this into another operation */ \
    ta = -ta; \
    /* 6723/8192 ~= Tan[7*Pi/32] ~= 0.820678790828660 */ \
    td += (t2*6723 + 4096) >> 13; \
    /* 16069/16384 ~= Sin[7*Pi/16] ~= 0.980785280403230 */ \
    t2 -= (td*16069 + 8192) >> 14; \
    /* 6723/8192 ~= Tan[7*Pi/32] ~= 0.820678790828660 */ \
    td += (t2*6723 + 4096) >> 13; \
    /* 2485/8192 ~= Tan[3*Pi/32] ~= 0.303346683607342 */ \
    t5 -= (ta*2485 + 4096) >> 13; \
    /* 18205/32768 ~= Sin[3*Pi/16] ~= 0.555570233019602 */ \
    ta += (t5*18205 + 16384) >> 15; \
    /* 2485/8192 ~= Tan[3*Pi/32] ~= 0.303346683607342 */ \
    t5 -= (ta*2485 + 4096) >> 13; \
    t2 += t5; \
    t2h = OD_RSHIFT1(t2); \
    t5 -= t2h; \
    ta = td - ta; \
    td -= OD_RSHIFT1(ta); \
    /* 13573/16384 ~= 2*Tan[Pi/8] ~= 0.828427124746190 */ \
    ta -= (t5*13573 + 8192) >> 14; \
    /* 11585/32768 ~= Sin[Pi/4]/2 ~= 0.353553390593274 */ \
    t5 += (ta*11585 + 16384) >> 15; \
    /* 13573/16384 ~= 2*Tan[Pi/8] ~= 0.828427124746190 */ \
    ta -= (t5*13573 + 8192) >> 14; \
    /* 17515/32768 ~= Tan[5*Pi/32] ~= 0.534511135950792 */ \
    t9 -= (t6*17515 + 16384) >> 15; \
    /* 13623/16384 ~= Sin[5*Pi/16] ~= 0.831469612302545 */ \
    t6 += (t9*13623 + 8192) >> 14; \
    /* 17515/32768 ~= Tan[5*Pi/32]) ~= 0.534511135950792 */ \
    t9 -= (t6*17515 + 16384) >> 15; \
    /* 6723/8192 ~= Tan[7*Pi/32]) ~= 0.820678790828660 */ \
    t1 -= (te*6723 + 4096) >> 13; \
    /* 16069/16384 ~= Sin[7*Pi/16] ~= 0.980785280403230 */ \
    te += (t1*16069 + 8192) >> 14; \
    /* 6723/8192 ~= Tan[7*Pi/32]) ~= 0.820678790828660 */ \
    t1 -= (te*6723 + 4096) >> 13; \
    te += t6; \
    teh = OD_RSHIFT1(te); \
    t6 = teh - t6; \
    t9 += t1; \
    t1 -= OD_RSHIFT1(t9); \
    /* -19195/32768 ~= Tan[Pi/8] - Tan[Pi/4] ~= -0.585786437626905 */ \
    t9 -= (t6*19195 + 16384) >> 15; \
    /* 11585/16384 ~= Sin[Pi/4] ~= 0.707106781186548 */ \
    t6 -= (t9*11585 + 8192) >> 14; \
    /* 7489/8192 ~= Tan[Pi/8] + Tan[Pi/4]/2 ~= 0.914213562373095 */ \
    t9 += (t6*7489 + 4096) >> 13; \
    tb = tc - tb; \
    tc = OD_RSHIFT1(tb) - tc; \
    t3 += t4; \
    t4 = OD_RSHIFT1(t3) - t4; \
    /* TODO: Can we move this into another operation */ \
    t3 = -t3; \
    t8 += tf; \
    tf = OD_RSHIFT1(t8) - tf; \
    t0 += t7; \
    t0h = OD_RSHIFT1(t0); \
    t7 = t0h - t7; \
    /* 4161/16384 ~= Tan[3*Pi/16] - Tan[Pi/8] ~= 0.253965075546204 */ \
    t3 += (tc*4161 + 8192) >> 14; \
    /* 15137/16384 ~= Sin[3*Pi/8] ~= 0.923879532511287 */ \
    tc -= (t3*15137 + 8192) >> 14; \
    /* 14341/16384 ~= Tan[3*Pi/16] + Tan[Pi/8]/2 ~= 0.875285419105846 */ \
    t3 += (tc*14341 + 8192) >> 14; \
    /* 14341/16384 ~= Tan[3*Pi/16] + Tan[Pi/8]/2 ~= 0.875285419105846 */ \
    t4 -= (tb*14341 + 8192) >> 14; \
    /* 15137/16384 ~= Sin[3*Pi/8] ~= 0.923879532511287 */ \
    tb += (t4*15137 + 8192) >> 14; \
    /* 4161/16384 ~= Tan[3*Pi/16] - Tan[Pi/8] ~= 0.253965075546204 */ \
    t4 -= (tb*4161 + 8192) >> 14; \
    /* 13573/16384 ~= 2*Tan[Pi/8] ~= 0.828427124746190 */ \
    t8 += (t7*13573 + 8192) >> 14; \
    /* 11585/32768 ~= Sin[Pi/4]/2 ~= 0.353553390593274 */ \
    t7 -= (t8*11585 + 16384) >> 15; \
    /* 13573/16384 ~= 2*Tan[Pi/8] ~= 0.828427124746190 */ \
    t8 += (t7*13573 + 8192) >> 14; \
    /* TODO: Can we move these into another operation */ \
    t1 = -t1; \
    t5 = -t5; \
    t9 = -t9; \
    tb = -tb; \
    td = -td; \
} while (0)

/* Embedded 32-point orthonormal Type-II fDCT. */
#define OD_FDCT_32(t0, tg, t8, to, t4, tk, tc, ts, t2, ti, ta, tq, t6, tm, \
                   te, tu, t1, th, t9, tp, t5, tl, td, tt, t3, tj, tb, tr, t7, tn, \
                   tf, tv) \
do { \
    dctcoef tgh; \
    dctcoef thh; \
    dctcoef tih; \
    dctcoef tkh; \
    dctcoef tmh; \
    dctcoef tnh; \
    dctcoef toh; \
    dctcoef tqh; \
    dctcoef tsh; \
    dctcoef tuh; \
    dctcoef tvh; \
    tv = t0 - tv; \
    tvh = OD_RSHIFT1(tv); \
    t0 -= tvh; \
    tu += t1; \
    tuh = OD_RSHIFT1(tu); \
    t1 = tuh - t1; \
    tt = t2 - tt; \
    t2 -= OD_RSHIFT1(tt); \
    ts += t3; \
    tsh = OD_RSHIFT1(ts); \
    t3 = tsh - t3; \
    tr = t4 - tr; \
    t4 -= OD_RSHIFT1(tr); \
    tq += t5; \
    tqh = OD_RSHIFT1(tq); \
    t5 = tqh - t5; \
    tp = t6 - tp; \
    t6 -= OD_RSHIFT1(tp); \
    to += t7; \
    toh = OD_RSHIFT1(to); \
    t7 = toh - t7; \
    tn = t8 - tn; \
    tnh = OD_RSHIFT1(tn); \
    t8 -= tnh; \
    tm += t9; \
    tmh = OD_RSHIFT1(tm); \
    t9 = tmh - t9; \
    tl = ta - tl; \
    ta -= OD_RSHIFT1(tl); \
    tk += tb; \
    tkh = OD_RSHIFT1(tk); \
    tb = tkh - tb; \
    tj = tc - tj; \
    tc -= OD_RSHIFT1(tj); \
    ti += td; \
    tih = OD_RSHIFT1(ti); \
    td = tih - td; \
    th = te - th; \
    thh = OD_RSHIFT1(th); \
    te -= thh; \
    tg += tf; \
    tgh = OD_RSHIFT1(tg); \
    tf = tgh - tf; \
    OD_FDCT_16_ASYM(t0, tg, tgh, t8, to, toh, t4, tk, tkh, tc, ts, tsh, \
     t2, ti, tih, ta, tq, tqh, t6, tm, tmh, te, tu, tuh); \
    OD_FDST_16_ASYM(tv, tvh, tf, tn, tnh, t7, tr, tb, tj, t3, \
     tt, td, tl, t5, tp, t9, th, thh, t1); \
} while (0)

/* Embedded 32-point orthonormal Type-II iDCT. */
#define OD_IDCT_32(t0, tg, t8, to, t4, tk, tc, ts, t2, ti, ta, tq, t6, tm, \
                   te, tu, t1, th, t9, tp, t5, tl, td, tt, t3, tj, tb, tr, t7, tn, \
                   tf, tv) \
do { \
    dctcoef t1h; \
    dctcoef t3h; \
    dctcoef t5h; \
    dctcoef t7h; \
    dctcoef t9h; \
    dctcoef tbh; \
    dctcoef tdh; \
    dctcoef tfh; \
    dctcoef thh; \
    dctcoef tth; \
    dctcoef tvh; \
    OD_IDST_16_ASYM(tv, tvh, tn, tr, tj, tt, tth, tl, tp, th, thh, \
     tu, tm, tq, ti, ts, tk, to, tg); \
    OD_IDCT_16_ASYM(t0, t8, t4, tc, t2, ta, t6, te, \
     t1, t1h, t9, t9h, t5, t5h, td, tdh, t3, t3h, tb, tbh, t7, t7h, tf, tfh); \
    tu = t1h - tu; \
    t1 -= tu; \
    te += thh; \
    th = te - th; \
    tm = t9h - tm; \
    t9 -= tm; \
    t6 += OD_RSHIFT1(tp); \
    tp = t6 - tp; \
    tq = t5h - tq; \
    t5 -= tq; \
    ta += OD_RSHIFT1(tl); \
    tl = ta - tl; \
    ti = tdh - ti; \
    td -= ti; \
    t2 += tth; \
    tt = t2 - tt; \
    ts = t3h - ts; \
    t3 -= ts; \
    tc += OD_RSHIFT1(tj); \
    tj = tc - tj; \
    tk = tbh - tk; \
    tb -= tk; \
    t4 += OD_RSHIFT1(tr); \
    tr = t4 - tr; \
    to = t7h - to; \
    t7 -= to; \
    t8 += OD_RSHIFT1(tn); \
    tn = t8 - tn; \
    tg = tfh - tg; \
    tf -= tg; \
    t0 += tvh; \
    tv = t0 - tv; \
} while (0)

/* 117 "muls", 117 + 128 = 245 adds, 36 shifts */
/* Embedded 32-point orthonormal Type-IV fDST. */
#define OD_FDST_32(t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, ta, tb, tc, td, te, tf, \
                   tg, th, ti, tj, tk, tl, tm, tn, to, tp, tq, tr, ts, tt, tu, tv) \
do { \
    dctcoef t0h; \
    dctcoef t1h; \
    dctcoef t2h; \
    dctcoef t3h; \
    dctcoef t4h; \
    dctcoef t6h; \
    dctcoef t8h; \
    dctcoef t9h; \
    dctcoef tah; \
    dctcoef tbh; \
    dctcoef tch; \
    dctcoef tdh; \
    dctcoef teh; \
    dctcoef tfh; \
    dctcoef tgh; \
    dctcoef thh; \
    dctcoef tih; \
    dctcoef tjh; \
    dctcoef tkh; \
    dctcoef tlh; \
    dctcoef tmh; \
    dctcoef tnh; \
    dctcoef tph; \
    dctcoef trh; \
    dctcoef tsh; \
    dctcoef tth; \
    dctcoef tuh; \
    dctcoef tvh; \
    /* Stage 0 */ \
    tp += (t6*659 + 2048) >> 12; \
    t6 -= (tp*10279 + 16384) >> 15; \
    tp += (t6*659 + 2048) >> 12; \
    th += (te*3045 + 4096) >> 13; \
    te -= (th*21403 + 16384) >> 15; \
    th += (te*3045 + 4096) >> 13; \
    t9 += (tm*20191 + 16384) >> 15; \
    tm -= (t9*29269 + 16384) >> 15; \
    t9 += (tm*20191 + 16384) >> 15; \
    tu += (t1*1207 + 16384) >> 15; \
    t1 -= (tu*2411 + 16384) >> 15; \
    tu += (t1*1207 + 16384) >> 15; \
    t4 += (tr*13113 + 8192) >> 14; \
    tr -= (t4*7993 + 4096) >> 13; \
    t4 += (tr*13113 + 8192) >> 14; \
    tj += (tc*10381 + 16384) >> 15; \
    tc -= (tj*4717 + 4096) >> 13; \
    tj += (tc*10381 + 16384) >> 15; \
    tb += (tk*18035 + 16384) >> 15; \
    tk -= (tb*6921 + 4096) >> 13; \
    tb += (tk*18035 + 16384) >> 15; \
    ts += (t3*1411 + 8192) >> 14; \
    t3 -= (ts*2801 + 8192) >> 14; \
    ts += (t3*1411 + 8192) >> 14; \
    tq += (t5*2225 + 8192) >> 14; \
    t5 -= (tq*2185 + 4096) >> 13; \
    tq += (t5*2225 + 8192) >> 14; \
    ti += (td*11273 + 16384) >> 15; \
    td -= (ti*315 + 256) >> 9; \
    ti += (td*11273 + 16384) >> 15; \
    tl += (ta*8637 + 16384) >> 15; \
    ta -= (tl*16151 + 16384) >> 15; \
    tl += (ta*8637 + 16384) >> 15; \
    tt += (t2*2013 + 16384) >> 15; \
    t2 -= (tt*4011 + 16384) >> 15; \
    tt += (t2*2013 + 16384) >> 15; \
    to += (t7*6101 + 16384) >> 15; \
    t7 -= (to*11793 + 16384) >> 15; \
    to += (t7*6101 + 16384) >> 15; \
    t8 += (tn*10659 + 8192) >> 14; \
    tn -= (t8*29957 + 16384) >> 15; \
    t8 += (tn*10659 + 8192) >> 14; \
    tg += (tf*819 + 1024) >> 11; \
    tf -= (tg*22595 + 16384) >> 15; \
    tg += (tf*819 + 1024) >> 11; \
    t0 += (tv*31973 + 16384) >> 15; \
    tv -= (t0*16379 + 8192) >> 14; \
    t0 += (tv*31973 + 16384) >> 15; \
    /* Stage 1 */ \
    tj -= ts; \
    tjh = OD_RSHIFT1(tj); \
    ts += tjh; \
    tr = tk - tr; \
    trh = OD_RSHIFT1(tr); \
    tk = trh - tk; \
    tc += t3; \
    tch = OD_RSHIFT1(tc); \
    t3 -= tch; \
    t4 += tb; \
    t4h = OD_RSHIFT1(t4); \
    tb -= t4h; \
    tv += tf; \
    tvh = OD_RSHIFT1(tv); \
    tf -= tvh; \
    t8 -= to; \
    t8h = OD_RSHIFT1(t8); \
    to += t8h; \
    t0 += tg; \
    t0h = OD_RSHIFT1(t0); \
    tg -= t0h; \
    tn = t7 - tn; \
    tnh = OD_RSHIFT1(tn); \
    t7 -= tnh; \
    th -= tu; \
    thh = OD_RSHIFT1(th); \
    tu += thh; \
    t6 += tm; \
    t6h = OD_RSHIFT1(t6); \
    tm = t6h - tm; \
    te += t1; \
    teh = OD_RSHIFT1(te); \
    t1 -= teh; \
    tp += t9; \
    tph = OD_RSHIFT1(tp); \
    t9 -= tph; \
    t2 -= td; \
    t2h = OD_RSHIFT1(t2); \
    td += t2h; \
    tl = tq - tl; \
    tlh = OD_RSHIFT1(tl); \
    tq -= tlh; \
    tt += ti; \
    tth = OD_RSHIFT1(tt); \
    ti -= tth; \
    ta += t5; \
    tah = OD_RSHIFT1(ta); \
    t5 -= tah; \
    /* Stage 2 */ \
    tm -= thh; \
    th += tm; \
    t9 = teh - t9; \
    te -= t9; \
    td = tlh - td; \
    tl -= td; \
    ti += tah; \
    ta -= ti; \
    tk = tjh - tk; \
    tj -= tk; \
    tb -= tch; \
    tc += tb; \
    tg += tnh; \
    tn = tg - tn; \
    tf += t8h; \
    t8 = tf - t8; \
    t3 -= trh; \
    tr += t3; \
    ts += t4h; \
    t4 -= ts; \
    to -= t0h; \
    t0 += to; \
    t7 = tvh - t7; \
    tv = t7 - tv; \
    t1 -= t6h; \
    t6 += t1; \
    tu += tph; \
    tp -= tu; \
    tq -= tth; \
    tt += tq; \
    t5 += t2h; \
    t2 -= t5; \
    /* Stage 3 */ \
    tj += (tc*11725 + 16384) >> 15; \
    tc -= (tj*5197 + 4096) >> 13; \
    tj += (tc*11725 + 16384) >> 15; \
    td += (ti*513 + 1024) >> 11; \
    ti -= (td*15447 + 16384) >> 15; \
    td += (ti*513 + 1024) >> 11; \
    th += (te*4861 + 16384) >> 15; \
    te -= (th*1189 + 2048) >> 12; \
    th += (te*4861 + 16384) >> 15; \
    tg += (tf*805 + 8192) >> 14; \
    tf -= (tg*803 + 4096) >> 13; \
    tg += (tf*805 + 8192) >> 14; \
    tb += (tk*7749 + 8192) >> 14; \
    tk -= (tb*12665 + 8192) >> 14; \
    tb += (tk*7749 + 8192) >> 14; \
    tl += (ta*2455 + 2048) >> 12; \
    ta -= (tl*28899 + 16384) >> 15; \
    tl += (ta*2455 + 2048) >> 12; \
    t9 += (tm*12151 + 8192) >> 14; \
    tm -= (t9*31357 + 16384) >> 15; \
    t9 += (tm*12151 + 8192) >> 14; \
    tn += (t8*29699 + 16384) >> 15; \
    t8 -= (tn*16305 + 8192) >> 14; \
    tn += (t8*29699 + 16384) >> 15; \
    /* Stage 4 */ \
    tf -= tc; \
    tfh = OD_RSHIFT1(tf); \
    tc += tfh; \
    ti = th - ti; \
    tih = OD_RSHIFT1(ti); \
    th -= tih; \
    tg += tj; \
    tgh = OD_RSHIFT1(tg); \
    tj = tgh - tj; \
    td -= te; \
    tdh = OD_RSHIFT1(td); \
    te += tdh; \
    tm = ta - tm; \
    tmh = OD_RSHIFT1(tm); \
    ta = tmh - ta; \
    t9 += tl; \
    t9h = OD_RSHIFT1(t9); \
    tl -= t9h; \
    tb += t8; \
    tbh = OD_RSHIFT1(tb); \
    t8 -= tbh; \
    tk += tn; \
    tkh = OD_RSHIFT1(tk); \
    tn -= tkh; \
    t1 -= t2; \
    t1h = OD_RSHIFT1(t1); \
    t2 += t1h; \
    t3 += tv; \
    t3h = OD_RSHIFT1(t3); \
    tv -= t3h; \
    tu += tt; \
    tuh = OD_RSHIFT1(tu); \
    tt -= tuh; \
    ts -= t0; \
    tsh = OD_RSHIFT1(ts); \
    t0 += tsh; \
    tq = t6 - tq; \
    t6 -= OD_RSHIFT1(tq); \
    to += tr; \
    tr = OD_RSHIFT1(to) - tr; \
    t7 = t4 - t7; \
    t4 -= OD_RSHIFT1(t7); \
    t5 -= tp; \
    tp += OD_RSHIFT1(t5); \
    /* Stage 5 */ \
    tp += (t6*2485 + 4096) >> 13; \
    t6 -= (tp*18205 + 16384) >> 15; \
    tp += (t6*2485 + 4096) >> 13; \
    to += (t7*3227 + 16384) >> 15; \
    t7 -= (to*6393 + 16384) >> 15; \
    to += (t7*3227 + 16384) >> 15; \
    tq += (t5*17515 + 16384) >> 15; \
    t5 -= (tq*13623 + 8192) >> 14; \
    tq += (t5*17515 + 16384) >> 15; \
    t4 += (tr*6723 + 4096) >> 13; \
    tr -= (t4*16069 + 8192) >> 14; \
    t4 += (tr*6723 + 4096) >> 13; \
    /* Stage 6 */ \
    tj += tdh; \
    td -= tj; \
    tc -= tih; \
    ti += tc; \
    th = tgh - th; \
    tg -= th; \
    te += tfh; \
    tf -= te; \
    tl = tkh - tl; \
    tk -= tl; \
    ta += tbh; \
    tb -= ta; \
    tn -= tmh; \
    tm += tn; \
    t8 += t9h; \
    t9 = t8 - t9; \
    tt = t3h - tt; \
    t3 -= tt; \
    t2 -= tsh; \
    ts += t2; \
    tv -= t1h; \
    t1 += tv; \
    t0 += tuh; \
    tu -= t0; \
    tp = OD_RSHIFT1(to) - tp; \
    to -= tp; \
    t6 += OD_RSHIFT1(t7); \
    t7 -= t6; \
    t4 = OD_RSHIFT1(tq) - t4; \
    tq -= t4; \
    tr += OD_RSHIFT1(t5); \
    t5 = tr - t5; \
    /* Stage 7 */ \
    td += (ti*21895 + 16384) >> 15; \
    ti -= (td*15137 + 8192) >> 14; \
    td += (ti*21895 + 16384) >> 15; \
    tj += (tc*21895 + 16384) >> 15; \
    tc -= (tj*15137 + 8192) >> 14; \
    tj += (tc*21895 + 16384) >> 15; \
    th += (te*13573 + 16384) >> 15; \
    te -= (th*11585 + 8192) >> 14; \
    th += (te*13573 + 16384) >> 15; \
    tb += (tk*21895 + 16384) >> 15; \
    tk -= (tb*15137 + 8192) >> 14; \
    tb += (tk*21895 + 16384) >> 15; \
    ta += (tl*3259 + 8192) >> 14; \
    tl -= (ta*3135 + 4096) >> 13; \
    ta += (tl*3259 + 8192) >> 14; \
    t9 += (tm*13573 + 16384) >> 15; \
    tm -= (t9*11585 + 8192) >> 14; \
    t9 += (tm*13573 + 16384) >> 15; \
    ts += (t3*3259 + 8192) >> 14; \
    t3 -= (ts*3135 + 4096) >> 13; \
    ts += (t3*3259 + 8192) >> 14; \
    t2 += (tt*3259 + 8192) >> 14; \
    tt -= (t2*3135 + 4096) >> 13; \
    t2 += (tt*3259 + 8192) >> 14; \
    tu += (t1*13573 + 16384) >> 15; \
    t1 -= (tu*11585 + 8192) >> 14; \
    tu += (t1*13573 + 16384) >> 15; \
    tp += (t6*13573 + 16384) >> 15; \
    t6 -= (tp*11585 + 8192) >> 14; \
    tp += (t6*13573 + 16384) >> 15; \
    tq += (t5*13573 + 16384) >> 15; \
    t5 -= (tq*11585 + 8192) >> 14; \
    tq += (t5*13573 + 16384) >> 15; \
} while (0)

/* 117 "muls", 117 + 128 = 245 adds, 36 shifts */
/* Embedded 32-point orthonormal Type-IV iDST. */
#define OD_IDST_32(t0, tg, t8, to, t4, tk, tc, ts, t2, ti, ta, tq, t6, tm, te, tu, \
                   t1, th, t9, tp, t5, tl, td, tt, t3, tj, tb, tr, t7, tn, tf, tv) \
do { \
    dctcoef t0h; \
    dctcoef t1h; \
    dctcoef t2h; \
    dctcoef t3h; \
    dctcoef t4h; \
    dctcoef t6h; \
    dctcoef t8h; \
    dctcoef t9h; \
    dctcoef tah; \
    dctcoef tbh; \
    dctcoef tch; \
    dctcoef tdh; \
    dctcoef teh; \
    dctcoef tfh; \
    dctcoef tgh; \
    dctcoef thh; \
    dctcoef tih; \
    dctcoef tjh; \
    dctcoef tkh; \
    dctcoef tlh; \
    dctcoef tmh; \
    dctcoef tnh; \
    dctcoef tph; \
    dctcoef trh; \
    dctcoef tsh; \
    dctcoef tth; \
    dctcoef tuh; \
    dctcoef tvh; \
    /* Stage 0 */ \
    tq -= (t5*13573 + 16384) >> 15; \
    t5 += (tq*11585 + 8192) >> 14; \
    tq -= (t5*13573 + 16384) >> 15; \
    tp -= (t6*13573 + 16384) >> 15; \
    t6 += (tp*11585 + 8192) >> 14; \
    tp -= (t6*13573 + 16384) >> 15; \
    tu -= (t1*13573 + 16384) >> 15; \
    t1 += (tu*11585 + 8192) >> 14; \
    tu -= (t1*13573 + 16384) >> 15; \
    t2 -= (tt*3259 + 8192) >> 14; \
    tt += (t2*3135 + 4096) >> 13; \
    t2 -= (tt*3259 + 8192) >> 14; \
    ts -= (t3*3259 + 8192) >> 14; \
    t3 += (ts*3135 + 4096) >> 13; \
    ts -= (t3*3259 + 8192) >> 14; \
    t9 -= (tm*13573 + 16384) >> 15; \
    tm += (t9*11585 + 8192) >> 14; \
    t9 -= (tm*13573 + 16384) >> 15; \
    ta -= (tl*3259 + 8192) >> 14; \
    tl += (ta*3135 + 4096) >> 13; \
    ta -= (tl*3259 + 8192) >> 14; \
    tb -= (tk*21895 + 16384) >> 15; \
    tk += (tb*15137 + 8192) >> 14; \
    tb -= (tk*21895 + 16384) >> 15; \
    th -= (te*13573 + 16384) >> 15; \
    te += (th*11585 + 8192) >> 14; \
    th -= (te*13573 + 16384) >> 15; \
    tj -= (tc*21895 + 16384) >> 15; \
    tc += (tj*15137 + 8192) >> 14; \
    tj -= (tc*21895 + 16384) >> 15; \
    td -= (ti*21895 + 16384) >> 15; \
    ti += (td*15137 + 8192) >> 14; \
    td -= (ti*21895 + 16384) >> 15; \
    /* Stage 1 */ \
    t5 = tr - t5; \
    tr -= OD_RSHIFT1(t5); \
    tq += t4; \
    t4 = OD_RSHIFT1(tq) - t4; \
    t7 += t6; \
    t6 -= OD_RSHIFT1(t7); \
    to += tp; \
    tp = OD_RSHIFT1(to) - tp; \
    tu += t0; \
    tuh = OD_RSHIFT1(tu); \
    t0 -= tuh; \
    t1 -= tv; \
    t1h = OD_RSHIFT1(t1); \
    tv += t1h; \
    ts -= t2; \
    tsh = OD_RSHIFT1(ts); \
    t2 += tsh; \
    t3 += tt; \
    t3h = OD_RSHIFT1(t3); \
    tt = t3h - tt; \
    t9 = t8 - t9; \
    t9h = OD_RSHIFT1(t9); \
    t8 -= t9h; \
    tm -= tn; \
    tmh = OD_RSHIFT1(tm); \
    tn += tmh; \
    tb += ta; \
    tbh = OD_RSHIFT1(tb); \
    ta -= tbh; \
    tk += tl; \
    tkh = OD_RSHIFT1(tk); \
    tl = tkh - tl; \
    tf += te; \
    tfh = OD_RSHIFT1(tf); \
    te -= tfh; \
    tg += th; \
    tgh = OD_RSHIFT1(tg); \
    th = tgh - th; \
    ti -= tc; \
    tih = OD_RSHIFT1(ti); \
    tc += tih; \
    td += tj; \
    tdh = OD_RSHIFT1(td); \
    tj -= tdh; \
    /* Stage 2 */ \
    t4 -= (tr*6723 + 4096) >> 13; \
    tr += (t4*16069 + 8192) >> 14; \
    t4 -= (tr*6723 + 4096) >> 13; \
    tq -= (t5*17515 + 16384) >> 15; \
    t5 += (tq*13623 + 8192) >> 14; \
    tq -= (t5*17515 + 16384) >> 15; \
    to -= (t7*3227 + 16384) >> 15; \
    t7 += (to*6393 + 16384) >> 15; \
    to -= (t7*3227 + 16384) >> 15; \
    tp -= (t6*2485 + 4096) >> 13; \
    t6 += (tp*18205 + 16384) >> 15; \
    tp -= (t6*2485 + 4096) >> 13; \
    /* Stage 3 */ \
    tp -= OD_RSHIFT1(t5); \
    t5 += tp; \
    t4 += OD_RSHIFT1(t7); \
    t7 = t4 - t7; \
    tr = OD_RSHIFT1(to) - tr; \
    to -= tr; \
    t6 += OD_RSHIFT1(tq); \
    tq = t6 - tq; \
    t0 -= tsh; \
    ts += t0; \
    tt += tuh; \
    tu -= tt; \
    tv += t3h; \
    t3 -= tv; \
    t2 -= t1h; \
    t1 += t2; \
    tn += tkh; \
    tk -= tn; \
    t8 += tbh; \
    tb -= t8; \
    tl += t9h; \
    t9 -= tl; \
    ta = tmh - ta; \
    tm = ta - tm; \
    te -= tdh; \
    td += te; \
    tj = tgh - tj; \
    tg -= tj; \
    th += tih; \
    ti = th - ti; \
    tc -= tfh; \
    tf += tc; \
    /* Stage 4 */ \
    tn -= (t8*29699 + 16384) >> 15; \
    t8 += (tn*16305 + 8192) >> 14; \
    tn -= (t8*29699 + 16384) >> 15; \
    t9 -= (tm*12151 + 8192) >> 14; \
    tm += (t9*31357 + 16384) >> 15; \
    t9 -= (tm*12151 + 8192) >> 14; \
    tl -= (ta*2455 + 2048) >> 12; \
    ta += (tl*28899 + 16384) >> 15; \
    tl -= (ta*2455 + 2048) >> 12; \
    tb -= (tk*7749 + 8192) >> 14; \
    tk += (tb*12665 + 8192) >> 14; \
    tb -= (tk*7749 + 8192) >> 14; \
    tg -= (tf*805 + 8192) >> 14; \
    tf += (tg*803 + 4096) >> 13; \
    tg -= (tf*805 + 8192) >> 14; \
    th -= (te*4861 + 16384) >> 15; \
    te += (th*1189 + 2048) >> 12; \
    th -= (te*4861 + 16384) >> 15; \
    td -= (ti*513 + 1024) >> 11; \
    ti += (td*15447 + 16384) >> 15; \
    td -= (ti*513 + 1024) >> 11; \
    tj -= (tc*11725 + 16384) >> 15; \
    tc += (tj*5197 + 4096) >> 13; \
    tj -= (tc*11725 + 16384) >> 15; \
    /* Stage 5 */ \
    t2 += t5; \
    t2h = OD_RSHIFT1(t2); \
    t5 -= t2h; \
    tt -= tq; \
    tth = OD_RSHIFT1(tt); \
    tq += tth; \
    tp += tu; \
    tph = OD_RSHIFT1(tp); \
    tu -= tph; \
    t6 -= t1; \
    t6h = OD_RSHIFT1(t6); \
    t1 += t6h; \
    tv = t7 - tv; \
    tvh = OD_RSHIFT1(tv); \
    t7 = tvh - t7; \
    t0 -= to; \
    t0h = OD_RSHIFT1(t0); \
    to += t0h; \
    t4 += ts; \
    t4h = OD_RSHIFT1(t4); \
    ts -= t4h; \
    tr -= t3; \
    trh = OD_RSHIFT1(tr); \
    t3 += trh; \
    t8 = tf - t8; \
    t8h = OD_RSHIFT1(t8); \
    tf -= t8h; \
    tn = tg - tn; \
    tnh = OD_RSHIFT1(tn); \
    tg -= tnh; \
    tc -= tb; \
    tch = OD_RSHIFT1(tc); \
    tb += tch; \
    tj += tk; \
    tjh = OD_RSHIFT1(tj); \
    tk = tjh - tk; \
    ta += ti; \
    tah = OD_RSHIFT1(ta); \
    ti -= tah; \
    tl += td; \
    tlh = OD_RSHIFT1(tl); \
    td = tlh - td; \
    te += t9; \
    teh = OD_RSHIFT1(te); \
    t9 = teh - t9; \
    th -= tm; \
    thh = OD_RSHIFT1(th); \
    tm += thh; \
    /* Stage 6 */ \
    t5 += tah; \
    ta -= t5; \
    ti += tth; \
    tt -= ti; \
    tq += tlh; \
    tl = tq - tl; \
    td -= t2h; \
    t2 += td; \
    t9 += tph; \
    tp -= t9; \
    t1 += teh; \
    te -= t1; \
    tm = t6h - tm; \
    t6 -= tm; \
    tu -= thh; \
    th += tu; \
    t7 += tnh; \
    tn = t7 - tn; \
    tg += t0h; \
    t0 -= tg; \
    to -= t8h; \
    t8 += to; \
    tf += tvh; \
    tv -= tf; \
    tb += t4h; \
    t4 -= tb; \
    t3 += tch; \
    tc -= t3; \
    tk = trh - tk; \
    tr = tk - tr; \
    ts -= tjh; \
    tj += ts; \
    /* Stage 7 */ \
    t0 -= (tv*31973 + 16384) >> 15; \
    tv += (t0*16379 + 8192) >> 14; \
    t0 -= (tv*31973 + 16384) >> 15; \
    tg -= (tf*819 + 1024) >> 11; \
    tf += (tg*22595 + 16384) >> 15; \
    tg -= (tf*819 + 1024) >> 11; \
    t8 -= (tn*10659 + 8192) >> 14; \
    tn += (t8*29957 + 16384) >> 15; \
    t8 -= (tn*10659 + 8192) >> 14; \
    to -= (t7*6101 + 16384) >> 15; \
    t7 += (to*11793 + 16384) >> 15; \
    to -= (t7*6101 + 16384) >> 15; \
    tt -= (t2*2013 + 16384) >> 15; \
    t2 += (tt*4011 + 16384) >> 15; \
    tt -= (t2*2013 + 16384) >> 15; \
    tl -= (ta*8637 + 16384) >> 15; \
    ta += (tl*16151 + 16384) >> 15; \
    tl -= (ta*8637 + 16384) >> 15; \
    ti -= (td*11273 + 16384) >> 15; \
    td += (ti*315 + 256) >> 9; \
    ti -= (td*11273 + 16384) >> 15; \
    tq -= (t5*2225 + 8192) >> 14; \
    t5 += (tq*2185 + 4096) >> 13; \
    tq -= (t5*2225 + 8192) >> 14; \
    ts -= (t3*1411 + 8192) >> 14; \
    t3 += (ts*2801 + 8192) >> 14; \
    ts -= (t3*1411 + 8192) >> 14; \
    tb -= (tk*18035 + 16384) >> 15; \
    tk += (tb*6921 + 4096) >> 13; \
    tb -= (tk*18035 + 16384) >> 15; \
    tj -= (tc*10381 + 16384) >> 15; \
    tc += (tj*4717 + 4096) >> 13; \
    tj -= (tc*10381 + 16384) >> 15; \
    t4 -= (tr*13113 + 8192) >> 14; \
    tr += (t4*7993 + 4096) >> 13; \
    t4 -= (tr*13113 + 8192) >> 14; \
    tu -= (t1*1207 + 16384) >> 15; \
    t1 += (tu*2411 + 16384) >> 15; \
    tu -= (t1*1207 + 16384) >> 15; \
    t9 -= (tm*20191 + 16384) >> 15; \
    tm += (t9*29269 + 16384) >> 15; \
    t9 -= (tm*20191 + 16384) >> 15; \
    th -= (te*3045 + 4096) >> 13; \
    te += (th*21403 + 16384) >> 15; \
    th -= (te*3045 + 4096) >> 13; \
    tp -= (t6*659 + 2048) >> 12; \
    t6 += (tp*10279 + 16384) >> 15; \
    tp -= (t6*659 + 2048) >> 12; \
} while (0)

/* Embedded 32-point asymmetric Type-II fDCT. */
#define OD_FDCT_32_ASYM(t0, tg, tgh, t8, to, toh, t4, tk, tkh, tc, ts, tsh, \
                        t2, ti, tih, ta, tq, tqh, t6, tm, tmh, te, tu, tuh, t1, th, \
                        thh, t9, tp, tph, t5, tl, tlh, td, tt, tth, t3, tj, tjh, tb, \
                        tr, trh, t7, tn, tnh, tf, tv, tvh) \
do { \
    t0 += tvh; \
    tv = t0 - tv; \
    t1 = tuh - t1; \
    tu -= t1; \
    t2 += tth; \
    tt = t2 - tt; \
    t3 = tsh - t3; \
    ts -= t3; \
    t4 += trh; \
    tr = t4 - tr; \
    t5 = tqh - t5; \
    tq -= t5; \
    t6 += tph; \
    tp = t6 - tp; \
    t7 = toh - t7; \
    to -= t7; \
    t8 += tnh; \
    tn = t8 - tn; \
    t9 = tmh - t9; \
    tm -= t9; \
    ta += tlh; \
    tl = ta - tl; \
    tb = tkh - tb; \
    tk -= tb; \
    tc += tjh; \
    tj = tc - tj; \
    td = tih - td; \
    ti -= td; \
    te += thh; \
    th = te - th; \
    tf = tgh - tf; \
    tg -= tf; \
    OD_FDCT_16(t0, tg, t8, to, t4, tk, tc, ts, \
     t2, ti, ta, tq, t6, tm, te, tu); \
    OD_FDST_16(tv, tf, tn, t7, tr, tb, tj, t3, \
     tt, td, tl, t5, tp, t9, th, t1); \
} while (0)

/* Embedded 32-point asymmetric Type-II iDCT. */
#define OD_IDCT_32_ASYM(t0, tg, t8, to, t4, tk, tc, ts, t2, ti, ta, tq, \
                        t6, tm, te, tu, t1, t1h, th, thh, t9, t9h, tp, tph, t5, t5h, \
                        tl, tlh, td, tdh, tt, tth, t3, t3h, tj, tjh, tb, tbh, tr, trh, \
                        t7, t7h, tn, tnh, tf, tfh, tv, tvh) \
do { \
    OD_IDST_16(tv, tn, tr, tj, tt, tl, tp, th, \
     tu, tm, tq, ti, ts, tk, to, tg); \
    OD_IDCT_16(t0, t8, t4, tc, t2, ta, t6, te, \
     t1, t9, t5, td, t3, tb, t7, tf); \
    tv = t0 - tv; \
    tvh = OD_RSHIFT1(tv); \
    t0 -= tvh; \
    t1 += tu; \
    t1h = OD_RSHIFT1(t1); \
    tu = t1h - tu; \
    tt = t2 - tt; \
    tth = OD_RSHIFT1(tt); \
    t2 -= tth; \
    t3 += ts; \
    t3h = OD_RSHIFT1(t3); \
    ts = t3h - ts; \
    tr = t4 - tr; \
    trh = OD_RSHIFT1(tr); \
    t4 -= trh; \
    t5 += tq; \
    t5h = OD_RSHIFT1(t5); \
    tq = t5h - tq; \
    tp = t6 - tp; \
    tph = OD_RSHIFT1(tp); \
    t6 -= tph; \
    t7 += to; \
    t7h = OD_RSHIFT1(t7); \
    to = t7h - to; \
    tn = t8 - tn; \
    tnh = OD_RSHIFT1(tn); \
    t8 -= tnh; \
    t9 += tm; \
    t9h = OD_RSHIFT1(t9); \
    tm = t9h - tm; \
    tl = ta - tl; \
    tlh = OD_RSHIFT1(tl); \
    ta -= tlh; \
    tb += tk; \
    tbh = OD_RSHIFT1(tb); \
    tk = tbh - tk; \
    tj = tc - tj; \
    tjh = OD_RSHIFT1(tj); \
    tc -= tjh; \
    td += ti; \
    tdh = OD_RSHIFT1(td); \
    ti = tdh - ti; \
    th = te - th; \
    thh = OD_RSHIFT1(th); \
    te -= thh; \
    tf += tg; \
    tfh = OD_RSHIFT1(tf); \
    tg = tfh - tg; \
} while (0)

/* Embedded 32-point asymmetric Type-IV fDST. */
#define OD_FDST_32_ASYM(t0, tg, t8, to, t4, tk, tc, ts, t2, ti, ta, tq, t6, \
                        tm, te, tu, t1, th, t9, tp, t5, tl, td, tt, t3, tj, tb, tr, \
                        t7, tn, tf, tv) \
do { \
    dctcoef t0h; \
    dctcoef t1h; \
    dctcoef t4h; \
    dctcoef t5h; \
    dctcoef tqh; \
    dctcoef trh; \
    dctcoef tuh; \
    dctcoef tvh; \
    \
    tu = -tu; \
    \
    /* 13573/16384 ~= 2*Tan[Pi/8] ~= 0.828427124746190 */ \
    OD_DCT_OVERFLOW_CHECK(tq, 13573, 8192, 271); \
    t5 -= (tq*13573 + 8192) >> 14; \
    /* 11585/32768 ~= Sin[Pi/4]/2 ~= 0.353553390593274 */ \
    OD_DCT_OVERFLOW_CHECK(t5, 11585, 16384, 272); \
    tq += (t5*11585 + 16384) >> 15; \
    /* 13573/16384 ~= 2*Tan[Pi/8] ~= 0.828427124746190 */ \
    OD_DCT_OVERFLOW_CHECK(tq, 13573, 8192, 273); \
    t5 -= (tq*13573 + 8192) >> 14; \
    /* 29957/32768 ~= Tan[Pi/8] + Tan[Pi/4]/2 ~= 0.914213562373095 */ \
    OD_DCT_OVERFLOW_CHECK(t6, 29957, 16384, 274); \
    tp += (t6*29957 + 16384) >> 15; \
    /* 11585/16384 ~= Sin[Pi/4] ~= 0.707106781186548 */ \
    OD_DCT_OVERFLOW_CHECK(tp, 11585, 8192, 275); \
    t6 -= (tp*11585 + 8192) >> 14; \
    /* -19195/32768 ~= Tan[Pi/8] - Tan[Pi/4] ~= -0.585786437626905 */ \
    OD_DCT_OVERFLOW_CHECK(t6, 19195, 16384, 276); \
    tp -= (t6*19195 + 16384) >> 15; \
    /* 29957/32768 ~= Tan[Pi/8] + Tan[Pi/4]/2 ~= 0.914213562373095 */ \
    OD_DCT_OVERFLOW_CHECK(t1, 29957, 16384, 277); \
    tu += (t1*29957 + 16384) >> 15; \
    /* 11585/16384 ~= Sin[Pi/4] ~= 0.707106781186548 */ \
    OD_DCT_OVERFLOW_CHECK(tu, 11585, 8192, 278); \
    t1 -= (tu*11585 + 8192) >> 14; \
    /* -19195/32768 ~= Tan[Pi/8] - Tan[Pi/4] ~= -0.585786437626905 */ \
    OD_DCT_OVERFLOW_CHECK(t1, 19195, 16384, 279); \
    tu -= (t1*19195 + 16384) >> 15; \
    /* 28681/32768 ~= Tan[3*Pi/16] + Tan[Pi/8]/2 ~= 0.875285419105846 */ \
    OD_DCT_OVERFLOW_CHECK(t2, 28681, 16384, 280); \
    tt += (t2*28681 + 16384) >> 15; \
    /* 15137/16384 ~= Sin[3*Pi/8] ~= 0.923879532511287 */ \
    OD_DCT_OVERFLOW_CHECK(tt, 15137, 8192, 281); \
    t2 -= (tt*15137 + 8192) >> 14; \
    /* 4161/16384 ~= Tan[3*Pi/16] - Tan[Pi/8] ~= 0.253965075546204 */ \
    OD_DCT_OVERFLOW_CHECK(t2, 4161, 8192, 282); \
    tt += (t2*4161 + 8192) >> 14; \
    /* 4161/16384 ~= Tan[3*Pi/16] - Tan[Pi/8] ~= 0.253965075546204 */ \
    OD_DCT_OVERFLOW_CHECK(ts, 4161, 8192, 283); \
    t3 += (ts*4161 + 8192) >> 14; \
    /* 15137/16384 ~= Sin[3*Pi/8] ~= 0.923879532511287 */ \
    OD_DCT_OVERFLOW_CHECK(t3, 15137, 8192, 284); \
    ts -= (t3*15137 + 8192) >> 14; \
    /* 14341/16384 ~= Tan[3*Pi/16] + Tan[Pi/8]/2 ~= 0.875285419105846 */ \
    OD_DCT_OVERFLOW_CHECK(ts, 14341, 8192, 285); \
    t3 += (ts*14341 + 8192) >> 14; \
    /* -19195/32768 ~= Tan[Pi/8] - Tan[Pi/4] ~= -0.585786437626905 */ \
    OD_DCT_OVERFLOW_CHECK(tm, 19195, 16384, 286); \
    t9 -= (tm*19195 + 16384) >> 15; \
    /* 11585/16384 ~= Sin[Pi/4] ~= 0.707106781186548 */ \
    OD_DCT_OVERFLOW_CHECK(t9, 11585, 8192, 287); \
    tm -= (t9*11585 + 8192) >> 14; \
    /* 7489/8192 ~= Tan[Pi/8] + Tan[Pi/4]/2 ~= 0.914213562373095 */ \
    OD_DCT_OVERFLOW_CHECK(tm, 7489, 4096, 288); \
    t9 += (tm*7489 + 4096) >> 13; \
    /* 3259/8192 ~= 2*Tan[Pi/16] ~= 0.397824734759316 */ \
    OD_DCT_OVERFLOW_CHECK(tl, 3259, 4096, 289); \
    ta += (tl*3259 + 4096) >> 13; \
    /* 3135/16384 ~= Sin[Pi/8]/2 ~= 0.1913417161825449 */ \
    OD_DCT_OVERFLOW_CHECK(ta, 3135, 8192, 290); \
    tl -= (ta*3135 + 8192) >> 14; \
    /* 3259/8192 ~= 2*Tan[Pi/16] ~= 0.397824734759316 */ \
    OD_DCT_OVERFLOW_CHECK(tl, 3259, 4096, 291); \
    ta += (tl*3259 + 4096) >> 13; \
    /* 4161/16384 ~= Tan[3*Pi/16] - Tan[Pi/8] ~= 0.253965075546204 */ \
    OD_DCT_OVERFLOW_CHECK(tk, 4161, 8192, 292); \
    tb += (tk*4161 + 8192) >> 14; \
    /* 15137/16384 ~= Sin[3*Pi/8] ~= 0.923879532511287 */ \
    OD_DCT_OVERFLOW_CHECK(tb, 15137, 8192, 293); \
    tk -= (tb*15137 + 8192) >> 14; \
    /* 14341/16384 ~= Tan[3*Pi/16] + Tan[Pi/8]/2 ~= 0.875285419105846 */ \
    OD_DCT_OVERFLOW_CHECK(tk, 14341, 8192, 294); \
    tb += (tk*14341 + 8192) >> 14; \
    /* 29957/32768 ~= Tan[Pi/8] + Tan[Pi/4]/2 ~= 0.914213562373095 */ \
    OD_DCT_OVERFLOW_CHECK(te, 29957, 16384, 295); \
    th += (te*29957 + 16384) >> 15; \
    /* 11585/16384 ~= Sin[Pi/4] ~= 0.707106781186548 */ \
    OD_DCT_OVERFLOW_CHECK(th, 11585, 8192, 296); \
    te -= (th*11585 + 8192) >> 14; \
    /* -19195/32768 ~= Tan[Pi/8] - Tan[Pi/4] ~= -0.585786437626905 */ \
    OD_DCT_OVERFLOW_CHECK(te, 19195, 16384, 297); \
    th -= (te*19195 + 16384) >> 15; \
    /* 28681/32768 ~= Tan[3*Pi/16] + Tan[Pi/8]/2 ~= 0.875285419105846 */ \
    OD_DCT_OVERFLOW_CHECK(tc, 28681, 16384, 298); \
    tj += (tc*28681 + 16384) >> 15; \
    /* 15137/16384 ~= Sin[3*Pi/8] ~= 0.923879532511287 */ \
    OD_DCT_OVERFLOW_CHECK(tj, 15137, 8192, 299); \
    tc -= (tj*15137 + 8192) >> 14; \
    /* 4161/16384 ~= Tan[3*Pi/16] - Tan[Pi/8] ~= 0.253965075546204 */ \
    OD_DCT_OVERFLOW_CHECK(tc, 4161, 8192, 300); \
    tj += (tc*4161 + 8192) >> 14; \
    /* 4161/16384 ~= Tan[3*Pi/16] - Tan[Pi/8] ~= 0.253965075546204 */ \
    OD_DCT_OVERFLOW_CHECK(ti, 4161, 8192, 301); \
    td += (ti*4161 + 8192) >> 14; \
    /* 15137/16384 ~= Sin[3*Pi/8] ~= 0.923879532511287 */ \
    OD_DCT_OVERFLOW_CHECK(td, 15137, 8192, 302); \
    ti -= (td*15137 + 8192) >> 14; \
    /* 14341/16384 ~= Tan[3*Pi/16] + Tan[Pi/8]/2 ~= 0.875285419105846 */ \
    OD_DCT_OVERFLOW_CHECK(ti, 14341, 8192, 303); \
    td += (ti*14341 + 8192) >> 14; \
    \
    t1 = -t1; \
    t2 = -t2; \
    t3 = -t3; \
    td = -td; \
    tg = -tg; \
    to = -to; \
    ts = -ts; \
    \
    tr -= OD_RSHIFT1(t5); \
    t5 += tr; \
    tq -= OD_RSHIFT1(t4); /* pass */ \
    t4 += tq; \
    t6 -= OD_RSHIFT1(t7); \
    t7 += t6; \
    to -= OD_RSHIFT1(tp); /* pass */ \
    tp += to; \
    t1 += OD_RSHIFT1(t0); /* pass */ \
    t0 -= t1; \
    tv -= OD_RSHIFT1(tu); \
    tu += tv; \
    t3 -= OD_RSHIFT1(tt); \
    tt += t3; \
    t2 += OD_RSHIFT1(ts); \
    ts -= t2; \
    t9 -= OD_RSHIFT1(t8); /* pass */ \
    t8 += t9; \
    tn += OD_RSHIFT1(tm); \
    tm -= tn; \
    tb += OD_RSHIFT1(ta); \
    ta -= tb; \
    tl -= OD_RSHIFT1(tk); \
    tk += tl; \
    te -= OD_RSHIFT1(tf); /* pass */ \
    tf += te; \
    tg -= OD_RSHIFT1(th); \
    th += tg; \
    tc -= OD_RSHIFT1(ti); \
    ti += tc; \
    td += OD_RSHIFT1(tj); \
    tj -= td; \
    \
    t4 = -t4; \
    \
    /* 6723/8192 ~= Tan[7*Pi/32] ~= 0.8206787908286602 */ \
    OD_DCT_OVERFLOW_CHECK(tr, 6723, 4096, 304); \
    t4 += (tr*6723 + 4096) >> 13; \
    /* 16069/16384 ~= Sin[7*Pi/16] ~= 0.9807852804032304 */ \
    OD_DCT_OVERFLOW_CHECK(t4, 16069, 8192, 305); \
    tr -= (t4*16069 + 8192) >> 14; \
    /* 6723/8192 ~= Tan[7*Pi/32] ~= 0.8206787908286602 */ \
    OD_DCT_OVERFLOW_CHECK(tr, 6723, 4096, 306); \
    t4 += (tr*6723 + 4096) >> 13; \
    /* 17515/32768 ~= Tan[5*Pi/32] ~= 0.5345111359507916 */ \
    OD_DCT_OVERFLOW_CHECK(tq, 17515, 16384, 307); \
    t5 += (tq*17515 + 16384) >> 15; \
    /* 13623/16384 ~= Sin[5*Pi/16] ~= 0.8314696123025452 */ \
    OD_DCT_OVERFLOW_CHECK(t5, 13623, 8192, 308); \
    tq -= (t5*13623 + 8192) >> 14; \
    /* 17515/32768 ~= Tan[5*Pi/32] ~= 0.5345111359507916 */ \
    OD_DCT_OVERFLOW_CHECK(tq, 17515, 16384, 309); \
    t5 += (tq*17515 + 16384) >> 15; \
    /* 3227/32768 ~= Tan[Pi/32] ~= 0.09849140335716425 */ \
    OD_DCT_OVERFLOW_CHECK(to, 3227, 16384, 310); \
    t7 += (to*3227 + 16384) >> 15; \
    /* 6393/32768 ~= Sin[Pi/16] ~= 0.19509032201612825 */ \
    OD_DCT_OVERFLOW_CHECK(t7, 6393, 16384, 311); \
    to -= (t7*6393 + 16384) >> 15; \
    /* 3227/32768 ~= Tan[Pi/32] ~= 0.09849140335716425 */ \
    OD_DCT_OVERFLOW_CHECK(to, 3227, 16384, 312); \
    t7 += (to*3227 + 16384) >> 15; \
    /* 2485/8192 ~= Tan[3*Pi/32] ~= 0.303346683607342 */ \
    OD_DCT_OVERFLOW_CHECK(tp, 2485, 4096, 313); \
    t6 += (tp*2485 + 4096) >> 13; \
    /* 18205/32768 ~= Sin[3*Pi/16] ~= 0.555570233019602 */ \
    OD_DCT_OVERFLOW_CHECK(t6, 18205, 16384, 314); \
    tp -= (t6*18205 + 16384) >> 15; \
    /* 2485/8192 ~= Tan[3*Pi/32] ~= 0.303346683607342 */ \
    OD_DCT_OVERFLOW_CHECK(tp, 2485, 4096, 315); \
    t6 += (tp*2485 + 4096) >> 13; \
    \
    t5 = -t5; \
    \
    tr += to; \
    trh = OD_RSHIFT1(tr); \
    to -= trh; \
    t4 += t7; \
    t4h = OD_RSHIFT1(t4); \
    t7 -= t4h; \
    t5 += tp; \
    t5h = OD_RSHIFT1(t5); \
    tp -= t5h; \
    tq += t6; \
    tqh = OD_RSHIFT1(tq); \
    t6 -= tqh; \
    t0 -= t3; \
    t0h = OD_RSHIFT1(t0); \
    t3 += t0h; \
    tv -= ts; \
    tvh = OD_RSHIFT1(tv); \
    ts += tvh; \
    tu += tt; \
    tuh = OD_RSHIFT1(tu); \
    tt -= tuh; \
    t1 -= t2; \
    t1h = OD_RSHIFT1(t1); \
    t2 += t1h; \
    t8 += tb; \
    tb -= OD_RSHIFT1(t8); \
    tn += tk; \
    tk -= OD_RSHIFT1(tn); \
    t9 += tl; \
    tl -= OD_RSHIFT1(t9); \
    tm -= ta; \
    ta += OD_RSHIFT1(tm); \
    tc -= tf; \
    tf += OD_RSHIFT1(tc); \
    tj += tg; \
    tg -= OD_RSHIFT1(tj); \
    td -= te; \
    te += OD_RSHIFT1(td); \
    ti += th; \
    th -= OD_RSHIFT1(ti); \
    \
    t9 = -t9; \
    tl = -tl; \
    \
    /* 805/16384 ~= Tan[Pi/64] ~= 0.04912684976946793 */ \
    OD_DCT_OVERFLOW_CHECK(tn, 805, 8192, 316); \
    t8 += (tn*805 + 8192) >> 14; \
    /* 803/8192 ~= Sin[Pi/32] ~= 0.0980171403295606 */ \
    OD_DCT_OVERFLOW_CHECK(t8, 803, 4096, 317); \
    tn -= (t8*803 + 4096) >> 13; \
    /* 805/16384 ~= Tan[Pi/64] ~= 0.04912684976946793 */ \
    OD_DCT_OVERFLOW_CHECK(tn, 805, 8192, 318); \
    t8 += (tn*805 + 8192) >> 14; \
    /* 11725/32768 ~= Tan[7*Pi/64] ~= 0.3578057213145241 */ \
    OD_DCT_OVERFLOW_CHECK(tb, 11725, 16384, 319); \
    tk += (tb*11725 + 16384) >> 15; \
    /* 5197/8192 ~= Sin[7*Pi/32] ~= 0.6343932841636455 */ \
    OD_DCT_OVERFLOW_CHECK(tk, 5197, 4096, 320); \
    tb -= (tk*5197 + 4096) >> 13; \
    /* 11725/32768 ~= Tan[7*Pi/64] ~= 0.3578057213145241 */ \
    OD_DCT_OVERFLOW_CHECK(tb, 11725, 16384, 321); \
    tk += (tb*11725 + 16384) >> 15; \
    /* 2455/4096 ~= Tan[11*Pi/64] ~= 0.5993769336819237 */ \
    OD_DCT_OVERFLOW_CHECK(tl, 2455, 2048, 322); \
    ta += (tl*2455 + 2048) >> 12; \
    /* 14449/16384 ~= Sin[11*Pi/32] ~= 0.881921264348355 */ \
    OD_DCT_OVERFLOW_CHECK(ta, 14449, 8192, 323); \
    tl -= (ta*14449 + 8192) >> 14; \
    /* 2455/4096 ~= Tan[11*Pi/64] ~= 0.5993769336819237 */ \
    OD_DCT_OVERFLOW_CHECK(tl, 2455, 2048, 324); \
    ta += (tl*2455 + 2048) >> 12; \
    /* 4861/32768 ~= Tan[3*Pi/64] ~= 0.14833598753834742 */ \
    OD_DCT_OVERFLOW_CHECK(tm, 4861, 16384, 325); \
    t9 += (tm*4861 + 16384) >> 15; \
    /* 1189/4096 ~= Sin[3*Pi/32] ~= 0.29028467725446233 */ \
    OD_DCT_OVERFLOW_CHECK(t9, 1189, 2048, 326); \
    tm -= (t9*1189 + 2048) >> 12; \
    /* 4861/32768 ~= Tan[3*Pi/64] ~= 0.14833598753834742 */ \
    OD_DCT_OVERFLOW_CHECK(tm, 4861, 16384, 327); \
    t9 += (tm*4861 + 16384) >> 15; \
    /* 805/16384 ~= Tan[Pi/64] ~= 0.04912684976946793 */ \
    OD_DCT_OVERFLOW_CHECK(tg, 805, 8192, 328); \
    tf += (tg*805 + 8192) >> 14; \
    /* 803/8192 ~= Sin[Pi/32] ~= 0.0980171403295606 */ \
    OD_DCT_OVERFLOW_CHECK(tf, 803, 4096, 329); \
    tg -= (tf*803 + 4096) >> 13; \
    /* 805/16384 ~= Tan[Pi/64] ~= 0.04912684976946793 */ \
    OD_DCT_OVERFLOW_CHECK(tg, 805, 8192, 330); \
    tf += (tg*805 + 8192) >> 14; \
    /* 2931/8192 ~= Tan[7*Pi/64] ~= 0.3578057213145241 */ \
    OD_DCT_OVERFLOW_CHECK(tj, 2931, 4096, 331); \
    tc += (tj*2931 + 4096) >> 13; \
    /* 5197/8192 ~= Sin[7*Pi/32] ~= 0.6343932841636455 */ \
    OD_DCT_OVERFLOW_CHECK(tc, 5197, 4096, 332); \
    tj -= (tc*5197 + 4096) >> 13; \
    /* 2931/8192 ~= Tan[7*Pi/64] ~= 0.3578057213145241 */ \
    OD_DCT_OVERFLOW_CHECK(tj, 2931, 4096, 333); \
    tc += (tj*2931 + 4096) >> 13; \
    /* 513/2048 ~= Tan[5*Pi/64] ~= 0.25048696019130545 */ \
    OD_DCT_OVERFLOW_CHECK(ti, 513, 1024, 334); \
    td += (ti*513 + 1024) >> 11; \
    /* 7723/16384 ~= Sin[5*Pi/32] ~= 0.47139673682599764 */ \
    OD_DCT_OVERFLOW_CHECK(td, 7723, 8192, 335); \
    ti -= (td*7723 + 8192) >> 14; \
    /* 513/2048 ~= Tan[5*Pi/64] ~= 0.25048696019130545 */ \
    OD_DCT_OVERFLOW_CHECK(ti, 513, 1024, 336); \
    td += (ti*513 + 1024) >> 11; \
    /* 4861/32768 ~= Tan[3*Pi/64] ~= 0.14833598753834742 */ \
    OD_DCT_OVERFLOW_CHECK(th, 4861, 16384, 337); \
    te += (th*4861 + 16384) >> 15; \
    /* 1189/4096 ~= Sin[3*Pi/32] ~= 0.29028467725446233 */ \
    OD_DCT_OVERFLOW_CHECK(te, 1189, 2048, 338); \
    th -= (te*1189 + 2048) >> 12; \
    /* 4861/32768 ~= Tan[3*Pi/64] ~= 0.14833598753834742 */ \
    OD_DCT_OVERFLOW_CHECK(th, 4861, 16384, 339); \
    te += (th*4861 + 16384) >> 15; \
    \
    ta = -ta; \
    tb = -tb; \
    \
    tt += t5h; \
    t5 -= tt; \
    t2 -= tqh; \
    tq += t2; \
    tp += t1h; \
    t1 -= tp; \
    t6 -= tuh; \
    tu += t6; \
    t7 += tvh; \
    tv -= t7; \
    to += t0h; \
    t0 -= to; \
    t3 -= t4h; \
    t4 += t3; \
    ts += trh; \
    tr -= ts; \
    tf -= OD_RSHIFT1(tn); \
    tn += tf; \
    tg -= OD_RSHIFT1(t8); \
    t8 += tg; \
    tk += OD_RSHIFT1(tc); \
    tc -= tk; \
    tb += OD_RSHIFT1(tj); \
    tj -= tb; \
    ta += OD_RSHIFT1(ti); \
    ti -= ta; \
    tl += OD_RSHIFT1(td); \
    td -= tl; \
    te -= OD_RSHIFT1(tm); \
    tm += te; \
    th -= OD_RSHIFT1(t9); \
    t9 += th; \
    ta -= t5; \
    t5 += OD_RSHIFT1(ta); \
    tq -= tl; \
    tl += OD_RSHIFT1(tq); \
    t2 -= ti; \
    ti += OD_RSHIFT1(t2); \
    td -= tt; \
    tt += OD_RSHIFT1(td); \
    tm += tp; \
    tp -= OD_RSHIFT1(tm); \
    t6 += t9; \
    t9 -= OD_RSHIFT1(t6); \
    te -= tu; \
    tu += OD_RSHIFT1(te); \
    t1 -= th; \
    th += OD_RSHIFT1(t1); \
    t0 -= tg; \
    tg += OD_RSHIFT1(t0); \
    tf += tv; \
    tv -= OD_RSHIFT1(tf); \
    t8 -= t7; \
    t7 += OD_RSHIFT1(t8); \
    to -= tn; \
    tn += OD_RSHIFT1(to); \
    t4 -= tk; \
    tk += OD_RSHIFT1(t4); \
    tb -= tr; \
    tr += OD_RSHIFT1(tb); \
    t3 -= tj; \
    tj += OD_RSHIFT1(t3); \
    tc -= ts; \
    ts += OD_RSHIFT1(tc); \
    \
    tr = -tr; \
    ts = -ts; \
    tt = -tt; \
    tu = -tu; \
    \
    /* 2847/4096 ~= (1/Sqrt[2] - Cos[63*Pi/128]/2)/Sin[63*Pi/128] ~=
        0.6950455016354713 */ \
    OD_DCT_OVERFLOW_CHECK(t0, 2847, 2048, 340); \
    tv += (t0*2847 + 2048) >> 12; \
    /* 5791/4096 ~= Sqrt[2]*Sin[63*Pi/128] ~= 1.413787627688534 */ \
    OD_DCT_OVERFLOW_CHECK(tv, 5791, 2048, 341); \
    t0 -= (tv*5791 + 2048) >> 12; \
    /* 5593/8192 ~= (1/Sqrt[2] - Cos[63*Pi/128])/Sin[63*Pi/128] ~=
        0.6827711905810085 */ \
    OD_DCT_OVERFLOW_CHECK(t0, 5593, 4096, 342); \
    tv += (t0*5593 + 4096) >> 13; \
    /* 4099/8192 ~= (1/Sqrt[2] - Cos[31*Pi/128]/2)/Sin[31*Pi/128] ~=
        0.5003088539809675 */ \
    OD_DCT_OVERFLOW_CHECK(tf, 4099, 4096, 343); \
    tg -= (tf*4099 + 4096) >> 13; \
    /* 1997/2048 ~= Sqrt[2]*Sin[31*Pi/128] ~= 0.9751575901732918 */ \
    OD_DCT_OVERFLOW_CHECK(tg, 1997, 1024, 344); \
    tf += (tg*1997 + 1024) >> 11; \
    /* -815/32768 ~= (1/Sqrt[2] - Cos[31*Pi/128])/Sin[31*Pi/128] ~=
        -0.02485756913896231 */ \
    OD_DCT_OVERFLOW_CHECK(tf, 815, 16384, 345); \
    tg += (tf*815 + 16384) >> 15; \
    /* 2527/4096 ~= (1/Sqrt[2] - Cos[17*Pi/128]/2)/Sin[17*Pi/128] ~=
        0.6169210657818165 */ \
    OD_DCT_OVERFLOW_CHECK(t8, 2527, 2048, 346); \
    tn -= (t8*2527 + 2048) >> 12; \
    /* 4695/8192 ~= Sqrt[2]*Sin[17*Pi/128] ~= 0.5730977622997507 */ \
    OD_DCT_OVERFLOW_CHECK(tn, 4695, 4096, 347); \
    t8 += (tn*4695 + 4096) >> 13; \
    /* -4187/8192 ~= (1/Sqrt[2] - Cos[17*Pi/128])/Sin[17*Pi/128] ~=
        -0.5110608601827629 */ \
    OD_DCT_OVERFLOW_CHECK(t8, 4187, 4096, 348); \
    tn += (t8*4187 + 4096) >> 13; \
    /* 5477/8192 ~= (1/Sqrt[2] - Cos[15*Pi/128]/2)/Sin[15*Pi/128] ~=
        0.6685570995525147 */ \
    OD_DCT_OVERFLOW_CHECK(to, 5477, 4096, 349); \
    t7 += (to*5477 + 4096) >> 13; \
    /* 4169/8192 ~= Sqrt[2]*Sin[15*Pi/128] ~= 0.5089684416985407 */ \
    OD_DCT_OVERFLOW_CHECK(t7, 4169, 4096, 350); \
    to -= (t7*4169 + 4096) >> 13; \
    /* -2571/4096 ~= (1/Sqrt[2] - Cos[15*Pi/128])/Sin[15*Pi/128] ~=
        -0.6276441593165217 */ \
    OD_DCT_OVERFLOW_CHECK(to, 2571, 2048, 351); \
    t7 -= (to*2571 + 2048) >> 12; \
    /* 5331/8192 ~= (1/Sqrt[2] - Cos[59*Pi/128]/2)/Sin[59*Pi/128] ~=
        0.6507957303604222 */ \
    OD_DCT_OVERFLOW_CHECK(t2, 5331, 4096, 352); \
    tt += (t2*5331 + 4096) >> 13; \
    /* 5749/4096 ~= Sqrt[2]*Sin[59*Pi/128] ~= 1.4035780182072333 */ \
    OD_DCT_OVERFLOW_CHECK(tt, 5749, 2048, 353); \
    t2 -= (tt*5749 + 2048) >> 12; \
    /* 2413/4096 ~= (1/Sqrt[2] - Cos[59*Pi/128])/Sin[59*Pi/128] ~=
        0.5891266122920528 */ \
    OD_DCT_OVERFLOW_CHECK(t2, 2413, 2048, 354); \
    tt += (t2*2413 + 2048) >> 12; \
    /* 4167/8192 ~= (1/Sqrt[2] - Cos[27*Pi/128]/2)/Sin[27*Pi/128] ~=
        0.5086435289805458 */ \
    OD_DCT_OVERFLOW_CHECK(td, 4167, 4096, 355); \
    ti -= (td*4167 + 4096) >> 13; \
    /* 891/1024 ~= Sqrt[2]*Sin[27*Pi/128] ~= 0.8700688593994939 */ \
    OD_DCT_OVERFLOW_CHECK(ti, 891, 512, 356); \
    td += (ti*891 + 512) >> 10; \
    /* -4327/32768 ~= (1/Sqrt[2] - Cos[27*Pi/128])/Sin[27*Pi/128] ~=
        -0.13204726103773165 */ \
    OD_DCT_OVERFLOW_CHECK(td, 4327, 16384, 357); \
    ti += (td*4327 + 16384) >> 15; \
    /* 2261/4096 ~= (1/Sqrt[2] - Cos[21*Pi/128]/2)/Sin[21*Pi/128] ~=
        0.5519664910950994 */ \
    OD_DCT_OVERFLOW_CHECK(ta, 2261, 2048, 358); \
    tl -= (ta*2261 + 2048) >> 12; \
    /* 2855/4096 ~= Sqrt[2]*Sin[21*Pi/128] ~= 0.6970633083205415 */ \
    OD_DCT_OVERFLOW_CHECK(tl, 2855, 2048, 359); \
    ta += (tl*2855 + 2048) >> 12; \
    /* -5417/16384 ~= (1/Sqrt[2] - Cos[21*Pi/128])/Sin[21*Pi/128] ~=
        -0.3306569439519963 */ \
    OD_DCT_OVERFLOW_CHECK(ta, 5417, 8192, 360); \
    tl += (ta*5417 + 8192) >> 14; \
    /* 3459/4096 ~= (1/Sqrt[2] - Cos[11*Pi/128]/2)/Sin[11*Pi/128] ~=
        0.8444243553292501 */ \
    OD_DCT_OVERFLOW_CHECK(tq, 3459, 2048, 361); \
    t5 += (tq*3459 + 2048) >> 12; \
    /* 1545/4096 ~= Sqrt[2]*Sin[11*Pi/128] ~= 0.37718879887892737 */ \
    OD_DCT_OVERFLOW_CHECK(t5, 1545, 2048, 362); \
    tq -= (t5*1545 + 2048) >> 12; \
    /* -1971/2048 ~= (1/Sqrt[2] - Cos[11*Pi/128])/Sin[11*Pi/128] ~=
        -0.9623434853244648 */ \
    OD_DCT_OVERFLOW_CHECK(tq, 1971, 1024, 363); \
    t5 -= (tq*1971 + 1024) >> 11; \
    /* 323/512 ~= (1/Sqrt[2] - Cos[57*Pi/128]/2)/Sin[57*Pi/128] ~=
        0.6309143839894504 */ \
    OD_DCT_OVERFLOW_CHECK(t3, 323, 256, 364); \
    ts += (t3*323 + 256) >> 9; \
    /* 5707/4096 ~= Sqrt[2]*Sin[57*Pi/128] ~= 1.3933930045694292 */ \
    OD_DCT_OVERFLOW_CHECK(ts, 5707, 2048, 365); \
    t3 -= (ts*5707 + 2048) >> 12; \
    /* 2229/4096 ~= (1/Sqrt[2] - Cos[57*Pi/128])/Sin[57*Pi/128] ~=
        0.5441561539205226 */ \
    OD_DCT_OVERFLOW_CHECK(t3, 2229, 2048, 366); \
    ts += (t3*2229 + 2048) >> 12; \
    /* 1061/2048 ~= (1/Sqrt[2] - Cos[25*Pi/128]/2)/Sin[25*Pi/128] ~=
        0.5180794213368158 */ \
    OD_DCT_OVERFLOW_CHECK(tc, 1061, 1024, 367); \
    tj -= (tc*1061 + 1024) >> 11; \
    /* 6671/8192 ~= Sqrt[2]*Sin[25*Pi/128] ~= 0.8143157536286402 */ \
    OD_DCT_OVERFLOW_CHECK(tj, 6671, 4096, 368); \
    tc += (tj*6671 + 4096) >> 13; \
    /* -6287/32768 ~= (1/Sqrt[2] - Cos[25*Pi/128])/Sin[25*Pi/128] ~=
        -0.19186603041023065 */ \
    OD_DCT_OVERFLOW_CHECK(tc, 6287, 16384, 369); \
    tj += (tc*6287 + 16384) >> 15; \
    /* 4359/8192 ~= (1/Sqrt[2] - Cos[23*Pi/128]/2)/Sin[23*Pi/128] ~=
        0.5321145141202145 */ \
    OD_DCT_OVERFLOW_CHECK(tb, 4359, 4096, 370); \
    tk -= (tb*4359 + 4096) >> 13; \
    /* 3099/4096 ~= Sqrt[2]*Sin[23*Pi/128] ~= 0.7566008898816587 */ \
    OD_DCT_OVERFLOW_CHECK(tk, 3099, 2048, 371); \
    tb += (tk*3099 + 2048) >> 12; \
    /* -2109/8192 ~= (1/Sqrt[2] - Cos[23*Pi/128])/Sin[23*Pi/128] ~=
        -0.2574717698598901 */ \
    OD_DCT_OVERFLOW_CHECK(tb, 2109, 4096, 372); \
    tk += (tb*2109 + 4096) >> 13; \
    /* 5017/8192 ~= (1/Sqrt[2] - Cos[55*Pi/128]/2)/Sin[55*Pi/128] ~=
        0.6124370775787037 */ \
    OD_DCT_OVERFLOW_CHECK(t4, 5017, 4096, 373); \
    tr += (t4*5017 + 4096) >> 13; \
    /* 1413/1024 ~= Sqrt[2]*Sin[55*Pi/128] ~= 1.3798511851368045 */ \
    OD_DCT_OVERFLOW_CHECK(tr, 1413, 512, 374); \
    t4 -= (tr*1413 + 512) >> 10; \
    /* 8195/16384 ~= (1/Sqrt[2] - Cos[55*Pi/128])/Sin[55*Pi/128] ~=
        0.5001583229201391 */ \
    OD_DCT_OVERFLOW_CHECK(t4, 8195, 8192, 375); \
    tr += (t4*8195 + 8192) >> 14; \
    /* 2373/4096 ~= (1/Sqrt[2] - Cos[19*Pi/128]/2)/Sin[19*Pi/128] ~=
        0.5793773719823809 */ \
    OD_DCT_OVERFLOW_CHECK(tm, 2373, 2048, 376); \
    t9 += (tm*2373 + 2048) >> 12; \
    /* 5209/8192 ~= Sqrt[2]*Sin[19*Pi/128] ~= 0.6358464401941452 */ \
    OD_DCT_OVERFLOW_CHECK(t9, 5209, 4096, 377); \
    tm -= (t9*5209 + 4096) >> 13; \
    /* -3391/8192 ~= (1/Sqrt[2] - Cos[19*Pi/128])/Sin[19*Pi/128] ~=
        -0.41395202418930155 */ \
    OD_DCT_OVERFLOW_CHECK(tm, 3391, 4096, 378); \
    t9 -= (tm*3391 + 4096) >> 13; \
    /* 1517/2048 ~= (1/Sqrt[2] - Cos[13*Pi/128]/2)/Sin[13*Pi/128] ~=
        0.7406956190518837 */ \
    OD_DCT_OVERFLOW_CHECK(t6, 1517, 1024, 379); \
    tp -= (t6*1517 + 1024) >> 11; \
    /* 1817/4096 ~= Sqrt[2]*Sin[13*Pi/128] ~= 0.4436129715409088 */ \
    OD_DCT_OVERFLOW_CHECK(tp, 1817, 2048, 380); \
    t6 += (tp*1817 + 2048) >> 12; \
    /* -6331/8192 ~= (1/Sqrt[2] - Cos[13*Pi/128])/Sin[13*Pi/128] ~=
        -0.772825983107003 */ \
    OD_DCT_OVERFLOW_CHECK(t6, 6331, 4096, 381); \
    tp += (t6*6331 + 4096) >> 13; \
    /* 515/1024 ~= (1/Sqrt[2] - Cos[29*Pi/128]/2)/Sin[29*Pi/128] ~=
        0.5029332763556925 */ \
    OD_DCT_OVERFLOW_CHECK(te, 515, 512, 382); \
    th -= (te*515 + 512) >> 10; \
    /* 7567/8192 ~= Sqrt[2]*Sin[29*Pi/128] ~= 0.9237258930790229 */ \
    OD_DCT_OVERFLOW_CHECK(th, 7567, 4096, 383); \
    te += (th*7567 + 4096) >> 13; \
    /* -2513/32768 ~= (1/Sqrt[2] - Cos[29*Pi/128])/Sin[29*Pi/128] ~=
        -0.07670567731102484 */ \
    OD_DCT_OVERFLOW_CHECK(te, 2513, 16384, 384); \
    th += (te*2513 + 16384) >> 15; \
    /* 2753/4096 ~= (1/Sqrt[2] - Cos[61*Pi/128]/2)/Sin[61*Pi/128] ~=
        0.6721457072988726 */ \
    OD_DCT_OVERFLOW_CHECK(t1, 2753, 2048, 385); \
    tu += (t1*2753 + 2048) >> 12; \
    /* 5777/4096 ~= Sqrt[2]*Sin[61*Pi/128] ~= 1.4103816894602614 */ \
    OD_DCT_OVERFLOW_CHECK(tu, 5777, 2048, 386); \
    t1 -= (tu*5777 + 2048) >> 12; \
    /* 1301/2048 ~= (1/Sqrt[2] - Cos[61*Pi/128])/Sin[61*Pi/128] ~=
        0.6352634915376478 */ \
    OD_DCT_OVERFLOW_CHECK(t1, 1301, 1024, 387); \
    tu += (t1*1301 + 1024) >> 11; \
} while (0)

/* Embedded 32-point asymmetric Type-IV iDST. */
#define OD_IDST_32_ASYM(t0, tg, t8, to, t4, tk, tc, ts, t2, ti, ta, tq, t6, \
                        tm, te, tu, t1, th, t9, tp, t5, tl, td, tt, t3, tj, tb, tr, \
                        t7, tn, tf, tv) \
do { \
    dctcoef t0h; \
    dctcoef t4h; \
    dctcoef tbh; \
    dctcoef tfh; \
    dctcoef tgh; \
    dctcoef tkh; \
    dctcoef trh; \
    dctcoef tvh; \
    /* 1301/2048 ~= (1/Sqrt[2] - Cos[61*Pi/128])/Sin[61*Pi/128] ~=
        0.6352634915376478 */ \
    tf -= (tg*1301 + 1024) >> 11; \
    /* 5777/4096 ~= Sqrt[2]*Sin[61*Pi/128] ~= 1.4103816894602614 */ \
    tg += (tf*5777 + 2048) >> 12; \
    /* 2753/4096 ~= (1/Sqrt[2] - Cos[61*Pi/128]/2)/Sin[61*Pi/128] ~=
        0.6721457072988726 */ \
    tf -= (tg*2753 + 2048) >> 12; \
    /* -2513/32768 ~= (1/Sqrt[2] - Cos[29*Pi/128])/Sin[29*Pi/128] ~=
        -0.07670567731102484 */ \
    th -= (te*2513 + 16384) >> 15; \
    /* 7567/8192 ~= Sqrt[2]*Sin[29*Pi/128] ~= 0.9237258930790229 */ \
    te -= (th*7567 + 4096) >> 13; \
    /* 515/1024 ~= (1/Sqrt[2] - Cos[29*Pi/128]/2)/Sin[29*Pi/128] ~=
        0.5029332763556925 */ \
    th += (te*515 + 512) >> 10; \
    /* -6331/8192 ~= (1/Sqrt[2] - Cos[13*Pi/128])/Sin[13*Pi/128] ~=
        -0.772825983107003 */ \
    tj -= (tc*6331 + 4096) >> 13; \
    /* 1817/4096 ~= Sqrt[2]*Sin[13*Pi/128] ~= 0.4436129715409088 */ \
    tc -= (tj*1817 + 2048) >> 12; \
    /* 1517/2048 ~= (1/Sqrt[2] - Cos[13*Pi/128]/2)/Sin[13*Pi/128] ~=
        0.7406956190518837 */ \
    tj += (tc*1517 + 1024) >> 11; \
    /* -3391/8192 ~= (1/Sqrt[2] - Cos[19*Pi/128])/Sin[19*Pi/128] ~=
        -0.41395202418930155 */ \
    ti += (td*3391 + 4096) >> 13; \
    /* 5209/8192 ~= Sqrt[2]*Sin[19*Pi/128] ~= 0.6358464401941452 */ \
    td += (ti*5209 + 4096) >> 13; \
    /* 2373/4096 ~= (1/Sqrt[2] - Cos[19*Pi/128]/2)/Sin[19*Pi/128] ~=
        0.5793773719823809 */ \
    ti -= (td*2373 + 2048) >> 12; \
    /* 8195/16384 ~= (1/Sqrt[2] - Cos[55*Pi/128])/Sin[55*Pi/128] ~=
        0.5001583229201391 */ \
    tr -= (t4*8195 + 8192) >> 14; \
    /* 1413/1024 ~= Sqrt[2]*Sin[55*Pi/128] ~= 1.3798511851368045 */ \
    t4 += (tr*1413 + 512) >> 10; \
    /* 5017/8192 ~= (1/Sqrt[2] - Cos[55*Pi/128]/2)/Sin[55*Pi/128] ~=
        0.6124370775787037 */ \
    tr -= (t4*5017 + 4096) >> 13; \
    /* -2109/8192 ~= (1/Sqrt[2] - Cos[23*Pi/128])/Sin[23*Pi/128] ~=
        -0.2574717698598901 */ \
    t5 -= (tq*2109 + 4096) >> 13; \
    /* 3099/4096 ~= Sqrt[2]*Sin[23*Pi/128] ~= 0.7566008898816587 */ \
    tq -= (t5*3099 + 2048) >> 12; \
    /* 4359/8192 ~= (1/Sqrt[2] - Cos[23*Pi/128]/2)/Sin[23*Pi/128] ~=
        0.5321145141202145 */ \
    t5 += (tq*4359 + 4096) >> 13; \
    /* -6287/32768 ~= (1/Sqrt[2] - Cos[25*Pi/128])/Sin[25*Pi/128] ~=
        -0.19186603041023065 */ \
    tp -= (t6*6287 + 16384) >> 15; \
    /* 6671/8192 ~= Sqrt[2]*Sin[25*Pi/128] ~= 0.8143157536286402 */ \
    t6 -= (tp*6671 + 4096) >> 13; \
    /* 1061/2048 ~= (1/Sqrt[2] - Cos[25*Pi/128]/2)/Sin[25*Pi/128] ~=
        0.5180794213368158 */ \
    tp += (t6*1061 + 1024) >> 11; \
    /* 2229/4096 ~= (1/Sqrt[2] - Cos[57*Pi/128])/Sin[57*Pi/128] ~=
        0.5441561539205226 */ \
    t7 -= (to*2229 + 2048) >> 12; \
    /* 5707/4096 ~= Sqrt[2]*Sin[57*Pi/128] ~= 1.3933930045694292 */ \
    to += (t7*5707 + 2048) >> 12; \
    /* 323/512 ~= (1/Sqrt[2] - Cos[57*Pi/128]/2)/Sin[57*Pi/128] ~=
        0.6309143839894504 */ \
    t7 -= (to*323 + 256) >> 9; \
    /* -1971/2048 ~= (1/Sqrt[2] - Cos[11*Pi/128])/Sin[11*Pi/128] ~=
        -0.9623434853244648 */ \
    tk += (tb*1971 + 1024) >> 11; \
    /* 1545/4096 ~= Sqrt[2]*Sin[11*Pi/128] ~= 0.37718879887892737 */ \
    tb += (tk*1545 + 2048) >> 12; \
    /* 3459/4096 ~= (1/Sqrt[2] - Cos[11*Pi/128]/2)/Sin[11*Pi/128] ~=
        0.8444243553292501 */ \
    tk -= (tb*3459 + 2048) >> 12; \
    /* -5417/16384 ~= (1/Sqrt[2] - Cos[21*Pi/128])/Sin[21*Pi/128] ~=
        -0.3306569439519963 */ \
    tl -= (ta*5417 + 8192) >> 14; \
    /* 2855/4096 ~= Sqrt[2]*Sin[21*Pi/128] ~= 0.6970633083205415 */ \
    ta -= (tl*2855 + 2048) >> 12; \
    /* 2261/4096 ~= (1/Sqrt[2] - Cos[21*Pi/128]/2)/Sin[21*Pi/128] ~=
        0.5519664910950994 */ \
    tl += (ta*2261 + 2048) >> 12; \
    /* -4327/32768 ~= (1/Sqrt[2] - Cos[27*Pi/128])/Sin[27*Pi/128] ~=
        -0.13204726103773165 */ \
    t9 -= (tm*4327 + 16384) >> 15; \
    /* 891/1024 ~= Sqrt[2]*Sin[27*Pi/128] ~= 0.8700688593994939 */ \
    tm -= (t9*891 + 512) >> 10; \
    /* 4167/8192 ~= (1/Sqrt[2] - Cos[27*Pi/128]/2)/Sin[27*Pi/128] ~=
        0.5086435289805458 */ \
    t9 += (tm*4167 + 4096) >> 13; \
    /* 2413/4096 ~= (1/Sqrt[2] - Cos[59*Pi/128])/Sin[59*Pi/128] ~=
        0.5891266122920528 */ \
    tn -= (t8*2413 + 2048) >> 12; \
    /* 5749/4096 ~= Sqrt[2]*Sin[59*Pi/128] ~= 1.4035780182072333 */ \
    t8 += (tn*5749 + 2048) >> 12; \
    /* 5331/8192 ~= (1/Sqrt[2] - Cos[59*Pi/128]/2)/Sin[59*Pi/128] ~=
        0.6507957303604222 */ \
    tn -= (t8*5331 + 4096) >> 13; \
    /* -2571/4096 ~= (1/Sqrt[2] - Cos[15*Pi/128])/Sin[15*Pi/128] ~=
        -0.6276441593165217 */ \
    ts += (t3*2571 + 2048) >> 12; \
    /* 4169/8192 ~= Sqrt[2]*Sin[15*Pi/128] ~= 0.5089684416985407 */ \
    t3 += (ts*4169 + 4096) >> 13; \
    /* 5477/8192 ~= (1/Sqrt[2] - Cos[15*Pi/128]/2)/Sin[15*Pi/128] ~=
        0.6685570995525147 */ \
    ts -= (t3*5477 + 4096) >> 13; \
    /* -4187/8192 ~= (1/Sqrt[2] - Cos[17*Pi/128])/Sin[17*Pi/128] ~=
        -0.5110608601827629 */ \
    tt -= (t2*4187 + 4096) >> 13; \
    /* 4695/8192 ~= Sqrt[2]*Sin[17*Pi/128] ~= 0.5730977622997507 */ \
    t2 -= (tt*4695 + 4096) >> 13; \
    /* 2527/4096 ~= (1/Sqrt[2] - Cos[17*Pi/128]/2)/Sin[17*Pi/128] ~=
        0.6169210657818165 */ \
    tt += (t2*2527 + 2048) >> 12; \
    /* -815/32768 ~= (1/Sqrt[2] - Cos[31*Pi/128])/Sin[31*Pi/128] ~=
        -0.02485756913896231 */ \
    t1 -= (tu*815 + 16384) >> 15; \
    /* 1997/2048 ~= Sqrt[2]*Sin[31*Pi/128] ~= 0.9751575901732918 */ \
    tu -= (t1*1997 + 1024) >> 11; \
    /* 4099/8192 ~= (1/Sqrt[2] - Cos[31*Pi/128]/2)/Sin[31*Pi/128] ~=
        0.5003088539809675 */ \
    t1 += (tu*4099 + 4096) >> 13; \
    /* 5593/8192 ~= (1/Sqrt[2] - Cos[63*Pi/128])/Sin[63*Pi/128] ~=
        0.6827711905810085 */ \
    tv -= (t0*5593 + 4096) >> 13; \
    /* 5791/4096 ~= Sqrt[2]*Sin[63*Pi/128] ~= 1.413787627688534 */ \
    t0 += (tv*5791 + 2048) >> 12; \
    /* 2847/4096 ~= (1/Sqrt[2] - Cos[63*Pi/128]/2)/Sin[63*Pi/128] ~=
        0.6950455016354713 */ \
    tv -= (t0*2847 + 2048) >> 12; \
    \
    t7 = -t7; \
    tf = -tf; \
    tn = -tn; \
    tr = -tr; \
    \
    t7 -= OD_RSHIFT1(t6); \
    t6 += t7; \
    tp -= OD_RSHIFT1(to); \
    to += tp; \
    tr -= OD_RSHIFT1(tq); \
    tq += tr; \
    t5 -= OD_RSHIFT1(t4); \
    t4 += t5; \
    tt -= OD_RSHIFT1(t3); \
    t3 += tt; \
    ts -= OD_RSHIFT1(t2); \
    t2 += ts; \
    tv += OD_RSHIFT1(tu); \
    tu -= tv; \
    t1 -= OD_RSHIFT1(t0); \
    t0 += t1; \
    th -= OD_RSHIFT1(tg); \
    tg += th; \
    tf -= OD_RSHIFT1(te); \
    te += tf; \
    ti += OD_RSHIFT1(tc); \
    tc -= ti; \
    tj += OD_RSHIFT1(td); \
    td -= tj; \
    tn -= OD_RSHIFT1(tm); \
    tm += tn; \
    t9 -= OD_RSHIFT1(t8); \
    t8 += t9; \
    tl -= OD_RSHIFT1(tb); \
    tb += tl; \
    tk -= OD_RSHIFT1(ta); \
    ta += tk; \
    \
    ti -= th; \
    th += OD_RSHIFT1(ti); \
    td -= te; \
    te += OD_RSHIFT1(td); \
    tm += tl; \
    tl -= OD_RSHIFT1(tm); \
    t9 += ta; \
    ta -= OD_RSHIFT1(t9); \
    tp += tq; \
    tq -= OD_RSHIFT1(tp); \
    t6 += t5; \
    t5 -= OD_RSHIFT1(t6); \
    t2 -= t1; \
    t1 += OD_RSHIFT1(t2); \
    tt -= tu; \
    tu += OD_RSHIFT1(tt); \
    tr += t7; \
    trh = OD_RSHIFT1(tr); \
    t7 -= trh; \
    t4 -= to; \
    t4h = OD_RSHIFT1(t4); \
    to += t4h; \
    t0 += t3; \
    t0h = OD_RSHIFT1(t0); \
    t3 -= t0h; \
    tv += ts; \
    tvh = OD_RSHIFT1(tv); \
    ts -= tvh; \
    tf -= tc; \
    tfh = OD_RSHIFT1(tf); \
    tc += tfh; \
    tg += tj; \
    tgh = OD_RSHIFT1(tg); \
    tj -= tgh; \
    tb -= t8; \
    tbh = OD_RSHIFT1(tb); \
    t8 += tbh; \
    tk += tn; \
    tkh = OD_RSHIFT1(tk); \
    tn -= tkh; \
    \
    ta = -ta; \
    tq = -tq; \
    \
    /* 4861/32768 ~= Tan[3*Pi/64] ~= 0.14833598753834742 */ \
    te -= (th*4861 + 16384) >> 15; \
    /* 1189/4096 ~= Sin[3*Pi/32] ~= 0.29028467725446233 */ \
    th += (te*1189 + 2048) >> 12; \
    /* 4861/32768 ~= Tan[3*Pi/64] ~= 0.14833598753834742 */ \
    te -= (th*4861 + 16384) >> 15; \
    /* 513/2048 ~= Tan[5*Pi/64] ~= 0.25048696019130545 */ \
    tm -= (t9*513 + 1024) >> 11; \
    /* 7723/16384 ~= Sin[5*Pi/32] ~= 0.47139673682599764 */ \
    t9 += (tm*7723 + 8192) >> 14; \
    /* 513/2048 ~= Tan[5*Pi/64] ~= 0.25048696019130545 */ \
    tm -= (t9*513 + 1024) >> 11; \
    /* 2931/8192 ~= Tan[7*Pi/64] ~= 0.3578057213145241 */ \
    t6 -= (tp*2931 + 4096) >> 13; \
    /* 5197/8192 ~= Sin[7*Pi/32] ~= 0.6343932841636455 */ \
    tp += (t6*5197 + 4096) >> 13; \
    /* 2931/8192 ~= Tan[7*Pi/64] ~= 0.3578057213145241 */ \
    t6 -= (tp*2931 + 4096) >> 13; \
    /* 805/16384 ~= Tan[Pi/64] ~= 0.04912684976946793 */ \
    tu -= (t1*805 + 8192) >> 14; \
    /* 803/8192 ~= Sin[Pi/32] ~= 0.0980171403295606 */ \
    t1 += (tu*803 + 4096) >> 13; \
    /* 805/16384 ~= Tan[Pi/64] ~= 0.04912684976946793 */ \
    tu -= (t1*805 + 8192) >> 14; \
    /* 4861/32768 ~= Tan[3*Pi/64] ~= 0.14833598753834742 */ \
    ti -= (td*4861 + 16384) >> 15; \
    /* 1189/4096 ~= Sin[3*Pi/32] ~= 0.29028467725446233 */ \
    td += (ti*1189 + 2048) >> 12; \
    /* 4861/32768 ~= Tan[3*Pi/64] ~= 0.14833598753834742 */ \
    ti -= (td*4861 + 16384) >> 15; \
    /* 2455/4096 ~= Tan[11*Pi/64] ~= 0.5993769336819237 */ \
    ta -= (tl*2455 + 2048) >> 12; \
    /* 14449/16384 ~= Sin[11*Pi/32] ~= 0.881921264348355 */ \
    tl += (ta*14449 + 8192) >> 14; \
    /* 2455/4096 ~= Tan[11*Pi/64] ~= 0.5993769336819237 */ \
    ta -= (tl*2455 + 2048) >> 12; \
    /* 11725/32768 ~= Tan[7*Pi/64] ~= 0.3578057213145241 */ \
    t5 -= (tq*11725 + 16384) >> 15; \
    /* 5197/8192 ~= Sin[7*Pi/32] ~= 0.6343932841636455 */ \
    tq += (t5*5197 + 4096) >> 13; \
    /* 11725/32768 ~= Tan[7*Pi/64] ~= 0.3578057213145241 */ \
    t5 -= (tq*11725 + 16384) >> 15; \
    /* 805/16384 ~= Tan[Pi/64] ~= 0.04912684976946793 */ \
    t2 -= (tt*805 + 8192) >> 14; \
    /* 803/8192 ~= Sin[Pi/32] ~= 0.0980171403295606 */ \
    tt += (t2*803 + 4096) >> 13; \
    /* 805/16384 ~= Tan[Pi/64] ~= 0.04912684976946793 */ \
    t2 -= (tt*805 + 8192) >> 14; \
    \
    tl = -tl; \
    ti = -ti; \
    \
    th += OD_RSHIFT1(t9); \
    t9 -= th; \
    te -= OD_RSHIFT1(tm); \
    tm += te; \
    t1 += OD_RSHIFT1(tp); \
    tp -= t1; \
    tu -= OD_RSHIFT1(t6); \
    t6 += tu; \
    ta -= OD_RSHIFT1(td); \
    td += ta; \
    tl += OD_RSHIFT1(ti); \
    ti -= tl; \
    t5 += OD_RSHIFT1(tt); \
    tt -= t5; \
    tq += OD_RSHIFT1(t2); \
    t2 -= tq; \
    \
    t8 -= tgh; \
    tg += t8; \
    tn += tfh; \
    tf -= tn; \
    t7 -= tvh; \
    tv += t7; \
    to -= t0h; \
    t0 += to; \
    tc += tbh; \
    tb -= tc; \
    tj += tkh; \
    tk -= tj; \
    ts += t4h; \
    t4 -= ts; \
    t3 += trh; \
    tr -= t3; \
    \
    tk = -tk; \
    \
    /* 2485/8192 ~= Tan[3*Pi/32] ~= 0.303346683607342 */ \
    tc -= (tj*2485 + 4096) >> 13; \
    /* 18205/32768 ~= Sin[3*Pi/16] ~= 0.555570233019602 */ \
    tj += (tc*18205 + 16384) >> 15; \
    /* 2485/8192 ~= Tan[3*Pi/32] ~= 0.303346683607342 */ \
    tc -= (tj*2485 + 4096) >> 13; \
    /* 3227/32768 ~= Tan[Pi/32] ~= 0.09849140335716425 */ \
    ts -= (t3*3227 + 16384) >> 15; \
    /* 6393/32768 ~= Sin[Pi/16] ~= 0.19509032201612825 */ \
    t3 += (ts*6393 + 16384) >> 15; \
    /* 3227/32768 ~= Tan[Pi/32] ~= 0.09849140335716425 */ \
    ts -= (t3*3227 + 16384) >> 15; \
    /* 17515/32768 ~= Tan[5*Pi/32] ~= 0.5345111359507916 */ \
    tk -= (tb*17515 + 16384) >> 15; \
    /* 13623/16384 ~= Sin[5*Pi/16] ~= 0.8314696123025452 */ \
    tb += (tk*13623 + 8192) >> 14; \
    /* 17515/32768 ~= Tan[5*Pi/32] ~= 0.5345111359507916 */ \
    tk -= (tb*17515 + 16384) >> 15; \
    /* 6723/8192 ~= Tan[7*Pi/32] ~= 0.8206787908286602 */ \
    t4 -= (tr*6723 + 4096) >> 13; \
    /* 16069/16384 ~= Sin[7*Pi/16] ~= 0.9807852804032304 */ \
    tr += (t4*16069 + 8192) >> 14; \
    /* 6723/8192 ~= Tan[7*Pi/32] ~= 0.8206787908286602 */ \
    t4 -= (tr*6723 + 4096) >> 13; \
    \
    t4 = -t4; \
    \
    tp += tm; \
    tm -= OD_RSHIFT1(tp); \
    t9 -= t6; \
    t6 += OD_RSHIFT1(t9); \
    th -= t1; \
    t1 += OD_RSHIFT1(th); \
    tu -= te; \
    te += OD_RSHIFT1(tu); /* pass */ \
    t5 -= tl; \
    tl += OD_RSHIFT1(t5); \
    ta += tq; \
    tq -= OD_RSHIFT1(ta); \
    td += tt; \
    tt -= OD_RSHIFT1(td); \
    t2 -= ti; \
    ti += OD_RSHIFT1(t2); /* pass */ \
    t7 += t8; \
    t8 -= OD_RSHIFT1(t7); \
    tn -= to; \
    to += OD_RSHIFT1(tn); \
    tf -= tv; \
    tv += OD_RSHIFT1(tf); \
    t0 += tg; \
    tg -= OD_RSHIFT1(t0); /* pass */ \
    tj -= t3; \
    t3 += OD_RSHIFT1(tj); /* pass */ \
    ts -= tc; \
    tc += OD_RSHIFT1(ts); \
    t4 -= tb; \
    tb += OD_RSHIFT1(t4); /* pass */ \
    tk -= tr; \
    tr += OD_RSHIFT1(tk); \
    \
    t1 = -t1; \
    t3 = -t3; \
    t7 = -t7; \
    t8 = -t8; \
    tg = -tg; \
    tm = -tm; \
    to = -to; \
    \
    /* 14341/16384 ~= Tan[3*Pi/16] + Tan[Pi/8]/2 ~= 0.875285419105846 */ \
    tm -= (t9*14341 + 8192) >> 14; \
    /* 15137/16384 ~= Sin[3*Pi/8] ~= 0.923879532511287 */ \
    t9 += (tm*15137 + 8192) >> 14; \
    /* 4161/16384 ~= Tan[3*Pi/16] - Tan[Pi/8] ~= 0.253965075546204 */ \
    tm -= (t9*4161 + 8192) >> 14; \
    /* 4161/16384 ~= Tan[3*Pi/16] - Tan[Pi/8] ~= 0.253965075546204 */ \
    tp -= (t6*4161 + 8192) >> 14; \
    /* 15137/16384 ~= Sin[3*Pi/8] ~= 0.923879532511287 */ \
    t6 += (tp*15137 + 8192) >> 14; \
    /* 28681/32768 ~= Tan[3*Pi/16] + Tan[Pi/8]/2 ~= 0.875285419105846 */ \
    tp -= (t6*28681 + 16384) >> 15; \
    /* -19195/32768 ~= Tan[Pi/8] - Tan[Pi/4] ~= -0.585786437626905 */ \
    th += (te*19195 + 16384) >> 15; \
    /* 11585/16384 ~= Sin[Pi/4] ~= 0.707106781186548 */ \
    te += (th*11585 + 8192) >> 14; \
    /* 29957/32768 ~= Tan[Pi/8] + Tan[Pi/4]/2 ~= 0.914213562373095 */ \
    th -= (te*29957 + 16384) >> 15; \
    /* 14341/16384 ~= Tan[3*Pi/16] + Tan[Pi/8]/2 ~= 0.875285419105846 */ \
    tq -= (t5*14341 + 8192) >> 14; \
    /* 15137/16384 ~= Sin[3*Pi/8] ~= 0.923879532511287 */ \
    t5 += (tq*15137 + 8192) >> 14; \
    /* 4161/16384 ~= Tan[3*Pi/16] - Tan[Pi/8] ~= 0.253965075546204 */ \
    tq -= (t5*4161 + 8192) >> 14; \
    /* 3259/8192 ~= 2*Tan[Pi/16] ~= 0.397824734759316 */ \
    ta -= (tl*3259 + 4096) >> 13; \
    /* 3135/16384 ~= Sin[Pi/8]/2 ~= 0.1913417161825449 */ \
    tl += (ta*3135 + 8192) >> 14; \
    /* 3259/8192 ~= 2*Tan[Pi/16] ~= 0.397824734759316 */ \
    ta -= (tl*3259 + 4096) >> 13; \
    /* 7489/8192 ~= Tan[Pi/8] + Tan[Pi/4]/2 ~= 0.914213562373095 */ \
    ti -= (td*7489 + 4096) >> 13; \
    /* 11585/16384 ~= Sin[Pi/4] ~= 0.707106781186548 */ \
    td += (ti*11585 + 8192) >> 14; \
    /* -19195/32768 ~= Tan[Pi/8] - Tan[Pi/4] ~= -0.585786437626905 */ \
    ti += (td*19195 + 16384) >> 15; \
    /* 14341/16384 ~= Tan[3*Pi/16] + Tan[Pi/8]/2 ~= 0.875285419105846 */ \
    to -= (t7*14341 + 8192) >> 14; \
    /* 15137/16384 ~= Sin[3*Pi/8] ~= 0.923879532511287 */ \
    t7 += (to*15137 + 8192) >> 14; \
    /* 4161/16384 ~= Tan[3*Pi/16] - Tan[Pi/8] ~= 0.253965075546204 */ \
    to -= (t7*4161 + 8192) >> 14; \
    /* 4161/16384 ~= Tan[3*Pi/16] - Tan[Pi/8] ~= 0.253965075546204 */ \
    tn -= (t8*4161 + 8192) >> 14; \
    /* 15137/16384 ~= Sin[3*Pi/8] ~= 0.923879532511287 */ \
    t8 += (tn*15137 + 8192) >> 14; \
    /* 28681/32768 ~= Tan[3*Pi/16] + Tan[Pi/8]/2 ~= 0.875285419105846 */ \
    tn -= (t8*28681 + 16384) >> 15; \
    /* -19195/32768 ~= Tan[Pi/8] - Tan[Pi/4] ~= -0.585786437626905 */ \
    tf += (tg*19195 + 16384) >> 15; \
    /* 11585/16384 ~= Sin[Pi/4] ~= 0.707106781186548 */ \
    tg += (tf*11585 + 8192) >> 14; \
    /* 29957/32768 ~= Tan[Pi/8] + Tan[Pi/4]/2 ~= 0.914213562373095 */ \
    tf -= (tg*29957 + 16384) >> 15; \
    /* -19195/32768 ~= Tan[Pi/8] - Tan[Pi/4] ~= -0.585786437626905 */ \
    tj += (tc*19195 + 16384) >> 15; \
    /* 11585/16384 ~= Sin[Pi/4] ~= 0.707106781186548 */ \
    tc += (tj*11585 + 8192) >> 14; \
    /* 29957/32768 ~= Tan[Pi/8] + Tan[Pi/4]/2 ~= 0.914213562373095 */ \
    tj -= (tc*29957 + 16384) >> 15; \
    /* 13573/16384 ~= 2*Tan[Pi/8] ~= 0.828427124746190 */ \
    tk += (tb*13573 + 8192) >> 14; \
    /* 11585/32768 ~= Sin[Pi/4]/2 ~= 0.353553390593274 */ \
    tb -= (tk*11585 + 16384) >> 15; \
    /* 13573/16384 ~= 2*Tan[Pi/8] ~= 0.828427124746190 */ \
    tk += (tb*13573 + 8192) >> 14; \
    \
    tf = -tf; \
    \
} while (0)

/* Embedded 64-point orthonormal Type-II fDCT. */
#define OD_FDCT_64(u0, uw, ug, uM, u8, uE, uo, uU, u4, uA, uk, uQ, uc, uI, \
                   us, uY, u2, uy, ui, uO, ua, uG, uq, uW, u6, uC, um, uS, ue, uK, \
                   uu, u_, u1, ux, uh, uN, u9, uF, up, uV, u5, uB, ul, uR, ud, uJ, \
                   ut, uZ, u3, uz, uj, uP, ub, uH, ur, uX, u7, uD, un, uT, uf, uL, uv, u) \
do { \
    dctcoef uwh; \
    dctcoef uxh; \
    dctcoef uyh; \
    dctcoef uzh; \
    dctcoef uAh; \
    dctcoef uBh; \
    dctcoef uCh; \
    dctcoef uDh; \
    dctcoef uEh; \
    dctcoef uFh; \
    dctcoef uGh; \
    dctcoef uHh; \
    dctcoef uIh; \
    dctcoef uJh; \
    dctcoef uKh; \
    dctcoef uLh; \
    dctcoef uMh; \
    dctcoef uNh; \
    dctcoef uOh; \
    dctcoef uPh; \
    dctcoef uQh; \
    dctcoef uRh; \
    dctcoef uSh; \
    dctcoef uTh; \
    dctcoef uUh; \
    dctcoef uVh; \
    dctcoef uWh; \
    dctcoef uXh; \
    dctcoef uYh; \
    dctcoef uZh; \
    dctcoef u_h; \
    dctcoef uh_; \
    u = u0 - u; \
    uh_ = OD_RSHIFT1(u); \
    u0 -= uh_; \
    u_ += u1; \
    u_h = OD_RSHIFT1(u_); \
    u1 = u_h - u1; \
    uZ = u2 - uZ; \
    uZh = OD_RSHIFT1(uZ); \
    u2 -= uZh; \
    uY += u3; \
    uYh = OD_RSHIFT1(uY); \
    u3 = uYh - u3; \
    uX = u4 - uX; \
    uXh = OD_RSHIFT1(uX); \
    u4 -= uXh; \
    uW += u5; \
    uWh = OD_RSHIFT1(uW); \
    u5 = uWh - u5; \
    uV = u6 - uV; \
    uVh = OD_RSHIFT1(uV); \
    u6 -= uVh; \
    uU += u7; \
    uUh = OD_RSHIFT1(uU); \
    u7 = uUh - u7; \
    uT = u8 - uT; \
    uTh = OD_RSHIFT1(uT); \
    u8 -= uTh; \
    uS += u9; \
    uSh = OD_RSHIFT1(uS); \
    u9 = uSh - u9; \
    uR = ua - uR; \
    uRh = OD_RSHIFT1(uR); \
    ua -= uRh; \
    uQ += ub; \
    uQh = OD_RSHIFT1(uQ); \
    ub = uQh - ub; \
    uP = uc - uP; \
    uPh = OD_RSHIFT1(uP); \
    uc -= uPh; \
    uO += ud; \
    uOh = OD_RSHIFT1(uO); \
    ud = uOh - ud; \
    uN = ue - uN; \
    uNh = OD_RSHIFT1(uN); \
    ue -= uNh; \
    uM += uf; \
    uMh = OD_RSHIFT1(uM); \
    uf = uMh - uf; \
    uL = ug - uL; \
    uLh = OD_RSHIFT1(uL); \
    ug -= uLh; \
    uK += uh; \
    uKh = OD_RSHIFT1(uK); \
    uh = uKh - uh; \
    uJ = ui - uJ; \
    uJh = OD_RSHIFT1(uJ); \
    ui -= uJh; \
    uI += uj; \
    uIh = OD_RSHIFT1(uI); \
    uj = uIh - uj; \
    uH = uk - uH; \
    uHh = OD_RSHIFT1(uH); \
    uk -= uHh; \
    uG += ul; \
    uGh = OD_RSHIFT1(uG); \
    ul = uGh - ul; \
    uF = um - uF; \
    uFh = OD_RSHIFT1(uF); \
    um -= uFh; \
    uE += un; \
    uEh = OD_RSHIFT1(uE); \
    un = uEh - un; \
    uD = uo - uD; \
    uDh = OD_RSHIFT1(uD); \
    uo -= uDh; \
    uC += up; \
    uCh = OD_RSHIFT1(uC); \
    up = uCh - up; \
    uB = uq - uB; \
    uBh = OD_RSHIFT1(uB); \
    uq -= uBh; \
    uA += ur; \
    uAh = OD_RSHIFT1(uA); \
    ur = uAh - ur; \
    uz = us - uz; \
    uzh = OD_RSHIFT1(uz); \
    us -= uzh; \
    uy += ut; \
    uyh = OD_RSHIFT1(uy); \
    ut = uyh - ut; \
    ux = uu - ux; \
    uxh = OD_RSHIFT1(ux); \
    uu -= uxh; \
    uw += uv; \
    uwh = OD_RSHIFT1(uw); \
    uv = uwh - uv; \
    OD_FDCT_32_ASYM(u0, uw, uwh, ug, uM, uMh, u8, uE, uEh, uo, uU, uUh, \
     u4, uA, uAh, uk, uQ, uQh, uc, uI, uIh, us, uY, uYh, u2, uy, uyh, \
     ui, uO, uOh, ua, uG, uGh, uq, uW, uWh, u6, uC, uCh, um, uS, uSh, \
     ue, uK, uKh, uu, u_, u_h); \
    OD_FDST_32_ASYM(u, uv, uL, uf, uT, un, uD, u7, uX, ur, uH, ub, uP, uj, \
     uz, u3, uZ, ut, uJ, ud, uR, ul, uB, u5, uV, up, uF, u9, uN, uh, ux, u1); \
} while (0)

/* Embedded 64-point orthonormal Type-II fDCT. */
#define OD_IDCT_64(u0, uw, ug, uM, u8, uE, uo, uU, u4, uA, uk, uQ, uc, uI, \
                   us, uY, u2, uy, ui, uO, ua, uG, uq, uW, u6, uC, um, uS, ue, uK, \
                   uu, u_, u1, ux, uh, uN, u9, uF, up, uV, u5, uB, ul, uR, ud, uJ, \
                   ut, uZ, u3, uz, uj, uP, ub, uH, ur, uX, u7, uD, un, uT, uf, uL, uv, u) \
do { \
    dctcoef u1h; \
    dctcoef u3h; \
    dctcoef u5h; \
    dctcoef u7h; \
    dctcoef u9h; \
    dctcoef ubh; \
    dctcoef udh; \
    dctcoef ufh; \
    dctcoef uhh; \
    dctcoef ujh; \
    dctcoef ulh; \
    dctcoef unh; \
    dctcoef uph; \
    dctcoef urh; \
    dctcoef uth; \
    dctcoef uvh; \
    dctcoef uxh; \
    dctcoef uzh; \
    dctcoef uBh; \
    dctcoef uDh; \
    dctcoef uFh; \
    dctcoef uHh; \
    dctcoef uJh; \
    dctcoef uLh; \
    dctcoef uNh; \
    dctcoef uPh; \
    dctcoef uRh; \
    dctcoef uTh; \
    dctcoef uVh; \
    dctcoef uXh; \
    dctcoef uZh; \
    dctcoef uh_; \
    OD_IDST_32_ASYM(u, uL, uT, uD, uX, uH, uP, uz, uZ, uJ, uR, uB, uV, uF, \
     uN, ux, u_, uK, uS, uC, uW, uG, uO, uy, uY, uI, uQ, uA, uU, uE, uM, uw); \
    OD_IDCT_32_ASYM(u0, ug, u8, uo, u4, uk, uc, us, u2, ui, ua, uq, u6, um, \
     ue, uu, u1, u1h, uh, uhh, u9, u9h, up, uph, u5, u5h, ul, ulh, ud, udh, \
     ut, uth, u3, u3h, uj, ujh, ub, ubh, ur, urh, u7, u7h, un, unh, uf, ufh, \
     uv, uvh); \
    uh_ = OD_RSHIFT1(u); \
    u0 += uh_; \
    u = u0 - u; \
    u_ = u1h - u_; \
    u1 -= u_; \
    uZh = OD_RSHIFT1(uZ); \
    u2 += uZh; \
    uZ = u2 - uZ; \
    uY = u3h - uY; \
    u3 -= uY; \
    uXh = OD_RSHIFT1(uX); \
    u4 += uXh; \
    uX = u4 - uX; \
    uW = u5h - uW; \
    u5 -= uW; \
    uVh = OD_RSHIFT1(uV); \
    u6 += uVh; \
    uV = u6 - uV; \
    uU = u7h - uU; \
    u7 -= uU; \
    uTh = OD_RSHIFT1(uT); \
    u8 += uTh; \
    uT = u8 - uT; \
    uS = u9h - uS; \
    u9 -= uS; \
    uRh = OD_RSHIFT1(uR); \
    ua += uRh; \
    uR = ua - uR; \
    uQ = ubh - uQ; \
    ub -= uQ; \
    uPh = OD_RSHIFT1(uP); \
    uc += uPh; \
    uP = uc - uP; \
    uO = udh - uO; \
    ud -= uO; \
    uNh = OD_RSHIFT1(uN); \
    ue += uNh; \
    uN = ue - uN; \
    uM = ufh - uM; \
    uf -= uM; \
    uLh = OD_RSHIFT1(uL); \
    ug += uLh; \
    uL = ug - uL; \
    uK = uhh - uK; \
    uh -= uK; \
    uJh = OD_RSHIFT1(uJ); \
    ui += uJh; \
    uJ = ui - uJ; \
    uI = ujh - uI; \
    uj -= uI; \
    uHh = OD_RSHIFT1(uH); \
    uk += uHh; \
    uH = uk - uH; \
    uG = ulh - uG; \
    ul -= uG; \
    uFh = OD_RSHIFT1(uF); \
    um += uFh; \
    uF = um - uF; \
    uE = unh - uE; \
    un -= uE; \
    uDh = OD_RSHIFT1(uD); \
    uo += uDh; \
    uD = uo - uD; \
    uC = uph - uC; \
    up -= uC; \
    uBh = OD_RSHIFT1(uB); \
    uq += uBh; \
    uB = uq - uB; \
    uA = urh - uA; \
    ur -= uA; \
    uzh = OD_RSHIFT1(uz); \
    us += uzh; \
    uz = us - uz; \
    uy = uth - uy; \
    ut -= uy; \
    uxh = OD_RSHIFT1(ux); \
    uu += uxh; \
    ux = uu - ux; \
    uw = uvh - uw; \
    uv -= uw; \
} while (0)

static void od_bin_fdct4(dctcoef y[4], const dctcoef *x, int xstride) {
    dctcoef t0 = x[0*xstride];
    dctcoef t2 = x[1*xstride];
    dctcoef t1 = x[2*xstride];
    dctcoef t3 = x[3*xstride];
    OD_FDCT_4(t0, t2, t1, t3);
    y[0] = t0;
    y[1] = t1;
    y[2] = t2;
    y[3] = t3;
}

static void od_bin_idct4(dctcoef *x, int xstride, const dctcoef y[4]) {
    dctcoef t0 = y[0];
    dctcoef t2 = y[1];
    dctcoef t1 = y[2];
    dctcoef t3 = y[3];
    OD_IDCT_4(t0, t2, t1, t3);
    x[0*xstride] = t0;
    x[1*xstride] = t1;
    x[2*xstride] = t2;
    x[3*xstride] = t3;
}

static void od_bin_fdst4(dctcoef y[4], const dctcoef *x, int xstride) {
    dctcoef t0 = x[0*xstride];
    dctcoef t2 = x[1*xstride];
    dctcoef t1 = x[2*xstride];
    dctcoef t3 = x[3*xstride];
    OD_FDST_4(t0, t2, t1, t3);
    y[0] = t0;
    y[1] = t1;
    y[2] = t2;
    y[3] = t3;
}

static void od_bin_idst4(dctcoef *x, int xstride, const dctcoef y[4]) {
    dctcoef t0 = y[0];
    dctcoef t2 = y[1];
    dctcoef t1 = y[2];
    dctcoef t3 = y[3];
    OD_IDST_4(t0, t2, t1, t3);
    x[0*xstride] = t0;
    x[1*xstride] = t1;
    x[2*xstride] = t2;
    x[3*xstride] = t3;
}

static void od_bin_fdct8(dctcoef y[8], const dctcoef *x, int xstride) {
    dctcoef t0 = x[0*xstride];
    dctcoef t4 = x[1*xstride];
    dctcoef t2 = x[2*xstride];
    dctcoef t6 = x[3*xstride];
    dctcoef t1 = x[4*xstride];
    dctcoef t5 = x[5*xstride];
    dctcoef t3 = x[6*xstride];
    dctcoef t7 = x[7*xstride];
    OD_FDCT_8(t0, t4, t2, t6, t1, t5, t3, t7);
    y[0] = t0;
    y[1] = t1;
    y[2] = t2;
    y[3] = t3;
    y[4] = t4;
    y[5] = t5;
    y[6] = t6;
    y[7] = t7;
}

static void od_bin_idct8(dctcoef *x, int xstride, const dctcoef y[8]) {
    dctcoef t0 = y[0];
    dctcoef t4 = y[1];
    dctcoef t2 = y[2];
    dctcoef t6 = y[3];
    dctcoef t1 = y[4];
    dctcoef t5 = y[5];
    dctcoef t3 = y[6];
    dctcoef t7 = y[7];
    OD_IDCT_8(t0, t4, t2, t6, t1, t5, t3, t7);
    x[0*xstride] = t0;
    x[1*xstride] = t1;
    x[2*xstride] = t2;
    x[3*xstride] = t3;
    x[4*xstride] = t4;
    x[5*xstride] = t5;
    x[6*xstride] = t6;
    x[7*xstride] = t7;
}

static void od_bin_fdst8(dctcoef y[8], const dctcoef *x, int xstride) {
    dctcoef t0 = x[0*xstride];
    dctcoef t4 = x[1*xstride];
    dctcoef t2 = x[2*xstride];
    dctcoef t6 = x[3*xstride];
    dctcoef t1 = x[4*xstride];
    dctcoef t5 = x[5*xstride];
    dctcoef t3 = x[6*xstride];
    dctcoef t7 = x[7*xstride];
    OD_FDST_8(t0, t4, t2, t6, t1, t5, t3, t7);
    y[0] = t0;
    y[1] = t1;
    y[2] = t2;
    y[3] = t3;
    y[4] = t4;
    y[5] = t5;
    y[6] = t6;
    y[7] = t7;
}

static void od_bin_idst8(dctcoef *x, int xstride, const dctcoef y[8]) {
    dctcoef t0 = y[0];
    dctcoef t4 = y[1];
    dctcoef t2 = y[2];
    dctcoef t6 = y[3];
    dctcoef t1 = y[4];
    dctcoef t5 = y[5];
    dctcoef t3 = y[6];
    dctcoef t7 = y[7];
    OD_IDST_8(t0, t4, t2, t6, t1, t5, t3, t7);
    x[0*xstride] = t0;
    x[1*xstride] = t1;
    x[2*xstride] = t2;
    x[3*xstride] = t3;
    x[4*xstride] = t4;
    x[5*xstride] = t5;
    x[6*xstride] = t6;
    x[7*xstride] = t7;
}

static void od_bin_fdct16(dctcoef y[16], const dctcoef *x, int xstride) {
    dctcoef s0 = x[0*xstride];
    dctcoef s8 = x[1*xstride];
    dctcoef s4 = x[2*xstride];
    dctcoef sc = x[3*xstride];
    dctcoef s2 = x[4*xstride];
    dctcoef sa = x[5*xstride];
    dctcoef s6 = x[6*xstride];
    dctcoef se = x[7*xstride];
    dctcoef s1 = x[8*xstride];
    dctcoef s9 = x[9*xstride];
    dctcoef s5 = x[10*xstride];
    dctcoef sd = x[11*xstride];
    dctcoef s3 = x[12*xstride];
    dctcoef sb = x[13*xstride];
    dctcoef s7 = x[14*xstride];
    dctcoef sf = x[15*xstride];
    OD_FDCT_16(s0, s8, s4, sc, s2, sa, s6, se, s1, s9, s5, sd, s3, sb, s7, sf);
    y[0] = s0;
    y[1] = s1;
    y[2] = s2;
    y[3] = s3;
    y[4] = s4;
    y[5] = s5;
    y[6] = s6;
    y[7] = s7;
    y[8] = s8;
    y[9] = s9;
    y[10] = sa;
    y[11] = sb;
    y[12] = sc;
    y[13] = sd;
    y[14] = se;
    y[15] = sf;
}

static void od_bin_idct16(dctcoef *x, int xstride, const dctcoef y[16]) {
    dctcoef s0 = y[0];
    dctcoef s8 = y[1];
    dctcoef s4 = y[2];
    dctcoef sc = y[3];
    dctcoef s2 = y[4];
    dctcoef sa = y[5];
    dctcoef s6 = y[6];
    dctcoef se = y[7];
    dctcoef s1 = y[8];
    dctcoef s9 = y[9];
    dctcoef s5 = y[10];
    dctcoef sd = y[11];
    dctcoef s3 = y[12];
    dctcoef sb = y[13];
    dctcoef s7 = y[14];
    dctcoef sf = y[15];
    OD_IDCT_16(s0, s8, s4, sc, s2, sa, s6, se, s1, s9, s5, sd, s3, sb, s7, sf);
    x[0*xstride] = s0;
    x[1*xstride] = s1;
    x[2*xstride] = s2;
    x[3*xstride] = s3;
    x[4*xstride] = s4;
    x[5*xstride] = s5;
    x[6*xstride] = s6;
    x[7*xstride] = s7;
    x[8*xstride] = s8;
    x[9*xstride] = s9;
    x[10*xstride] = sa;
    x[11*xstride] = sb;
    x[12*xstride] = sc;
    x[13*xstride] = sd;
    x[14*xstride] = se;
    x[15*xstride] = sf;
}

static void od_bin_fdst16(dctcoef y[16], const dctcoef *x, int xstride) {
    dctcoef s0 = x[0*xstride];
    dctcoef s8 = x[1*xstride];
    dctcoef s4 = x[2*xstride];
    dctcoef sc = x[3*xstride];
    dctcoef s2 = x[4*xstride];
    dctcoef sa = x[5*xstride];
    dctcoef s6 = x[6*xstride];
    dctcoef se = x[7*xstride];
    dctcoef s1 = x[8*xstride];
    dctcoef s9 = x[9*xstride];
    dctcoef s5 = x[10*xstride];
    dctcoef sd = x[11*xstride];
    dctcoef s3 = x[12*xstride];
    dctcoef sb = x[13*xstride];
    dctcoef s7 = x[14*xstride];
    dctcoef sf = x[15*xstride];
    OD_FDST_16(s0, s8, s4, sc, s2, sa, s6, se, s1, s9, s5, sd, s3, sb, s7, sf);
    y[0] = s0;
    y[1] = s1;
    y[2] = s2;
    y[3] = s3;
    y[4] = s4;
    y[5] = s5;
    y[6] = s6;
    y[7] = s7;
    y[8] = s8;
    y[9] = s9;
    y[10] = sa;
    y[11] = sb;
    y[12] = sc;
    y[13] = sd;
    y[14] = se;
    y[15] = sf;
}

static void od_bin_idst16(dctcoef *x, int xstride, const dctcoef y[16]) {
    dctcoef s0 = y[0];
    dctcoef s8 = y[1];
    dctcoef s4 = y[2];
    dctcoef sc = y[3];
    dctcoef s2 = y[4];
    dctcoef sa = y[5];
    dctcoef s6 = y[6];
    dctcoef se = y[7];
    dctcoef s1 = y[8];
    dctcoef s9 = y[9];
    dctcoef s5 = y[10];
    dctcoef sd = y[11];
    dctcoef s3 = y[12];
    dctcoef sb = y[13];
    dctcoef s7 = y[14];
    dctcoef sf = y[15];
    OD_IDST_16(s0, s8, s4, sc, s2, sa, s6, se, s1, s9, s5, sd, s3, sb, s7, sf);
    x[0*xstride] = s0;
    x[1*xstride] = s1;
    x[2*xstride] = s2;
    x[3*xstride] = s3;
    x[4*xstride] = s4;
    x[5*xstride] = s5;
    x[6*xstride] = s6;
    x[7*xstride] = s7;
    x[8*xstride] = s8;
    x[9*xstride] = s9;
    x[10*xstride] = sa;
    x[11*xstride] = sb;
    x[12*xstride] = sc;
    x[13*xstride] = sd;
    x[14*xstride] = se;
    x[15*xstride] = sf;
}

/* 215 adds, 38 shifts, 87 "muls". */
static void od_bin_fdct32(dctcoef y[32], const dctcoef *x, int xstride) {
    dctcoef t0 = x[0*xstride];
    dctcoef tg = x[1*xstride];
    dctcoef t8 = x[2*xstride];
    dctcoef to = x[3*xstride];
    dctcoef t4 = x[4*xstride];
    dctcoef tk = x[5*xstride];
    dctcoef tc = x[6*xstride];
    dctcoef ts = x[7*xstride];
    dctcoef t2 = x[8*xstride];
    dctcoef ti = x[9*xstride];
    dctcoef ta = x[10*xstride];
    dctcoef tq = x[11*xstride];
    dctcoef t6 = x[12*xstride];
    dctcoef tm = x[13*xstride];
    dctcoef te = x[14*xstride];
    dctcoef tu = x[15*xstride];
    dctcoef t1 = x[16*xstride];
    dctcoef th = x[17*xstride];
    dctcoef t9 = x[18*xstride];
    dctcoef tp = x[19*xstride];
    dctcoef t5 = x[20*xstride];
    dctcoef tl = x[21*xstride];
    dctcoef td = x[22*xstride];
    dctcoef tt = x[23*xstride];
    dctcoef t3 = x[24*xstride];
    dctcoef tj = x[25*xstride];
    dctcoef tb = x[26*xstride];
    dctcoef tr = x[27*xstride];
    dctcoef t7 = x[28*xstride];
    dctcoef tn = x[29*xstride];
    dctcoef tf = x[30*xstride];
    dctcoef tv = x[31*xstride];
    OD_FDCT_32(t0, tg, t8, to, t4, tk, tc, ts, t2, ti, ta, tq, t6, tm, te, tu,
               t1, th, t9, tp, t5, tl, td, tt, t3, tj, tb, tr, t7, tn, tf, tv);
    y[0] = t0;
    y[1] = t1;
    y[2] = t2;
    y[3] = t3;
    y[4] = t4;
    y[5] = t5;
    y[6] = t6;
    y[7] = t7;
    y[8] = t8;
    y[9] = t9;
    y[10] = ta;
    y[11] = tb;
    y[12] = tc;
    y[13] = td;
    y[14] = te;
    y[15] = tf;
    y[16] = tg;
    y[17] = th;
    y[18] = ti;
    y[19] = tj;
    y[20] = tk;
    y[21] = tl;
    y[22] = tm;
    y[23] = tn;
    y[24] = to;
    y[25] = tp;
    y[26] = tq;
    y[27] = tr;
    y[28] = ts;
    y[29] = tt;
    y[30] = tu;
    y[31] = tv;
}

static void od_bin_idct32(dctcoef *x, int xstride, const dctcoef y[32]) {
    dctcoef t0 = y[0];
    dctcoef tg = y[1];
    dctcoef t8 = y[2];
    dctcoef to = y[3];
    dctcoef t4 = y[4];
    dctcoef tk = y[5];
    dctcoef tc = y[6];
    dctcoef ts = y[7];
    dctcoef t2 = y[8];
    dctcoef ti = y[9];
    dctcoef ta = y[10];
    dctcoef tq = y[11];
    dctcoef t6 = y[12];
    dctcoef tm = y[13];
    dctcoef te = y[14];
    dctcoef tu = y[15];
    dctcoef t1 = y[16];
    dctcoef th = y[17];
    dctcoef t9 = y[18];
    dctcoef tp = y[19];
    dctcoef t5 = y[20];
    dctcoef tl = y[21];
    dctcoef td = y[22];
    dctcoef tt = y[23];
    dctcoef t3 = y[24];
    dctcoef tj = y[25];
    dctcoef tb = y[26];
    dctcoef tr = y[27];
    dctcoef t7 = y[28];
    dctcoef tn = y[29];
    dctcoef tf = y[30];
    dctcoef tv = y[31];
    OD_IDCT_32(t0, tg, t8, to, t4, tk, tc, ts, t2, ti, ta, tq, t6, tm, te, tu,
               t1, th, t9, tp, t5, tl, td, tt, t3, tj, tb, tr, t7, tn, tf, tv);
    x[0*xstride] = t0;
    x[1*xstride] = t1;
    x[2*xstride] = t2;
    x[3*xstride] = t3;
    x[4*xstride] = t4;
    x[5*xstride] = t5;
    x[6*xstride] = t6;
    x[7*xstride] = t7;
    x[8*xstride] = t8;
    x[9*xstride] = t9;
    x[10*xstride] = ta;
    x[11*xstride] = tb;
    x[12*xstride] = tc;
    x[13*xstride] = td;
    x[14*xstride] = te;
    x[15*xstride] = tf;
    x[16*xstride] = tg;
    x[17*xstride] = th;
    x[18*xstride] = ti;
    x[19*xstride] = tj;
    x[20*xstride] = tk;
    x[21*xstride] = tl;
    x[22*xstride] = tm;
    x[23*xstride] = tn;
    x[24*xstride] = to;
    x[25*xstride] = tp;
    x[26*xstride] = tq;
    x[27*xstride] = tr;
    x[28*xstride] = ts;
    x[29*xstride] = tt;
    x[30*xstride] = tu;
    x[31*xstride] = tv;
}

static void od_bin_fdst32(dctcoef y[32], const dctcoef *x, int xstride) {
    dctcoef t0 = x[0*xstride];
    dctcoef tg = x[1*xstride];
    dctcoef t8 = x[2*xstride];
    dctcoef to = x[3*xstride];
    dctcoef t4 = x[4*xstride];
    dctcoef tk = x[5*xstride];
    dctcoef tc = x[6*xstride];
    dctcoef ts = x[7*xstride];
    dctcoef t2 = x[8*xstride];
    dctcoef ti = x[9*xstride];
    dctcoef ta = x[10*xstride];
    dctcoef tq = x[11*xstride];
    dctcoef t6 = x[12*xstride];
    dctcoef tm = x[13*xstride];
    dctcoef te = x[14*xstride];
    dctcoef tu = x[15*xstride];
    dctcoef t1 = x[16*xstride];
    dctcoef th = x[17*xstride];
    dctcoef t9 = x[18*xstride];
    dctcoef tp = x[19*xstride];
    dctcoef t5 = x[20*xstride];
    dctcoef tl = x[21*xstride];
    dctcoef td = x[22*xstride];
    dctcoef tt = x[23*xstride];
    dctcoef t3 = x[24*xstride];
    dctcoef tj = x[25*xstride];
    dctcoef tb = x[26*xstride];
    dctcoef tr = x[27*xstride];
    dctcoef t7 = x[28*xstride];
    dctcoef tn = x[29*xstride];
    dctcoef tf = x[30*xstride];
    dctcoef tv = x[31*xstride];
    OD_FDST_32(t0, tg, t8, to, t4, tk, tc, ts, t2, ti, ta, tq, t6, tm, te, tu,
               t1, th, t9, tp, t5, tl, td, tt, t3, tj, tb, tr, t7, tn, tf, tv);
    y[0] = t0;
    y[1] = t1;
    y[2] = t2;
    y[3] = t3;
    y[4] = t4;
    y[5] = t5;
    y[6] = t6;
    y[7] = t7;
    y[8] = t8;
    y[9] = t9;
    y[10] = ta;
    y[11] = tb;
    y[12] = tc;
    y[13] = td;
    y[14] = te;
    y[15] = tf;
    y[16] = tg;
    y[17] = th;
    y[18] = ti;
    y[19] = tj;
    y[20] = tk;
    y[21] = tl;
    y[22] = tm;
    y[23] = tn;
    y[24] = to;
    y[25] = tp;
    y[26] = tq;
    y[27] = tr;
    y[28] = ts;
    y[29] = tt;
    y[30] = tu;
    y[31] = tv;
}

static void od_bin_idst32(dctcoef *x, int xstride, const dctcoef y[32]) {
    dctcoef t0 = y[0];
    dctcoef tg = y[1];
    dctcoef t8 = y[2];
    dctcoef to = y[3];
    dctcoef t4 = y[4];
    dctcoef tk = y[5];
    dctcoef tc = y[6];
    dctcoef ts = y[7];
    dctcoef t2 = y[8];
    dctcoef ti = y[9];
    dctcoef ta = y[10];
    dctcoef tq = y[11];
    dctcoef t6 = y[12];
    dctcoef tm = y[13];
    dctcoef te = y[14];
    dctcoef tu = y[15];
    dctcoef t1 = y[16];
    dctcoef th = y[17];
    dctcoef t9 = y[18];
    dctcoef tp = y[19];
    dctcoef t5 = y[20];
    dctcoef tl = y[21];
    dctcoef td = y[22];
    dctcoef tt = y[23];
    dctcoef t3 = y[24];
    dctcoef tj = y[25];
    dctcoef tb = y[26];
    dctcoef tr = y[27];
    dctcoef t7 = y[28];
    dctcoef tn = y[29];
    dctcoef tf = y[30];
    dctcoef tv = y[31];
    OD_IDST_32(t0, tg, t8, to, t4, tk, tc, ts, t2, ti, ta, tq, t6, tm, te, tu,
               t1, th, t9, tp, t5, tl, td, tt, t3, tj, tb, tr, t7, tn, tf, tv);
    x[0*xstride] = t0;
    x[1*xstride] = t1;
    x[2*xstride] = t2;
    x[3*xstride] = t3;
    x[4*xstride] = t4;
    x[5*xstride] = t5;
    x[6*xstride] = t6;
    x[7*xstride] = t7;
    x[8*xstride] = t8;
    x[9*xstride] = t9;
    x[10*xstride] = ta;
    x[11*xstride] = tb;
    x[12*xstride] = tc;
    x[13*xstride] = td;
    x[14*xstride] = te;
    x[15*xstride] = tf;
    x[16*xstride] = tg;
    x[17*xstride] = th;
    x[18*xstride] = ti;
    x[19*xstride] = tj;
    x[20*xstride] = tk;
    x[21*xstride] = tl;
    x[22*xstride] = tm;
    x[23*xstride] = tn;
    x[24*xstride] = to;
    x[25*xstride] = tp;
    x[26*xstride] = tq;
    x[27*xstride] = tr;
    x[28*xstride] = ts;
    x[29*xstride] = tt;
    x[30*xstride] = tu;
    x[31*xstride] = tv;
}

static void od_bin_fdct64(dctcoef y[64], const dctcoef *x, int xstride) {
    dctcoef t0 = x[0*xstride];
    dctcoef tw = x[1*xstride];
    dctcoef tg = x[2*xstride];
    dctcoef tM = x[3*xstride];
    dctcoef t8 = x[4*xstride];
    dctcoef tE = x[5*xstride];
    dctcoef to = x[6*xstride];
    dctcoef tU = x[7*xstride];
    dctcoef t4 = x[8*xstride];
    dctcoef tA = x[9*xstride];
    dctcoef tk = x[10*xstride];
    dctcoef tQ = x[11*xstride];
    dctcoef tc = x[12*xstride];
    dctcoef tI = x[13*xstride];
    dctcoef ts = x[14*xstride];
    dctcoef tY = x[15*xstride];
    dctcoef t2 = x[16*xstride];
    dctcoef ty = x[17*xstride];
    dctcoef ti = x[18*xstride];
    dctcoef tO = x[19*xstride];
    dctcoef ta = x[20*xstride];
    dctcoef tG = x[21*xstride];
    dctcoef tq = x[22*xstride];
    dctcoef tW = x[23*xstride];
    dctcoef t6 = x[24*xstride];
    dctcoef tC = x[25*xstride];
    dctcoef tm = x[26*xstride];
    dctcoef tS = x[27*xstride];
    dctcoef te = x[28*xstride];
    dctcoef tK = x[29*xstride];
    dctcoef tu = x[30*xstride];
    dctcoef t_ = x[31*xstride];
    dctcoef t1 = x[32*xstride];
    dctcoef tx = x[33*xstride];
    dctcoef th = x[34*xstride];
    dctcoef tN = x[35*xstride];
    dctcoef t9 = x[36*xstride];
    dctcoef tF = x[37*xstride];
    dctcoef tp = x[38*xstride];
    dctcoef tV = x[39*xstride];
    dctcoef t5 = x[40*xstride];
    dctcoef tB = x[41*xstride];
    dctcoef tl = x[42*xstride];
    dctcoef tR = x[43*xstride];
    dctcoef td = x[44*xstride];
    dctcoef tJ = x[45*xstride];
    dctcoef tt = x[46*xstride];
    dctcoef tZ = x[47*xstride];
    dctcoef t3 = x[48*xstride];
    dctcoef tz = x[49*xstride];
    dctcoef tj = x[50*xstride];
    dctcoef tP = x[51*xstride];
    dctcoef tb = x[52*xstride];
    dctcoef tH = x[53*xstride];
    dctcoef tr = x[54*xstride];
    dctcoef tX = x[55*xstride];
    dctcoef t7 = x[56*xstride];
    dctcoef tD = x[57*xstride];
    dctcoef tn = x[58*xstride];
    dctcoef tT = x[59*xstride];
    dctcoef tf = x[60*xstride];
    dctcoef tL = x[61*xstride];
    dctcoef tv = x[62*xstride];
    dctcoef t  = x[63*xstride];
    OD_FDCT_64(t0, tw, tg, tM, t8, tE, to, tU, t4, tA, tk, tQ, tc, tI, ts, tY, t2,
               ty, ti, tO, ta, tG, tq, tW, t6, tC, tm, tS, te, tK, tu, t_, t1, tx,
               th, tN, t9, tF, tp, tV, t5, tB, tl, tR, td, tJ, tt, tZ, t3, tz, tj,
               tP, tb, tH, tr, tX, t7, tD, tn, tT, tf, tL, tv, t);
    y[0] = t0;
    y[1] = t1;
    y[2] = t2;
    y[3] = t3;
    y[4] = t4;
    y[5] = t5;
    y[6] = t6;
    y[7] = t7;
    y[8] = t8;
    y[9] = t9;
    y[10] = ta;
    y[11] = tb;
    y[12] = tc;
    y[13] = td;
    y[14] = te;
    y[15] = tf;
    y[16] = tg;
    y[17] = th;
    y[18] = ti;
    y[19] = tj;
    y[20] = tk;
    y[21] = tl;
    y[22] = tm;
    y[23] = tn;
    y[24] = to;
    y[25] = tp;
    y[26] = tq;
    y[27] = tr;
    y[28] = ts;
    y[29] = tt;
    y[30] = tu;
    y[31] = tv;
    y[32] = tw;
    y[33] = tx;
    y[34] = ty;
    y[35] = tz;
    y[36] = tA;
    y[37] = tB;
    y[38] = tC;
    y[39] = tD;
    y[40] = tE;
    y[41] = tF;
    y[41] = tF;
    y[42] = tG;
    y[43] = tH;
    y[44] = tI;
    y[45] = tJ;
    y[46] = tK;
    y[47] = tL;
    y[48] = tM;
    y[49] = tN;
    y[50] = tO;
    y[51] = tP;
    y[52] = tQ;
    y[53] = tR;
    y[54] = tS;
    y[55] = tT;
    y[56] = tU;
    y[57] = tV;
    y[58] = tW;
    y[59] = tX;
    y[60] = tY;
    y[61] = tZ;
    y[62] = t_;
    y[63] = t;
}

static void od_bin_idct64(dctcoef *x, int xstride, const dctcoef y[64]) {
    dctcoef t0 = y[0];
    dctcoef tw = y[1];
    dctcoef tg = y[2];
    dctcoef tM = y[3];
    dctcoef t8 = y[4];
    dctcoef tE = y[5];
    dctcoef to = y[6];
    dctcoef tU = y[7];
    dctcoef t4 = y[8];
    dctcoef tA = y[9];
    dctcoef tk = y[10];
    dctcoef tQ = y[11];
    dctcoef tc = y[12];
    dctcoef tI = y[13];
    dctcoef ts = y[14];
    dctcoef tY = y[15];
    dctcoef t2 = y[16];
    dctcoef ty = y[17];
    dctcoef ti = y[18];
    dctcoef tO = y[19];
    dctcoef ta = y[20];
    dctcoef tG = y[21];
    dctcoef tq = y[22];
    dctcoef tW = y[23];
    dctcoef t6 = y[24];
    dctcoef tC = y[25];
    dctcoef tm = y[26];
    dctcoef tS = y[27];
    dctcoef te = y[28];
    dctcoef tK = y[29];
    dctcoef tu = y[30];
    dctcoef t_ = y[31];
    dctcoef t1 = y[32];
    dctcoef tx = y[33];
    dctcoef th = y[34];
    dctcoef tN = y[35];
    dctcoef t9 = y[36];
    dctcoef tF = y[37];
    dctcoef tp = y[38];
    dctcoef tV = y[39];
    dctcoef t5 = y[40];
    dctcoef tB = y[41];
    dctcoef tl = y[42];
    dctcoef tR = y[43];
    dctcoef td = y[44];
    dctcoef tJ = y[45];
    dctcoef tt = y[46];
    dctcoef tZ = y[47];
    dctcoef t3 = y[48];
    dctcoef tz = y[49];
    dctcoef tj = y[50];
    dctcoef tP = y[51];
    dctcoef tb = y[52];
    dctcoef tH = y[53];
    dctcoef tr = y[54];
    dctcoef tX = y[55];
    dctcoef t7 = y[56];
    dctcoef tD = y[57];
    dctcoef tn = y[58];
    dctcoef tT = y[59];
    dctcoef tf = y[60];
    dctcoef tL = y[61];
    dctcoef tv = y[62];
    dctcoef t  = y[63];
    OD_IDCT_64(t0, tw, tg, tM, t8, tE, to, tU, t4, tA, tk, tQ, tc, tI, ts, tY, t2,
               ty, ti, tO, ta, tG, tq, tW, t6, tC, tm, tS, te, tK, tu, t_, t1, tx,
               th, tN, t9, tF, tp, tV, t5, tB, tl, tR, td, tJ, tt, tZ, t3, tz, tj,
               tP, tb, tH, tr, tX, t7, tD, tn, tT, tf, tL, tv, t);
    x[0*xstride]  = t0;
    x[1*xstride]  = t1;
    x[2*xstride]  = t2;
    x[3*xstride]  = t3;
    x[4*xstride]  = t4;
    x[5*xstride]  = t5;
    x[6*xstride]  = t6;
    x[7*xstride]  = t7;
    x[8*xstride]  = t8;
    x[9*xstride]  = t9;
    x[10*xstride] = ta;
    x[11*xstride] = tb;
    x[12*xstride] = tc;
    x[13*xstride] = td;
    x[14*xstride] = te;
    x[15*xstride] = tf;
    x[16*xstride] = tg;
    x[17*xstride] = th;
    x[18*xstride] = ti;
    x[19*xstride] = tj;
    x[20*xstride] = tk;
    x[21*xstride] = tl;
    x[22*xstride] = tm;
    x[23*xstride] = tn;
    x[24*xstride] = to;
    x[25*xstride] = tp;
    x[26*xstride] = tq;
    x[27*xstride] = tr;
    x[28*xstride] = ts;
    x[29*xstride] = tt;
    x[30*xstride] = tu;
    x[31*xstride] = tv;
    x[32*xstride] = tw;
    x[33*xstride] = tx;
    x[34*xstride] = ty;
    x[35*xstride] = tz;
    x[36*xstride] = tA;
    x[37*xstride] = tB;
    x[38*xstride] = tC;
    x[39*xstride] = tD;
    x[40*xstride] = tE;
    x[41*xstride] = tF;
    x[41*xstride] = tF;
    x[42*xstride] = tG;
    x[43*xstride] = tH;
    x[44*xstride] = tI;
    x[45*xstride] = tJ;
    x[46*xstride] = tK;
    x[47*xstride] = tL;
    x[48*xstride] = tM;
    x[49*xstride] = tN;
    x[50*xstride] = tO;
    x[51*xstride] = tP;
    x[52*xstride] = tQ;
    x[53*xstride] = tR;
    x[54*xstride] = tS;
    x[55*xstride] = tT;
    x[56*xstride] = tU;
    x[57*xstride] = tV;
    x[58*xstride] = tW;
    x[59*xstride] = tX;
    x[60*xstride] = tY;
    x[61*xstride] = tZ;
    x[62*xstride] = t_;
    x[63*xstride] = t;
}

static void tx_fwd_2d(FFV2DSP *ctx, int tx, dctcoef *dst, int dst_stride,
                      const dctcoef *src, int src_stride)
{
    const int len_x = FFV2_IDX_TO_BS(FFV2_IDX_X(tx));
    const int len_y = FFV2_IDX_TO_BS(FFV2_IDX_Y(tx));
    LOCAL_ALIGNED_32(dctcoef, tmp, [FFV2_MAX_TX * FFV2_MAX_TX]);
    for (int i = 0; i < len_x; i++)
        ctx->fwd_tx_1d[FFV2_IDX_Y(tx)][FFV2_IDX_TX(tx)](tmp + len_y*i, src + i, src_stride);
    for (int i = 0; i < len_y; i++)
        ctx->fwd_tx_1d[FFV2_IDX_X(tx)][FFV2_IDX_TX(tx)](dst + dst_stride*i, tmp + i, len_y);
}

static void tx_inv_2d(FFV2DSP *ctx, int tx, dctcoef *dst, int dst_stride,
                      const dctcoef *src, int src_stride)
{
    const int len_x = FFV2_IDX_TO_BS(FFV2_IDX_X(tx));
    const int len_y = FFV2_IDX_TO_BS(FFV2_IDX_Y(tx));
    LOCAL_ALIGNED_32(dctcoef, tmp, [FFV2_MAX_TX * FFV2_MAX_TX]);
    for (int i = 0; i < len_y; i++)
        ctx->inv_tx_1d[FFV2_IDX_X(tx)][FFV2_IDX_TX(tx)](tmp + i, len_y, src + src_stride*i);
    for (int i = 0; i < len_x; i++)
        ctx->inv_tx_1d[FFV2_IDX_Y(tx)][FFV2_IDX_TX(tx)](dst + i, dst_stride, tmp + len_y*i);
}

av_cold int ff_ffv2dsp_init(FFV2DSP *ctx, int depth)
{
    ctx->fwd_tx = tx_fwd_2d;
    ctx->inv_tx = tx_inv_2d;

#define TX_PAIR(TYPE, FWD_TEMPLATE, INV_TEMPLATE, SIZE)              \
        ctx->fwd_tx_1d[FFV2_TX_ ##SIZE][TYPE] = FWD_TEMPLATE ##SIZE; \
        ctx->inv_tx_1d[FFV2_TX_ ##SIZE][TYPE] = INV_TEMPLATE ##SIZE; \

    TX_PAIR(FFV2_TX_DCT, od_bin_fdct, od_bin_idct,  4)
    TX_PAIR(FFV2_TX_DCT, od_bin_fdct, od_bin_idct,  8)
    TX_PAIR(FFV2_TX_DCT, od_bin_fdct, od_bin_idct, 16)
    TX_PAIR(FFV2_TX_DCT, od_bin_fdct, od_bin_idct, 32)
    TX_PAIR(FFV2_TX_DCT, od_bin_fdct, od_bin_idct, 64)

    TX_PAIR(FFV2_TX_DST, od_bin_fdst, od_bin_idst,  4)
    TX_PAIR(FFV2_TX_DST, od_bin_fdst, od_bin_idst,  8)
    TX_PAIR(FFV2_TX_DST, od_bin_fdst, od_bin_idst, 16)
    TX_PAIR(FFV2_TX_DST, od_bin_fdst, od_bin_idst, 32)

    ctx->lap_prefilter_hor  = lap_prefilter_hor;
    ctx->lap_postfilter_hor = lap_postfilter_hor;
    ctx->lap_prefilter_ver  = lap_prefilter_ver;
    ctx->lap_postfilter_ver = lap_postfilter_ver;

    ctx->coding_to_raster = coding_to_raster;
    ctx->raster_to_coding = raster_to_coding;

    switch (depth) {
    case 8:
        ctx->ref2coeff = ref_2_coeffs_8;
        ctx->coeff2ref = coeffs_2_ref_8;
        break;
    case 10:
        ctx->ref2coeff = ref_2_coeffs_10;
        ctx->coeff2ref = coeffs_2_ref_10;
        break;
    case 12:
        ctx->ref2coeff = ref_2_coeffs_12;
        ctx->coeff2ref = coeffs_2_ref_12;
        break;
    default:
        return 1;
    }
    return 0;
}
