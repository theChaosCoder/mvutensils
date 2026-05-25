// Overlap copy (really addition)
// Copyright(c)2006 A.G.Balakhnin aka Fizick

// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA, or visit
// http://www.gnu.org/copyleft/gpl.html .

#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <algorithm>
#include <unordered_map>

#include "CPU.h"
#include "Overlap.h"

#ifndef M_PI
#define M_PI       3.14159265358979323846f
#endif

extern uint32_t g_cpuinfo;

void OverlapWindows::Init(int nx, int ny, int ox, int oy) {
    this->nx = nx;
    this->ny = ny;
    this->ox = ox;
    this->oy = oy;
    this->size = nx * ny;

    //  windows
    fWin1UVx = (float *)malloc(nx * sizeof(float));
    fWin1UVxfirst = (float *)malloc(nx * sizeof(float));
    fWin1UVxlast = (float *)malloc(nx * sizeof(float));
    for (int i = 0; i < ox; i++) {
        fWin1UVx[i] = cosf(M_PI * (i - ox + 0.5f) / (ox * 2));
        fWin1UVx[i] = fWin1UVx[i] * fWin1UVx[i];  // left window (rised cosine)
        fWin1UVxfirst[i] = 1;                                 // very first window
        fWin1UVxlast[i] = fWin1UVx[i];                  // very last
    }
    for (int i = ox; i < nx - ox; i++) {
        fWin1UVx[i] = 1;
        fWin1UVxfirst[i] = 1; // very first window
        fWin1UVxlast[i] = 1;  // very last
    }
    for (int i = nx - ox; i < nx; i++) {
        fWin1UVx[i] = cosf(M_PI * (i - nx + ox + 0.5f) / (ox * 2));
        fWin1UVx[i] = fWin1UVx[i] * fWin1UVx[i];  // right window (falled cosine)
        fWin1UVxfirst[i] = fWin1UVx[i];                 // very first window
        fWin1UVxlast[i] = 1;                                  // very last
    }

    fWin1UVy = (float *)malloc(ny * sizeof(float));
    fWin1UVyfirst = (float *)malloc(ny * sizeof(float));
    fWin1UVylast = (float *)malloc(ny * sizeof(float));
    for (int i = 0; i < oy; i++) {
        fWin1UVy[i] = cosf(M_PI * (i - oy + 0.5f) / (oy * 2));
        fWin1UVy[i] = fWin1UVy[i] * fWin1UVy[i];  // left window (rised cosine)
        fWin1UVyfirst[i] = 1;                                 // very first window
        fWin1UVylast[i] = fWin1UVy[i];                  // very last
    }
    for (int i = oy; i < ny - oy; i++) {
        fWin1UVy[i] = 1;
        fWin1UVyfirst[i] = 1; // very first window
        fWin1UVylast[i] = 1;  // very last
    }
    for (int i = ny - oy; i < ny; i++) {
        fWin1UVy[i] = cosf(M_PI * (i - ny + oy + 0.5f) / (oy * 2));
        fWin1UVy[i] = fWin1UVy[i] * fWin1UVy[i];  // right window (falled cosine)
        fWin1UVyfirst[i] = fWin1UVy[i];                 // very first window
        fWin1UVylast[i] = 1;                                  // very last
    }

    Overlap9Windows = (int16_t *)malloc(size * 9 * sizeof(int16_t));

    int16_t *winOverUVTL = Overlap9Windows;
    int16_t *winOverUVTM = Overlap9Windows + size;
    int16_t *winOverUVTR = Overlap9Windows + size * 2;
    int16_t *winOverUVML = Overlap9Windows + size * 3;
    int16_t *winOverUVMM = Overlap9Windows + size * 4;
    int16_t *winOverUVMR = Overlap9Windows + size * 5;
    int16_t *winOverUVBL = Overlap9Windows + size * 6;
    int16_t *winOverUVBM = Overlap9Windows + size * 7;
    int16_t *winOverUVBR = Overlap9Windows + size * 8;

    for (int j = 0; j < ny; j++) {
        for (int i = 0; i < nx; i++) {
            winOverUVTL[i] = (int)(fWin1UVyfirst[j] * fWin1UVxfirst[i] * 2048 + 0.5f);
            winOverUVTM[i] = (int)(fWin1UVyfirst[j] * fWin1UVx[i] * 2048 + 0.5f);
            winOverUVTR[i] = (int)(fWin1UVyfirst[j] * fWin1UVxlast[i] * 2048 + 0.5f);
            winOverUVML[i] = (int)(fWin1UVy[j] * fWin1UVxfirst[i] * 2048 + 0.5f);
            winOverUVMM[i] = (int)(fWin1UVy[j] * fWin1UVx[i] * 2048 + 0.5f);
            winOverUVMR[i] = (int)(fWin1UVy[j] * fWin1UVxlast[i] * 2048 + 0.5f);
            winOverUVBL[i] = (int)(fWin1UVylast[j] * fWin1UVxfirst[i] * 2048 + 0.5f);
            winOverUVBM[i] = (int)(fWin1UVylast[j] * fWin1UVx[i] * 2048 + 0.5f);
            winOverUVBR[i] = (int)(fWin1UVylast[j] * fWin1UVxlast[i] * 2048 + 0.5f);
        }
        winOverUVTL += nx;
        winOverUVTM += nx;
        winOverUVTR += nx;
        winOverUVML += nx;
        winOverUVMM += nx;
        winOverUVMR += nx;
        winOverUVBL += nx;
        winOverUVBM += nx;
        winOverUVBR += nx;
    }
}



OverlapWindows::~OverlapWindows() {
    free(Overlap9Windows);
    free(fWin1UVx);
    free(fWin1UVxfirst);
    free(fWin1UVxlast);
    free(fWin1UVy);
    free(fWin1UVyfirst);
    free(fWin1UVylast);
}


template <unsigned blockWidth, unsigned blockHeight, typename PixelType2, typename PixelType>
void overlaps_c(uint8_t * VS_RESTRICT pDst8, ptrdiff_t nDstPitch, const uint8_t * VS_RESTRICT pSrc8, ptrdiff_t nSrcPitch, const int16_t * VS_RESTRICT pWin, ptrdiff_t nWinPitch) {
    /* pWin from 0 to 2048 */
    for (unsigned j = 0; j < blockHeight; j++) {
        for (unsigned i = 0; i < blockWidth; i++) {
            PixelType2 *pDst = (PixelType2 *)pDst8;
            const PixelType *pSrc = (const PixelType *)pSrc8;

            pDst[i] += ((pSrc[i] * pWin[i]) >> 6);
        }
        pDst8 += nDstPitch;
        pSrc8 += nSrcPitch;
        pWin += nWinPitch;
    }
}


#if defined(MVTOOLS_X86)


#include <emmintrin.h>


template <unsigned blockWidth, unsigned blockHeight>
struct OverlapsWrapper {
    static_assert(blockWidth >= 8, "");

    static void overlaps_sse2(uint8_t * VS_RESTRICT pDst8, ptrdiff_t nDstPitch, const uint8_t * VS_RESTRICT pSrc, ptrdiff_t nSrcPitch, const int16_t * VS_RESTRICT pWin, ptrdiff_t nWinPitch) {
        /* pWin from 0 to 2048 */
        for (unsigned y = 0; y < blockHeight; y++) {
            for (unsigned x = 0; x < blockWidth; x += 8) {
                uint16_t *pDst = (uint16_t *)pDst8;

                __m128i src = _mm_loadl_epi64((const __m128i *)&pSrc[x]);
                __m128i win = _mm_loadu_si128((const __m128i *)&pWin[x]);
                __m128i dst = _mm_loadu_si128((__m128i *)&pDst[x]);

                src = _mm_unpacklo_epi8(src, _mm_setzero_si128());

                __m128i lo = _mm_mullo_epi16(src, win);
                __m128i hi = _mm_mulhi_epi16(src, win);
                lo = _mm_srli_epi16(lo, 6);
                hi = _mm_slli_epi16(hi, 10);
                dst = _mm_adds_epu16(dst, _mm_or_si128(lo, hi));
                _mm_storeu_si128((__m128i *)&pDst[x], dst);
            }

            pDst8 += nDstPitch;
            pSrc += nSrcPitch;
            pWin += nWinPitch;
        }
    }

};


template <unsigned blockHeight>
struct OverlapsWrapper<4, blockHeight> {

    static void overlaps_sse2(uint8_t * VS_RESTRICT pDst, ptrdiff_t nDstPitch, const uint8_t * VS_RESTRICT pSrc, ptrdiff_t nSrcPitch, const int16_t * VS_RESTRICT pWin, ptrdiff_t nWinPitch) {
        /* pWin from 0 to 2048 */
        for (unsigned y = 0; y < blockHeight; y++) {
            __m128i src = _mm_cvtsi32_si128(*(const int *)pSrc);
            __m128i win = _mm_loadl_epi64((const __m128i *)pWin);
            __m128i dst = _mm_loadl_epi64((const __m128i *)pDst);

            src = _mm_unpacklo_epi8(src, _mm_setzero_si128());

            __m128i lo = _mm_mullo_epi16(src, win);
            __m128i hi = _mm_mulhi_epi16(src, win);
            lo = _mm_srli_epi16(lo, 6);
            hi = _mm_slli_epi16(hi, 10);
            dst = _mm_adds_epu16(dst, _mm_or_si128(lo, hi));
            _mm_storel_epi64((__m128i *)pDst, dst);

            pDst += nDstPitch;
            pSrc += nSrcPitch;
            pWin += nWinPitch;
        }
    }

};


#endif


// opt can fit in four bits, if the width and height need more than eight bits each.
#define KEY(width, height, bits, opt) (unsigned)(width) << 24 | (height) << 16 | (bits) << 8 | (opt)

#if defined(MVTOOLS_X86)
#define OVERS_SSE2(width, height) \
    { KEY(width, height, 8, MVOPT_SSE2), OverlapsWrapper<width, height>::overlaps_sse2 },
#else
#define OVERS_SSE2(width, height)
#endif

#define OVERS(width, height) \
    { KEY(width, height, 8, MVOPT_SCALAR), overlaps_c<width, height, uint16_t, uint8_t> }, \
    { KEY(width, height, 16, MVOPT_SCALAR), overlaps_c<width, height, uint32_t, uint16_t> },

static const std::unordered_map<uint32_t, OverlapsFunction> overlaps_functions = {
    OVERS(2, 2)
    OVERS(2, 4)
    OVERS(4, 2)
    OVERS(4, 4)
    OVERS(4, 8)
    OVERS(8, 1)
    OVERS(8, 2)
    OVERS(8, 4)
    OVERS(8, 8)
    OVERS(8, 16)
    OVERS(16, 1)
    OVERS(16, 2)
    OVERS(16, 4)
    OVERS(16, 8)
    OVERS(16, 16)
    OVERS(16, 32)
    OVERS(32, 8)
    OVERS(32, 16)
    OVERS(32, 32)
    OVERS(32, 64)
    OVERS(64, 16)
    OVERS(64, 32)
    OVERS(64, 64)
    OVERS(64, 128)
    OVERS(128, 32)
    OVERS(128, 64)
    OVERS(128, 128)
    OVERS_SSE2(4, 2)
    OVERS_SSE2(4, 4)
    OVERS_SSE2(4, 8)
    OVERS_SSE2(8, 1)
    OVERS_SSE2(8, 2)
    OVERS_SSE2(8, 4)
    OVERS_SSE2(8, 8)
    OVERS_SSE2(8, 16)
    OVERS_SSE2(16, 1)
    OVERS_SSE2(16, 2)
    OVERS_SSE2(16, 4)
    OVERS_SSE2(16, 8)
    OVERS_SSE2(16, 16)
    OVERS_SSE2(16, 32)
    OVERS_SSE2(32, 8)
    OVERS_SSE2(32, 16)
    OVERS_SSE2(32, 32)
    OVERS_SSE2(32, 64)
    OVERS_SSE2(64, 16)
    OVERS_SSE2(64, 32)
    OVERS_SSE2(64, 64)
    OVERS_SSE2(64, 128)
    OVERS_SSE2(128, 32)
    OVERS_SSE2(128, 64)
    OVERS_SSE2(128, 128)
};

OverlapsFunction selectOverlapsFunction(unsigned width, unsigned height, unsigned bits) {
    OverlapsFunction overs = overlaps_functions.at(KEY(width, height, bits, MVOPT_SCALAR));

#if defined(MVTOOLS_X86)
    int cpu = g_cpuinfo;

    try {
        overs = overlaps_functions.at(KEY(width, height, bits, MVOPT_SSE2));
    } catch (std::out_of_range &) { }
    if (g_cpuinfo & X264_CPU_AVX2) {
        OverlapsFunction tmp = selectOverlapsFunctionAVX2(width, height, bits);
        if (tmp)
            overs = tmp;
    }
#endif

    return overs;
}

#undef OVERS
#undef OVERS_SSE2
#undef KEY
