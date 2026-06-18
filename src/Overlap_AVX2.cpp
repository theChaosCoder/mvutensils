#include <stdexcept>
#include <unordered_map>

#include "Overlap.h"

#if defined(MVTOOLS_X86)

#include <immintrin.h>

template <int blockWidth, int blockHeight>
static void overlaps_avx2(uint8_t *pDst8, ptrdiff_t nDstPitch, const uint8_t *pSrc, ptrdiff_t nSrcPitch, const int16_t *pWin, ptrdiff_t nWinPitch) {
    static_assert(blockWidth >= 16 || (blockWidth == 8 && blockHeight >= 2), "");

    int pitchMul = blockWidth == 8 ? 2 : 1;

    /* pWin from 0 to 2048 */
    for (unsigned y = 0; y < blockHeight; y += pitchMul) {
        for (unsigned x = 0; x < blockWidth; x += 16 / pitchMul) {
            uint16_t *pDst = (uint16_t *)pDst8;

            __m256i src, win, dst;

            if (blockWidth == 8) {
                src = _mm256_cvtepu8_epi16(_mm_unpacklo_epi64(_mm_loadl_epi64((const __m128i *)(pSrc + x)), _mm_loadl_epi64((const __m128i *)(pSrc + nSrcPitch + x))));
                win = _mm256_inserti128_si256(_mm256_castsi128_si256(_mm_loadu_si128((const __m128i *)(pWin + x))), _mm_loadu_si128((const __m128i *)(pWin + nWinPitch + x)), 1);
                dst = _mm256_inserti128_si256(_mm256_castsi128_si256(_mm_loadu_si128((const __m128i *)(pDst + x))), _mm_loadu_si128((const __m128i *)(pDst8 + nDstPitch + x * sizeof(uint16_t))), 1);
            } else {
                src = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)(pSrc + x)));
                win = _mm256_loadu_si256((const __m256i *)(pWin + x));
                dst = _mm256_loadu_si256((const __m256i *)(pDst + x));
            }

            // (src * win) >> 6 as the high 16 bits of (src<<8)*(win<<2) =
            // src*win*2^10 (>>16 nets >>6). One mulhi replaces the
            // mullo/mulhi/srli/slli/or split-product sequence.
            src = _mm256_slli_epi16(src, 8);
            win = _mm256_slli_epi16(win, 2);
            dst = _mm256_adds_epu16(dst, _mm256_mulhi_epu16(src, win));

            if (blockWidth == 8) {
                _mm_storeu_si128((__m128i *)(pDst + x), _mm256_castsi256_si128(dst));
                _mm_storeu_si128((__m128i *)(pDst8 + nDstPitch + x * sizeof(uint16_t)), _mm256_extracti128_si256(dst, 1));
            } else {
                _mm256_storeu_si256((__m256i *)(pDst + x), dst);
            }
        }

        pDst8 += nDstPitch * pitchMul;
        pSrc += nSrcPitch * pitchMul;
        pWin += nWinPitch * pitchMul;
    }
}

#endif


enum InstructionSets {
    Scalar,
    SSE2,
    AVX2,
};


// opt can fit in four bits, if the width and height need more than eight bits each.
#define KEY(width, height, bits, opt) (unsigned)(width) << 24 | (height) << 16 | (bits) << 8 | (opt)

#if defined(MVTOOLS_X86)
#define OVERS_AVX2(width, height) \
    { KEY(width, height, 8, AVX2), overlaps_avx2<width, height> },
#else
#define OVERS_AVX2(width, height)
#endif

static const std::unordered_map<uint32_t, OverlapsFunction> overlaps_functions = {
    OVERS_AVX2(8, 2)
    OVERS_AVX2(8, 4)
    OVERS_AVX2(8, 8)
    OVERS_AVX2(8, 16)
    OVERS_AVX2(16, 1)
    OVERS_AVX2(16, 2)
    OVERS_AVX2(16, 4)
    OVERS_AVX2(16, 8)
    OVERS_AVX2(16, 16)
    OVERS_AVX2(16, 32)
    OVERS_AVX2(32, 8)
    OVERS_AVX2(32, 16)
    OVERS_AVX2(32, 32)
    OVERS_AVX2(32, 64)
    OVERS_AVX2(64, 16)
    OVERS_AVX2(64, 32)
    OVERS_AVX2(64, 64)
    OVERS_AVX2(64, 128)
    OVERS_AVX2(128, 32)
    OVERS_AVX2(128, 64)
    OVERS_AVX2(128, 128)
};


OverlapsFunction selectOverlapsFunctionAVX2(unsigned width, unsigned height, unsigned bits) {
    try {
        return overlaps_functions.at(KEY(width, height, bits, AVX2));
    } catch (std::out_of_range &) {
        return nullptr;
    }
}
