#include <stdexcept>
#include <unordered_map>

#include "Overlap.h"

#if defined(MVTOOLS_X86)

#include <immintrin.h>

template <int blockWidth, int blockHeight>
static void overlaps_avx2(uint8_t *pDst8, ptrdiff_t nDstPitch, const uint8_t *pSrc, ptrdiff_t nSrcPitch, const int16_t *pWin, ptrdiff_t nWinPitch) {
    static_assert(blockWidth >= 16, "");

    /* pWin from 0 to 2048 */
    for (unsigned y = 0; y < (unsigned)blockHeight; y++) {
        for (unsigned x = 0; x < (unsigned)blockWidth; x += 16) {
            uint16_t *pDst = (uint16_t *)pDst8;

            __m256i src = _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i *)(pSrc + x)));
            __m256i win = _mm256_loadu_si256((const __m256i *)(pWin + x));
            __m256i dst = _mm256_loadu_si256((const __m256i *)(pDst + x));

            // (src * win) >> 6 == high 16 bits of (src<<8)*(win<<2) = src*win*2^10 (>>16 nets >>6).
            src = _mm256_slli_epi16(src, 8);
            win = _mm256_slli_epi16(win, 2);
            dst = _mm256_adds_epu16(dst, _mm256_mulhi_epu16(src, win));
            _mm256_storeu_si256((__m256i *)(pDst + x), dst);
        }

        pDst8 += nDstPitch;
        pSrc += nSrcPitch;
        pWin += nWinPitch;
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
    // width 8 intentionally omitted: the SSE2 intrinsic is faster there (see overlaps_avx2 note)
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
