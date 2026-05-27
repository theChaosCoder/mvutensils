#pragma once

#include <cstdint>
#include <algorithm>
#include <VSHelper4.h>

// top, middle, botom and left, middle, right windows
#define OW_TL 0
#define OW_TM 1
#define OW_TR 2
#define OW_ML 3
#define OW_MM 4
#define OW_MR 5
#define OW_BL 6
#define OW_BM 7
#define OW_BR 8

class OverlapWindows {
    int nx = -1; // window sizes
    int ny = -1;
    int ox = -1; // overap sizes
    int oy = -1;
    int size = -1; // full window size= nx*ny

    int16_t *Overlap9Windows = nullptr;

    float *fWin1UVx = nullptr;
    float *fWin1UVxfirst = nullptr;
    float *fWin1UVxlast = nullptr;
    float *fWin1UVy = nullptr;
    float *fWin1UVyfirst = nullptr;
    float *fWin1UVylast = nullptr;
public:
    OverlapWindows() = default;
    void Init(int nx, int ny, int ox, int oy);
    ~OverlapWindows();
    const int16_t *GetWindow(int i) const {
        return Overlap9Windows + size * i;
    }
};



typedef void (*OverlapsFunction)(uint8_t *pDst, ptrdiff_t nDstPitch,
                                 const uint8_t *pSrc, ptrdiff_t nSrcPitch,
                                 const int16_t *pWin, ptrdiff_t nWinPitch);


typedef void (*ToPixelsFunction)(uint8_t *pDst, ptrdiff_t nDstPitch,
                                 const uint8_t *pSrc, ptrdiff_t nSrcPitch,
                                 int width, int height, int bitsPerSample);

template<typename PixelType2, typename PixelType>
void ToPixels(uint8_t *VS_RESTRICT pDst8, ptrdiff_t nDstPitch, const uint8_t *VS_RESTRICT pSrc8, ptrdiff_t nSrcPitch, int nWidth, int nHeight, int bitsPerSample) {
    int pixelMax = (1 << bitsPerSample) - 1;

    for (int h = 0; h < nHeight; h++) {
        for (int i = 0; i < nWidth; i++) {
            const PixelType2 *pSrc = (const PixelType2 *)pSrc8;
            PixelType *pDst = (PixelType *)pDst8;

            int a = (pSrc[i] + 16) >> 5;
            if (sizeof(PixelType) == 1)
                pDst[i] = a | ((255 - a) >> (sizeof(int) * 8 - 1));
            else
                pDst[i] = std::min(pixelMax, a);
        }
        pDst8 += nDstPitch;
        pSrc8 += nSrcPitch;
    }
}   

OverlapsFunction selectOverlapsFunction(unsigned width, unsigned height, unsigned bits);

#if defined(MVTOOLS_X86)
OverlapsFunction selectOverlapsFunctionAVX2(unsigned width, unsigned height, unsigned bits);
#endif
