#pragma once

#include <cstdint>
#include <algorithm>
#include <VSHelper4.h>
#include "SuperPyramid.h"

// time-weihted blend src with ref frames (used for interpolation for poor motion estimation)
template <typename PixelType>
static void Blend(uint8_t *VS_RESTRICT pdst, const uint8_t *VS_RESTRICT psrc, const uint8_t *VS_RESTRICT pref, int height, int width, ptrdiff_t stride, int time256) {
    for (int h = 0; h < height; h++) {
        for (int w = 0; w < width; w++) {
            const PixelType *psrc_ = (const PixelType *)psrc;
            const PixelType *pref_ = (const PixelType *)pref;
            PixelType *pdst_ = (PixelType *)pdst;

            pdst_[w] = (psrc_[w] * (256 - time256) + pref_[w] * time256) >> 8;
        }
        pdst += stride;
        psrc += stride;
        pref += stride;
    }
}

template <typename PixelType>
static void FlowInter(
        uint8_t *VS_RESTRICT pdst8, ptrdiff_t dst_pitch,
        const PyramidPlane &prefB, const PyramidPlane &prefF,
        const uint16_t *VXFullB, const uint16_t *VXFullF,
        const uint16_t *VYFullB, const uint16_t *VYFullF,
        const uint16_t *MaskB, const uint16_t *MaskF, ptrdiff_t tilePitch,
        int dstX, int dstY,
        int width, int height,
        int time256) {

    PixelType *pdst = (PixelType *)pdst8;

    tilePitch /= sizeof(uint16_t);
    dst_pitch /= sizeof(PixelType);

    int nPelLog = ilog2(prefB.nPel);

    for (int h = 0; h < height; h++) {
        for (int w = 0; w < width; w++) {
            int vxF = ((static_cast<int>(VXFullF[w]) - (1 << 15)) * time256) >> 8;
            int vyF = ((static_cast<int>(VYFullF[w]) - (1 << 15)) * time256) >> 8;
            int64_t dstF = *reinterpret_cast<const PixelType *>(prefF.GetPointer<PixelType>(vxF + ((w + dstX) << nPelLog), vyF + ((h + dstY) << nPelLog)));
            int dstF0 = *reinterpret_cast<const PixelType *>(prefF.GetPointer<PixelType>(((w + dstX) << nPelLog), ((h + dstY) << nPelLog)));
            int vxB = ((static_cast<int>(VXFullB[w]) - (1 << 15)) * (256 - time256)) >> 8;
            int vyB = ((static_cast<int>(VYFullB[w]) - (1 << 15)) * (256 - time256)) >> 8;
            int64_t dstB = *reinterpret_cast<const PixelType *>(prefB.GetPointer<PixelType>(vxB + ((w + dstX) << nPelLog), vyB + ((h + dstY) << nPelLog)));
            int dstB0 = *reinterpret_cast<const PixelType *>(prefB.GetPointer<PixelType>(((w + dstX) << nPelLog), ((h + dstY) << nPelLog)));
            pdst[w] = (PixelType)((((dstF * (256 - (MaskF[w] >> 8)) + (((MaskF[w] >> 8) * (dstB * (256 - (MaskB[w] >> 8)) + (MaskB[w] >> 8) * dstF0) + 256) >> 8) + 256) >> 8) * (256 - time256) +
                ((dstB * (256 - (MaskB[w] >> 8)) + (((MaskB[w] >> 8) * (dstF * (256 - (MaskF[w] >> 8)) + (MaskF[w] >> 8) * dstB0) + 256) >> 8) + 256) >> 8) * time256) >> 8) - 1;
        }

        pdst += dst_pitch;
        VXFullB += tilePitch;
        VYFullB += tilePitch;
        VXFullF += tilePitch;
        VYFullF += tilePitch;
        MaskB += tilePitch;
        MaskF += tilePitch;
    }
}

template <typename PixelType>
static void FlowInterExtra(
        uint8_t *VS_RESTRICT pdst8, ptrdiff_t dst_pitch,
        const PyramidPlane &prefB, const PyramidPlane &prefF,
        const uint16_t *VXFullB, const uint16_t *VXFullF,
        const uint16_t *VYFullB, const uint16_t *VYFullF,
        const uint16_t *MaskB, const uint16_t *MaskF, ptrdiff_t tilePitch,
        int dstX, int dstY,
        int width, int height,
        int time256,
        const uint16_t *VXFullBB, const uint16_t *VXFullFF,
        const uint16_t *VYFullBB, const uint16_t *VYFullFF) {

    PixelType *pdst = (PixelType *)pdst8;

    dst_pitch /= sizeof(PixelType);
    tilePitch /= sizeof(int16_t);

    int nPelLog = ilog2(prefB.nPel);

    for (int h = 0; h < height; h++) {
        for (int w = 0; w < width; w++) {
            int vxF = ((static_cast<int>(VXFullF[w]) - (1 << 15)) * time256) >> 8;
            int vyF = ((static_cast<int>(VYFullF[w]) - (1 << 15)) * time256) >> 8;
            int dstF = *reinterpret_cast<const PixelType *>(prefF.GetPointer<PixelType>(vxF + ((w + dstX) << nPelLog), vyF + ((h + dstY) << nPelLog)));

            int vxFF = ((static_cast<int>(VXFullFF[w]) - (1 << 15)) * time256) >> 8;
            int vyFF = ((static_cast<int>(VYFullFF[w]) - (1 << 15)) * time256) >> 8;
            int dstFF = *reinterpret_cast<const PixelType *>(prefF.GetPointer<PixelType>(vxFF + ((w + dstX) << nPelLog), vyFF + ((h + dstY) << nPelLog)));

            int vxB = ((static_cast<int>(VXFullB[w]) - (1 << 15)) * (256 - time256)) >> 8;
            int vyB = ((static_cast<int>(VYFullB[w]) - (1 << 15)) * (256 - time256)) >> 8;
            int dstB = *reinterpret_cast<const PixelType *>(prefB.GetPointer<PixelType>(vxB + ((w + dstX) << nPelLog), vyB + ((h + dstY) << nPelLog)));

            int vxBB = ((static_cast<int>(VXFullBB[w]) - (1 << 15)) * (256 - time256)) >> 8;
            int vyBB = ((static_cast<int>(VYFullBB[w]) - (1 << 15)) * (256 - time256)) >> 8;
            int dstBB = *reinterpret_cast<const PixelType *>(prefB.GetPointer<PixelType>(vxBB + ((w + dstX) << nPelLog), vyBB + ((h + dstY) << nPelLog)));

            /* use median, firsly get min max of compensations */
            int minfb = std::min(dstB, dstF);
            int maxfb = std::max(dstB, dstF);

            int medianBB = std::max(minfb, std::min(dstBB, maxfb));
            int medianFF = std::max(minfb, std::min(dstFF, maxfb));

            pdst[w] = ((((medianBB * (MaskF[w] >> 8) + dstF * (256 - (MaskF[w] >> 8)) + 256) >> 8) * (256 - time256) +
                ((medianFF * (MaskB[w] >> 8) + dstB * (256 - (MaskB[w] >> 8)) + 256) >> 8) * time256) >> 8) - 1;
        }
        pdst += dst_pitch;
        VXFullB += tilePitch;
        VYFullB += tilePitch;
        VXFullF += tilePitch;
        VYFullF += tilePitch;
        MaskB += tilePitch;
        MaskF += tilePitch;
        VXFullBB += tilePitch;
        VYFullBB += tilePitch;
        VXFullFF += tilePitch;
        VYFullFF += tilePitch;
    }
}