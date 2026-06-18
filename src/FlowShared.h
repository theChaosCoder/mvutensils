#pragma once

#include <cstdint>
#include <algorithm>
#include "SuperPyramid.h"
#include "Common.h"

// time-weihted blend src with ref frames (used for interpolation for poor motion estimation)
template <typename PixelType>
static void Blend(uint8_t *MVU_RESTRICT pdst, const uint8_t *MVU_RESTRICT psrc, const uint8_t *MVU_RESTRICT pref, int height, int width, ptrdiff_t stride, int time256) noexcept {
    for (int h = 0; h < height; h++) {
        // Hoist the row casts out of the inner loop; recomputing them per pixel
        // defeats autovectorization (the result is the same unit-stride row).
        const PixelType *psrc_ = (const PixelType *)psrc;
        const PixelType *pref_ = (const PixelType *)pref;
        PixelType *pdst_ = (PixelType *)pdst;

        for (int w = 0; w < width; w++) {
            pdst_[w] = (psrc_[w] * (256 - time256) + pref_[w] * time256) >> 8;
        }
        pdst += stride;
        psrc += stride;
        pref += stride;
    }
}

template <typename PixelType>
static void FlowInter(
        uint8_t *MVU_RESTRICT pdst8, ptrdiff_t dst_pitch,
        const PyramidPlane &prefB, const PyramidPlane &prefF,
        const uint16_t *VXFullB, const uint16_t *VXFullF,
        const uint16_t *VYFullB, const uint16_t *VYFullF,
        const uint16_t *MaskB, const uint16_t *MaskF, ptrdiff_t tilePitch,
        int dstX, int dstY,
        int width, int height,
        int time256) noexcept {

    PixelType *pdst = (PixelType *)pdst8;

    tilePitch /= sizeof(uint16_t);
    dst_pitch /= sizeof(PixelType);

    int nPelLog = ilog2(prefB.nPel);

    for (int h = 0; h < height; h++) {
        int yBase = (h + dstY) << nPelLog;
        const PixelType *prefF0Ptr = reinterpret_cast<const PixelType *>(prefF.GetPointer<PixelType>(dstX << nPelLog, yBase));
        const PixelType *prefB0Ptr = reinterpret_cast<const PixelType *>(prefB.GetPointer<PixelType>(dstX << nPelLog, yBase));
        for (int w = 0; w < width; w++) {
            int xBase = (w + dstX) << nPelLog;
            int vxF = ((static_cast<int>(VXFullF[w]) - (1 << 15)) * time256) >> 8;
            int vyF = ((static_cast<int>(VYFullF[w]) - (1 << 15)) * time256) >> 8;
            int vxB = ((static_cast<int>(VXFullB[w]) - (1 << 15)) * (256 - time256)) >> 8;
            int vyB = ((static_cast<int>(VYFullB[w]) - (1 << 15)) * (256 - time256)) >> 8;
            // Issue both motion-compensated loads before the dependent blend math so the
            // two (likely cache-missing) gathers can overlap (memory-level parallelism).
            int64_t dstF = *reinterpret_cast<const PixelType *>(prefF.GetPointer<PixelType>(vxF + xBase, vyF + yBase));
            int64_t dstB = *reinterpret_cast<const PixelType *>(prefB.GetPointer<PixelType>(vxB + xBase, vyB + yBase));
            int dstF0 = prefF0Ptr[w];
            int dstB0 = prefB0Ptr[w];
            pdst[w] = (PixelType)((((dstF * (256 - MaskF[w]) + ((MaskF[w] * (dstB * (256 - MaskB[w]) + MaskB[w] * dstF0) + 256) >> 8) + 256) >> 8) * (256 - time256) +
                ((dstB * (256 - MaskB[w]) + ((MaskB[w] * (dstF * (256 - MaskF[w]) + MaskF[w] * dstB0) + 256) >> 8) + 256) >> 8) * time256) >> 8) - 1;
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
        uint8_t *MVU_RESTRICT pdst8, ptrdiff_t dst_pitch,
        const PyramidPlane &prefB, const PyramidPlane &prefF,
        const uint16_t *VXFullB, const uint16_t *VXFullF,
        const uint16_t *VYFullB, const uint16_t *VYFullF,
        const uint16_t *MaskB, const uint16_t *MaskF, ptrdiff_t tilePitch,
        int dstX, int dstY,
        int width, int height,
        int time256,
        const uint16_t *VXFullBB, const uint16_t *VXFullFF,
        const uint16_t *VYFullBB, const uint16_t *VYFullFF) noexcept {

    PixelType *pdst = (PixelType *)pdst8;

    dst_pitch /= sizeof(PixelType);
    tilePitch /= sizeof(int16_t);

    int nPelLog = ilog2(prefB.nPel);

    for (int h = 0; h < height; h++) {
        int yBase = (h + dstY) << nPelLog;
        for (int w = 0; w < width; w++) {
            int xBase = (w + dstX) << nPelLog;
            int vxF = ((static_cast<int>(VXFullF[w]) - (1 << 15)) * time256) >> 8;
            int vyF = ((static_cast<int>(VYFullF[w]) - (1 << 15)) * time256) >> 8;
            int vxFF = ((static_cast<int>(VXFullFF[w]) - (1 << 15)) * time256) >> 8;
            int vyFF = ((static_cast<int>(VYFullFF[w]) - (1 << 15)) * time256) >> 8;
            int vxB = ((static_cast<int>(VXFullB[w]) - (1 << 15)) * (256 - time256)) >> 8;
            int vyB = ((static_cast<int>(VYFullB[w]) - (1 << 15)) * (256 - time256)) >> 8;
            int vxBB = ((static_cast<int>(VXFullBB[w]) - (1 << 15)) * (256 - time256)) >> 8;
            int vyBB = ((static_cast<int>(VYFullBB[w]) - (1 << 15)) * (256 - time256)) >> 8;
            // Issue all four motion-compensated loads before the dependent median/blend math
            // so the (likely cache-missing) gathers can overlap (memory-level parallelism).
            int dstF = *reinterpret_cast<const PixelType *>(prefF.GetPointer<PixelType>(vxF + xBase, vyF + yBase));
            int dstFF = *reinterpret_cast<const PixelType *>(prefF.GetPointer<PixelType>(vxFF + xBase, vyFF + yBase));
            int dstB = *reinterpret_cast<const PixelType *>(prefB.GetPointer<PixelType>(vxB + xBase, vyB + yBase));
            int dstBB = *reinterpret_cast<const PixelType *>(prefB.GetPointer<PixelType>(vxBB + xBase, vyBB + yBase));

            /* use median, firsly get min max of compensations */
            int minfb = std::min(dstB, dstF);
            int maxfb = std::max(dstB, dstF);

            int medianBB = std::max(minfb, std::min(dstBB, maxfb));
            int medianFF = std::max(minfb, std::min(dstFF, maxfb));

            pdst[w] = ((((medianBB * MaskF[w] + dstF * (256 - MaskF[w]) + 256) >> 8) * (256 - time256) +
                ((medianFF * MaskB[w] + dstB * (256 - MaskB[w]) + 256) >> 8) * time256) >> 8) - 1;
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