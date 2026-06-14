// Make a motion compensate temporal denoiser
// Copyright(c)2006 A.G.Balakhnin aka Fizick
// See legal notice in Copying.txt for more information

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

#include <VapourSynth4.h>
#include <VSHelper4.h>

#include "Common.h"
#include "SuperPyramid.h"
#include "MotionBlockPyramid.h"
#include "MaskResize.h"

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

struct FlowFPSData {
    VSNode *node;
    VSVideoInfo vi;
    const VSVideoInfo *oldvi;

    VSNode *super;
    VSNode *mvbw;
    VSNode *mvfw;

    int64_t num, den;
    bool extraMask;
    float ml;
    bool blend;
    int64_t thscd1;
    int thscd2;

    int64_t fa;
    int64_t fb;

    int deltaFrame;

    MaskResizer maskResizerFull;
    MaskResizer maskResizerSubSampled;

    std::string prefix;

    const VSAPI *vsapi;

    FlowFPSData(const VSAPI *vsapi) : vsapi(vsapi) {};

    ~FlowFPSData() {
        vsapi->freeNode(node);
        vsapi->freeNode(super);
        vsapi->freeNode(mvbw);
        vsapi->freeNode(mvfw);
    }
};

template<typename PixelType>
static const VSFrame *VS_CC flowfpsGetFrame(int n, int activationReason, void *instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    const FlowFPSData *d = reinterpret_cast<FlowFPSData *>(instanceData);

    // FIXME, catch exceptions

    if (activationReason == arInitial) {
        int off = -d->deltaFrame;

        int nleft = (int)(n * d->fa / d->fb);
        int nright = nleft + off;

        int time256 = (int)(((double)n * d->fa / d->fb - nleft) * 256 + 0.5);
        if (off > 1)
            time256 = time256 / off;

        if (time256 == 0) {
            vsapi->requestFrameFilter(std::min(nleft, d->oldvi->numFrames - 1), d->node, frameCtx);
            return nullptr;
        } else if (time256 == 256) {
            vsapi->requestFrameFilter(std::min(nright, d->oldvi->numFrames - 1), d->node, frameCtx);
            return nullptr;
        }

        if (nleft < d->oldvi->numFrames && nright < d->oldvi->numFrames) { // for the good estimation case
            if (d->extraMask)
                vsapi->requestFrameFilter(nleft, d->mvfw, frameCtx); // requests nleft - off, nleft
            vsapi->requestFrameFilter(nright, d->mvfw, frameCtx);    // requests nleft, nleft + off
            vsapi->requestFrameFilter(nleft, d->mvbw, frameCtx);     // requests nleft, nleft + off
            if (d->extraMask)
                vsapi->requestFrameFilter(nright, d->mvbw, frameCtx); // requests nleft + off, nleft + off + off

            vsapi->requestFrameFilter(nleft, d->super, frameCtx);
            vsapi->requestFrameFilter(nright, d->super, frameCtx);
        }

        vsapi->requestFrameFilter(std::min(nleft, d->oldvi->numFrames - 1), d->node, frameCtx);

        if (d->blend)
            vsapi->requestFrameFilter(std::min(nright, d->oldvi->numFrames - 1), d->node, frameCtx);

    } else if (activationReason == arAllFramesReady) {
        int nleft = (int)(n * d->fa / d->fb);
        // intermediate product may be very large! Now I know how to multiply int64
        int time256 = (int)(((double)n * d->fa / d->fb - nleft) * 256 + 0.5);

        int off = -d->deltaFrame; // integer offset of reference frame
        // usually off must be = 1
        if (off > 1)
            time256 = time256 / off;

        int nright = nleft + off;

        if (time256 == 0) {
            return vsapi->getFrameFilter(std::min(nleft, d->oldvi->numFrames - 1), d->node, frameCtx); // simply left
        } else if (time256 == 256) {
            return vsapi->getFrameFilter(std::min(nright, d->oldvi->numFrames - 1), d->node, frameCtx); // simply right
        }

        bool vectorsLoadFrame = (nleft < d->oldvi->numFrames && nright < d->oldvi->numFrames);

        const VSFrame *mvF = vectorsLoadFrame ? vsapi->getFrameFilter(nright, d->mvfw, frameCtx) : nullptr;
        MotionBlockPyramid vectorsF(mvF, 1, d->prefix, core, vsapi);
        vsapi->freeFrame(mvF);

        const VSFrame *mvB = vectorsLoadFrame ? vsapi->getFrameFilter(nleft, d->mvbw, frameCtx) : nullptr;
        MotionBlockPyramid vectorsB(mvB, 1, d->prefix, core, vsapi);
        vsapi->freeFrame(mvB);

        if (vectorsB.IsUsable(d->thscd1, d->thscd2) && vectorsF.IsUsable(d->thscd1, d->thscd2)) {
            // If both are usable, that means both nleft and nright are less than oldvi->numFrames. Thus there is no need to check nleft and nright here.
            FramePyramid src(vsapi->getFrameFilter(nleft, d->super, frameCtx), 1, d->prefix, core, vsapi);
            FramePyramid ref(vsapi->getFrameFilter(nright, d->super, frameCtx), 1, d->prefix, core, vsapi);
            const VSFrame *dstPropSrc = vsapi->getFrameFilter(nleft, d->super, frameCtx);
            VSFrame *dst = vsapi->newVideoFrame(&d->vi.format, d->vi.width, d->vi.height, dstPropSrc, core);
            vsapi->freeFrame(dstPropSrc);

            auto SmallB = vectorsB.MakeSmallVectorMasks();
            auto SmallF = vectorsF.MakeSmallVectorMasks();

            ptrdiff_t occlusionMaskPitch = roundUpTo64(vectorsB.nBlkX * sizeof(uint16_t));

            std::unique_ptr<uint16_t, decltype(&vsh::vsh_aligned_free)> MaskSmallB{
                vsh::vsh_aligned_malloc<uint16_t>(occlusionMaskPitch * vectorsB.nBlkY, 64),
                vsh::vsh_aligned_free
            };

            std::unique_ptr<uint16_t, decltype(&vsh::vsh_aligned_free)> MaskSmallF{
                vsh::vsh_aligned_malloc<uint16_t>(occlusionMaskPitch * vectorsF.nBlkY, 64),
                vsh::vsh_aligned_free
            };

            // FIXME, remember to divide occlusion mask values by 256 later
            // analyse vectors field to detect occlusion
            //        double occNormB = (256-time256)/(256*ml);
            //        MakeVectorOcclusionMask(mvClipB, nBlkX, nBlkY, occNormB, 1.0, nPel, MaskSmallB, nBlkXP);
            vectorsB.MakeVectorOcclusionMask<uint16_t>(d->ml, 1.0f, MaskSmallB.get(), occlusionMaskPitch, (256 - time256));

            // analyse vectors field to detect occlusion
            //        double occNormF = time256/(256*ml);
            //        MakeVectorOcclusionMask(mvClipF, nBlkX, nBlkY, occNormF, 1.0, nPel, MaskSmallF, nBlkXP);
            vectorsF.MakeVectorOcclusionMask<uint16_t>(d->ml, 1.0f, MaskSmallF.get(), occlusionMaskPitch, (256 - time256));

            std::unique_ptr<void, decltype(&vsh::vsh_aligned_free)> tmp{
                vsh::vsh_aligned_malloc(std::max(d->maskResizerFull.tmpSize, d->maskResizerSubSampled.tmpSize), 64),
                vsh::vsh_aligned_free
            };

            constexpr ptrdiff_t dstTileStride = roundUpTo64(MaskResizer::TileSize * sizeof(uint16_t));

            std::unique_ptr<uint16_t, decltype(&vsh::vsh_aligned_free)> dstTileMaskF{
                vsh::vsh_aligned_malloc<uint16_t>(dstTileStride * MaskResizer::TileSize, 64),
                vsh::vsh_aligned_free
            };

            std::unique_ptr<uint16_t, decltype(&vsh::vsh_aligned_free)> dstTileMaskB{
                vsh::vsh_aligned_malloc<uint16_t>(dstTileStride * MaskResizer::TileSize, 64),
                vsh::vsh_aligned_free
            };

            std::unique_ptr<uint16_t, decltype(&vsh::vsh_aligned_free)> dstTileXF{
                vsh::vsh_aligned_malloc<uint16_t>(dstTileStride * MaskResizer::TileSize, 64),
                vsh::vsh_aligned_free
            };

            std::unique_ptr<uint16_t, decltype(&vsh::vsh_aligned_free)> dstTileYF{
                vsh::vsh_aligned_malloc<uint16_t>(dstTileStride * MaskResizer::TileSize, 64),
                vsh::vsh_aligned_free
            };

            std::unique_ptr<uint16_t, decltype(&vsh::vsh_aligned_free)> dstTileXB{
                vsh::vsh_aligned_malloc<uint16_t>(dstTileStride * MaskResizer::TileSize, 64),
                vsh::vsh_aligned_free
            };

            std::unique_ptr<uint16_t, decltype(&vsh::vsh_aligned_free)> dstTileYB{
                vsh::vsh_aligned_malloc<uint16_t>(dstTileStride * MaskResizer::TileSize, 64),
                vsh::vsh_aligned_free
            };

            mvuzimgxx::zimage_buffer_const srcBufMaskF;
            srcBufMaskF.plane[0].data = MaskSmallF.get();
            srcBufMaskF.plane[0].stride = occlusionMaskPitch;
            srcBufMaskF.plane[0].mask = ZIMG_BUFFER_MAX;

            mvuzimgxx::zimage_buffer_const srcBufMaskB;
            srcBufMaskB.plane[0].data = MaskSmallB.get();
            srcBufMaskB.plane[0].stride = occlusionMaskPitch;
            srcBufMaskB.plane[0].mask = ZIMG_BUFFER_MAX;

            mvuzimgxx::zimage_buffer_const srcBufSmallFX;
            srcBufSmallFX.plane[0].data = SmallF->VXSmallY;
            srcBufSmallFX.plane[0].stride = SmallF->pitchVSmallY;
            srcBufSmallFX.plane[0].mask = ZIMG_BUFFER_MAX;

            mvuzimgxx::zimage_buffer_const srcBufSmallFY;
            srcBufSmallFY.plane[0].data = SmallF->VYSmallY;
            srcBufSmallFY.plane[0].stride = SmallF->pitchVSmallY;
            srcBufSmallFY.plane[0].mask = ZIMG_BUFFER_MAX;

            mvuzimgxx::zimage_buffer_const srcBufSmallBX;
            srcBufSmallBX.plane[0].data = SmallB->VXSmallY;
            srcBufSmallBX.plane[0].stride = SmallB->pitchVSmallY;
            srcBufSmallBX.plane[0].mask = ZIMG_BUFFER_MAX;

            mvuzimgxx::zimage_buffer_const srcBufSmallBY;
            srcBufSmallBY.plane[0].data = SmallB->VYSmallY;
            srcBufSmallBY.plane[0].stride = SmallB->pitchVSmallY;
            srcBufSmallBY.plane[0].mask = ZIMG_BUFFER_MAX;

            mvuzimgxx::zimage_buffer dstBufMaskF;
            dstBufMaskF.plane[0].data = dstTileMaskF.get();
            dstBufMaskF.plane[0].stride = dstTileStride;
            dstBufMaskF.plane[0].mask = ZIMG_BUFFER_MAX;

            mvuzimgxx::zimage_buffer dstBufMaskB;
            dstBufMaskB.plane[0].data = dstTileMaskB.get();
            dstBufMaskB.plane[0].stride = dstTileStride;
            dstBufMaskB.plane[0].mask = ZIMG_BUFFER_MAX;

            mvuzimgxx::zimage_buffer dstBufSmallXF;
            dstBufSmallXF.plane[0].data = dstTileXF.get();
            dstBufSmallXF.plane[0].stride = dstTileStride;
            dstBufSmallXF.plane[0].mask = ZIMG_BUFFER_MAX;

            mvuzimgxx::zimage_buffer dstBufSmallYF;
            dstBufSmallYF.plane[0].data = dstTileYF.get();
            dstBufSmallYF.plane[0].stride = dstTileStride;
            dstBufSmallYF.plane[0].mask = ZIMG_BUFFER_MAX;

            mvuzimgxx::zimage_buffer dstBufSmallXB;
            dstBufSmallXB.plane[0].data = dstTileXB.get();
            dstBufSmallXB.plane[0].stride = dstTileStride;
            dstBufSmallXB.plane[0].mask = ZIMG_BUFFER_MAX;

            mvuzimgxx::zimage_buffer dstBufSmallYB;
            dstBufSmallYB.plane[0].data = dstTileYB.get();
            dstBufSmallYB.plane[0].stride = dstTileStride;
            dstBufSmallYB.plane[0].mask = ZIMG_BUFFER_MAX;

            // forward from previous to current
            const VSFrame *mvFF = d->extraMask ? vsapi->getFrameFilter(nleft, d->mvfw, frameCtx) : nullptr;
            MotionBlockPyramid vectorsFF(mvFF, 1, d->prefix, core, vsapi);
            vsapi->freeFrame(mvFF);

            // backward from next next to next
            const VSFrame *mvBB = d->extraMask ? vsapi->getFrameFilter(nright, d->mvbw, frameCtx) : nullptr;
            MotionBlockPyramid vectorsBB(mvBB, 1, d->prefix, core, vsapi);
            vsapi->freeFrame(mvBB);

            if (d->extraMask && vectorsBB.IsUsable(d->thscd1, d->thscd2) && vectorsFF.IsUsable(d->thscd1, d->thscd2)) { // slow method with extra frames
                // get vector mask from extra frames
                auto SmallBB = vectorsBB.MakeSmallVectorMasks();
                auto SmallFF = vectorsFF.MakeSmallVectorMasks();

                std::unique_ptr<uint16_t, decltype(&vsh::vsh_aligned_free)> dstTileXFF{
                    vsh::vsh_aligned_malloc<uint16_t>(dstTileStride * MaskResizer::TileSize, 64),
                    vsh::vsh_aligned_free
                };

                std::unique_ptr<uint16_t, decltype(&vsh::vsh_aligned_free)> dstTileYFF{
                    vsh::vsh_aligned_malloc<uint16_t>(dstTileStride * MaskResizer::TileSize, 64),
                    vsh::vsh_aligned_free
                };

                std::unique_ptr<uint16_t, decltype(&vsh::vsh_aligned_free)> dstTileXBB{
                    vsh::vsh_aligned_malloc<uint16_t>(dstTileStride * MaskResizer::TileSize, 64),
                    vsh::vsh_aligned_free
                };

                std::unique_ptr<uint16_t, decltype(&vsh::vsh_aligned_free)> dstTileYBB{
                    vsh::vsh_aligned_malloc<uint16_t>(dstTileStride * MaskResizer::TileSize, 64),
                    vsh::vsh_aligned_free
                };

                mvuzimgxx::zimage_buffer_const srcBufSmallFFX;
                srcBufSmallFFX.plane[0].data = SmallFF->VXSmallY;
                srcBufSmallFFX.plane[0].stride = SmallFF->pitchVSmallY;
                srcBufSmallFFX.plane[0].mask = ZIMG_BUFFER_MAX;

                mvuzimgxx::zimage_buffer_const srcBufSmallFFY;
                srcBufSmallFFY.plane[0].data = SmallFF->VYSmallY;
                srcBufSmallFFY.plane[0].stride = SmallFF->pitchVSmallY;
                srcBufSmallFFY.plane[0].mask = ZIMG_BUFFER_MAX;

                mvuzimgxx::zimage_buffer_const srcBufSmallBBX;
                srcBufSmallBBX.plane[0].data = SmallBB->VXSmallY;
                srcBufSmallBBX.plane[0].stride = SmallBB->pitchVSmallY;
                srcBufSmallBBX.plane[0].mask = ZIMG_BUFFER_MAX;

                mvuzimgxx::zimage_buffer_const srcBufSmallBBY;
                srcBufSmallBBY.plane[0].data = SmallBB->VYSmallY;
                srcBufSmallBBY.plane[0].stride = SmallBB->pitchVSmallY;
                srcBufSmallBBY.plane[0].mask = ZIMG_BUFFER_MAX;

                mvuzimgxx::zimage_buffer dstBufSmallXFF;
                dstBufSmallXFF.plane[0].data = dstTileXFF.get();
                dstBufSmallXFF.plane[0].stride = dstTileStride;
                dstBufSmallXFF.plane[0].mask = ZIMG_BUFFER_MAX;

                mvuzimgxx::zimage_buffer dstBufSmallYFF;
                dstBufSmallYFF.plane[0].data = dstTileYFF.get();
                dstBufSmallYFF.plane[0].stride = dstTileStride;
                dstBufSmallYFF.plane[0].mask = ZIMG_BUFFER_MAX;

                mvuzimgxx::zimage_buffer dstBufSmallXBB;
                dstBufSmallXBB.plane[0].data = dstTileXBB.get();
                dstBufSmallXBB.plane[0].stride = dstTileStride;
                dstBufSmallXBB.plane[0].mask = ZIMG_BUFFER_MAX;

                mvuzimgxx::zimage_buffer dstBufSmallYBB;
                dstBufSmallYBB.plane[0].data = dstTileYBB.get();
                dstBufSmallYBB.plane[0].stride = dstTileStride;
                dstBufSmallYBB.plane[0].mask = ZIMG_BUFFER_MAX;

                ptrdiff_t dstStrideY = vsapi->getStride(dst, 0);
                uint8_t *dstPtrY = vsapi->getWritePtr(dst, 0);

                for (auto &tile : d->maskResizerFull.tiles) {
                    tile.graph.process(srcBufMaskF, dstBufMaskF, tmp.get());
                    tile.graph.process(srcBufMaskB, dstBufMaskB, tmp.get());

                    tile.graph.process(srcBufSmallFX, dstBufSmallXF, tmp.get());
                    tile.graph.process(srcBufSmallFY, dstBufSmallYF, tmp.get());

                    tile.graph.process(srcBufSmallBX, dstBufSmallXB, tmp.get());
                    tile.graph.process(srcBufSmallBY, dstBufSmallYB, tmp.get());

                    tile.graph.process(srcBufSmallFFX, dstBufSmallXFF, tmp.get());
                    tile.graph.process(srcBufSmallFFY, dstBufSmallYFF, tmp.get());

                    tile.graph.process(srcBufSmallBBX, dstBufSmallXBB, tmp.get());
                    tile.graph.process(srcBufSmallBBY, dstBufSmallYBB, tmp.get());

                    FlowInterExtra<PixelType>(dstPtrY + tile.dstX + tile.dstY * dstStrideY, dstStrideY,
                         ref.GetLevel(0).planes[0], src.GetLevel(0).planes[0],
                         dstTileXB.get(), dstTileXF.get(), dstTileYB.get(), dstTileYF.get(),
                         dstTileMaskB.get(), dstTileMaskF.get(), dstTileStride,
                         tile.dstX, tile.dstY, tile.dstWidth, tile.dstHeight, time256,
                         dstTileXBB.get(), dstTileXFF.get(), dstTileYBB.get(), dstTileYFF.get());
                }

                if (d->vi.format.numPlanes == 3) {
                    AdjustSmallVectorMaskSubSampling(*SmallF, vectorsF.nBlkX, vectorsF.nBlkY, d->vi.format.subSamplingW, d->vi.format.subSamplingH);
                    AdjustSmallVectorMaskSubSampling(*SmallB, vectorsB.nBlkX, vectorsB.nBlkY, d->vi.format.subSamplingW, d->vi.format.subSamplingH);
                    AdjustSmallVectorMaskSubSampling(*SmallFF, vectorsFF.nBlkX, vectorsFF.nBlkY, d->vi.format.subSamplingW, d->vi.format.subSamplingH);
                    AdjustSmallVectorMaskSubSampling(*SmallBB, vectorsBB.nBlkX, vectorsBB.nBlkY, d->vi.format.subSamplingW, d->vi.format.subSamplingH);

                    ptrdiff_t dstStrideU = vsapi->getStride(dst, 1);
                    ptrdiff_t dstStrideV = vsapi->getStride(dst, 2);
                    uint8_t *dstPtrU = vsapi->getWritePtr(dst, 1);
                    uint8_t *dstPtrV = vsapi->getWritePtr(dst, 2);

                    for (auto &tile : (d->vi.format.subSamplingH > 0 || d->vi.format.subSamplingW > 0) ? d->maskResizerSubSampled.tiles : d->maskResizerFull.tiles) {
                        tile.graph.process(srcBufMaskF, dstBufMaskF, tmp.get());
                        tile.graph.process(srcBufMaskB, dstBufMaskB, tmp.get());

                        tile.graph.process(srcBufSmallFX, dstBufSmallXF, tmp.get());
                        tile.graph.process(srcBufSmallFY, dstBufSmallYF, tmp.get());

                        tile.graph.process(srcBufSmallBX, dstBufSmallXB, tmp.get());
                        tile.graph.process(srcBufSmallBY, dstBufSmallYB, tmp.get());

                        tile.graph.process(srcBufSmallFFX, dstBufSmallXFF, tmp.get());
                        tile.graph.process(srcBufSmallFFY, dstBufSmallYFF, tmp.get());

                        tile.graph.process(srcBufSmallBBX, dstBufSmallXBB, tmp.get());
                        tile.graph.process(srcBufSmallBBY, dstBufSmallYBB, tmp.get());

                        FlowInterExtra<PixelType>(dstPtrU + tile.dstX + tile.dstY * dstStrideU, dstStrideU,
                             ref.GetLevel(0).planes[1], src.GetLevel(0).planes[1],
                             dstTileXB.get(), dstTileXF.get(), dstTileYB.get(), dstTileYF.get(),
                             dstTileMaskB.get(), dstTileMaskF.get(), dstTileStride,
                             tile.dstX, tile.dstY, tile.dstWidth, tile.dstHeight, time256,
                             dstTileXBB.get(), dstTileXFF.get(), dstTileYBB.get(), dstTileYFF.get());

                        FlowInterExtra<PixelType>(dstPtrV + tile.dstX + tile.dstY * dstStrideV, dstStrideV,
                             ref.GetLevel(0).planes[2], src.GetLevel(0).planes[2],
                             dstTileXB.get(), dstTileXF.get(), dstTileYB.get(), dstTileYF.get(),
                             dstTileMaskB.get(), dstTileMaskF.get(), dstTileStride,
                             tile.dstX, tile.dstY, tile.dstWidth, tile.dstHeight, time256,
                             dstTileXBB.get(), dstTileXFF.get(), dstTileYBB.get(), dstTileYFF.get());
                    }
                }
            } else { // old method without extra frames
                ptrdiff_t dstStrideY = vsapi->getStride(dst, 0);
                uint8_t *dstPtrY = vsapi->getWritePtr(dst, 0);

                for (auto &tile : d->maskResizerFull.tiles) {
                    tile.graph.process(srcBufMaskF, dstBufMaskF, tmp.get());
                    tile.graph.process(srcBufMaskB, dstBufMaskB, tmp.get());

                    tile.graph.process(srcBufSmallFX, dstBufSmallXF, tmp.get());
                    tile.graph.process(srcBufSmallFY, dstBufSmallYF, tmp.get());

                    tile.graph.process(srcBufSmallBX, dstBufSmallXB, tmp.get());
                    tile.graph.process(srcBufSmallBY, dstBufSmallYB, tmp.get());

                    FlowInter<PixelType>(dstPtrY + tile.dstX + tile.dstY * dstStrideY, dstStrideY,
                         ref.GetLevel(0).planes[0], src.GetLevel(0).planes[0],
                         dstTileXB.get(), dstTileXF.get(), dstTileYB.get(), dstTileYF.get(),
                         dstTileMaskB.get(), dstTileMaskF.get(), dstTileStride,
                         tile.dstX, tile.dstY, tile.dstWidth, tile.dstHeight, time256);

                }

                if (d->vi.format.numPlanes == 3) {
                    AdjustSmallVectorMaskSubSampling(*SmallF, vectorsF.nBlkX, vectorsF.nBlkY, d->vi.format.subSamplingW, d->vi.format.subSamplingH);
                    AdjustSmallVectorMaskSubSampling(*SmallB, vectorsB.nBlkX, vectorsB.nBlkY, d->vi.format.subSamplingW, d->vi.format.subSamplingH);

                    ptrdiff_t dstStrideU = vsapi->getStride(dst, 1);
                    ptrdiff_t dstStrideV = vsapi->getStride(dst, 2);
                    uint8_t *dstPtrU = vsapi->getWritePtr(dst, 1);
                    uint8_t *dstPtrV = vsapi->getWritePtr(dst, 2);

                    for (auto &tile : (d->vi.format.subSamplingH > 0 || d->vi.format.subSamplingW > 0) ? d->maskResizerSubSampled.tiles : d->maskResizerFull.tiles) {
                        tile.graph.process(srcBufMaskF, dstBufMaskF, tmp.get());
                        tile.graph.process(srcBufMaskB, dstBufMaskB, tmp.get());

                        tile.graph.process(srcBufSmallFX, dstBufSmallXF, tmp.get());
                        tile.graph.process(srcBufSmallFY, dstBufSmallYF, tmp.get());

                        tile.graph.process(srcBufSmallBX, dstBufSmallXB, tmp.get());
                        tile.graph.process(srcBufSmallBY, dstBufSmallYB, tmp.get());

                        FlowInter<PixelType>(dstPtrU + tile.dstX + tile.dstY * dstStrideU, dstStrideU,
                                ref.GetLevel(0).planes[1], src.GetLevel(0).planes[1],
                                dstTileXB.get(), dstTileXF.get(), dstTileYB.get(), dstTileYF.get(),
                                dstTileMaskB.get(), dstTileMaskF.get(), dstTileStride,
                                tile.dstX, tile.dstY, tile.dstWidth, tile.dstHeight, time256);

                        FlowInter<PixelType>(dstPtrV + tile.dstX + tile.dstY * dstStrideV, dstStrideV,
                                ref.GetLevel(0).planes[2], src.GetLevel(0).planes[2],
                                dstTileXB.get(), dstTileXF.get(), dstTileYB.get(), dstTileYF.get(),
                                dstTileMaskB.get(), dstTileMaskF.get(), dstTileStride,
                                tile.dstX, tile.dstY, tile.dstWidth, tile.dstHeight, time256);
                    }
                }
            }

            return dst;
        } else { // poor estimation
            if (d->blend) { //let's blend src with ref frames like ConvertFPS
                const VSFrame *src = vsapi->getFrameFilter(std::min(nleft, d->oldvi->numFrames - 1), d->node, frameCtx);
                const VSFrame *ref = vsapi->getFrameFilter(std::min(nright, d->oldvi->numFrames - 1), d->node, frameCtx);
                VSFrame *dst = vsapi->newVideoFrame(&d->vi.format, d->vi.width, d->vi.height, src, core);

                // blend with time weight
                for (int plane = 0; plane < d->vi.format.numPlanes; plane++)
                    Blend<PixelType>(vsapi->getWritePtr(dst, plane), vsapi->getReadPtr(src, plane), vsapi->getReadPtr(ref, plane), vsapi->getFrameHeight(dst, plane), vsapi->getFrameWidth(dst, plane), vsapi->getStride(dst, plane), time256);

                vsapi->freeFrame(src);
                vsapi->freeFrame(ref);

                return dst;
            } else {
                return vsapi->getFrameFilter(std::min(nleft, d->oldvi->numFrames - 1), d->node, frameCtx);
            }
        }
    }

    return nullptr;
}

// FIXME, use reducerational in vshelper?
static inline void setFPS(VSVideoInfo *vi, int64_t num, int64_t den) {
    if (num <= 0 || den <= 0) {
        vi->fpsNum = 0;
        vi->fpsDen = 1;
    } else {
        int64_t x = num;
        int64_t y = den;
        while (y) {
            int64_t t = x % y;
            x = y;
            y = t;
        }
        vi->fpsNum = num / x;
        vi->fpsDen = den / x;
    }
}


static void VS_CC flowfpsCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    std::unique_ptr<FlowFPSData> d(new FlowFPSData(vsapi));
    int err;

    d->num = vsapi->mapGetInt(in, "num", 0, &err);
    if (err)
        d->num = 25;

    d->den = vsapi->mapGetInt(in, "den", 0, &err);
    if (err)
        d->den = 1;

    d->extraMask = !!vsapi->mapGetIntSaturated(in, "extramask", 0, &err);
    if (err)
        d->extraMask = true;

    d->ml = vsapi->mapGetFloatSaturated(in, "ml", 0, &err);
    if (err)
        d->ml = 100.0f;

    d->blend = !!vsapi->mapGetInt(in, "blend", 0, &err);
    if (err)
        d->blend = 1;

    d->thscd1 = vsapi->mapGetInt(in, "thscd1", 0, &err);
    if (err)
        d->thscd1 = MV_DEFAULT_SCD1;

    d->thscd2 = vsapi->mapGetIntSaturated(in, "thscd2", 0, &err);
    if (err)
        d->thscd2 = MV_DEFAULT_SCD2;

    const char *prefix = vsapi->mapGetData(in, "prefix", 0, &err);
    if (prefix)
        d->prefix = prefix;
    else
        d->prefix = DEFAULT_MVUTENSILS_PREFIX;

    d->node = vsapi->mapGetNode(in, "clip", 0, nullptr);
    d->oldvi = vsapi->getVideoInfo(d->node);
    d->vi = *d->oldvi;

    try {
        if (d->ml <= 0.0)
            throw std::runtime_error("ml must be greater than 0");

        d->super = vsapi->mapGetNode(in, "super", 0, nullptr);

        FramePyramid super(d->super, d->prefix, core, vsapi);

        d->mvbw = vsapi->mapGetNode(in, "mvbw", 0, nullptr);
        d->mvfw = vsapi->mapGetNode(in, "mvfw", 0, nullptr);

        MotionBlockPyramid vectorsFw(d->mvfw, d->prefix, core, vsapi);
        MotionBlockPyramid vectorsBw(d->mvbw, d->prefix, core, vsapi);

        vectorsFw.ScaleThSCD(d->thscd1, d->thscd2, d->vi.format.bitsPerSample);

        d->deltaFrame = vectorsFw.nDeltaFrame;

        if (!vectorsFw.IsCompatibleForAnalysis(super))
            throw std::runtime_error("wrong source or super clip frame size");

        if (!vectorsFw.IsCompatible(vectorsBw) || (vectorsBw.nDeltaFrame != -vectorsFw.nDeltaFrame) || vectorsFw.nDeltaFrame > 0 || vectorsBw.nDeltaFrame < 0)
            throw std::runtime_error("mvfw and mvbw must be compatible with each other and have opposite sign delta");

        d->maskResizerFull.Init(vectorsFw.nBlkX, vectorsFw.nBlkY, vectorsFw.nBlkSizeX, vectorsFw.nBlkSizeY, vectorsFw.nOverlapX, vectorsFw.nOverlapY,
            d->vi.width, d->vi.height);

        if (d->vi.format.subSamplingH > 0 || d->vi.format.subSamplingW > 0)
            d->maskResizerSubSampled.Init(vectorsFw.nBlkX, vectorsFw.nBlkY, vectorsFw.nBlkSizeX >> d->vi.format.subSamplingW, vectorsFw.nBlkSizeY >> d->vi.format.subSamplingH, vectorsFw.nOverlapX >> d->vi.format.subSamplingW, vectorsFw.nOverlapY >> d->vi.format.subSamplingH,
                d->vi.width >> d->vi.format.subSamplingW, d->vi.height >> d->vi.format.subSamplingH);

        if (d->vi.fpsNum == 0 || d->vi.fpsDen == 0)
            throw std::runtime_error("input clip must have known framerate");

        int64_t numeratorOld = d->vi.fpsNum;
        int64_t denominatorOld = d->vi.fpsDen;
        int64_t numerator, denominator;

        if (d->num != 0 && d->den != 0) {
            numerator = d->num;
            denominator = d->den;
        } else {
            numerator = numeratorOld * 2; // double fps by default
            denominator = denominatorOld;
        }

        //  safe for big numbers since v2.1
        d->fa = denominator * numeratorOld;
        d->fb = numerator * denominatorOld;
        int64_t fgcd = gcd(d->fa, d->fb); // general common divisor
        d->fa /= fgcd;
        d->fb /= fgcd;

        setFPS(&d->vi, numerator, denominator);

        d->vi.numFrames = (int)(d->vi.numFrames * d->fb / d->fa);
        d->vi.fpsDen = d->fa;
        d->vi.fpsNum = d->fb;
    } catch (std::runtime_error &e) {
        vsapi->mapSetError(out, ("FlowFPS: " + std::string(e.what())).c_str());
        return;
    }

    // FIXME, are mvbw and mvfw  strictspatial when extraMask is false?
    VSFilterDependency deps[4] = { 
        {d->node, rpGeneral},
        {d->super, rpGeneral},
        {d->mvbw, rpGeneral},
        {d->mvfw, rpGeneral},
    };
    vsapi->createVideoFilter(out, "FlowFPS", &d->vi, (d->vi.format.bitsPerSample == 8) ? flowfpsGetFrame<uint8_t> : flowfpsGetFrame<uint16_t>, filterFree<FlowFPSData>, fmParallel, deps, ARRAY_SIZE(deps), d.get(), core);
    d.release();

    // FIXME, why is assumefps being called here originally instead of just setting the properties directly?
}

void flowfpsRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi) {
    vspapi->registerFunction("FlowFPS",
                 "clip:vnode;"
                 "super:vnode;"
                 "mvbw:vnode;"
                 "mvfw:vnode;"
                 "num:int:opt;"
                 "den:int:opt;"
                 "extramask:int:opt;"
                 "ml:float:opt;"
                 "blend:int:opt;"
                 "thscd1:int:opt;"
                 "thscd2:int:opt;"
                 "prefix:data:opt;",
                 "clip:vnode;",
                 flowfpsCreate, nullptr, plugin);
}
