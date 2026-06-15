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
#include "FlowShared.h"

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

        MotionBlockPyramid vectorsF(vectorsLoadFrame ? vsapi->getFrameFilter(nright, d->mvfw, frameCtx) : nullptr, 1, d->prefix, core, vsapi);
        MotionBlockPyramid vectorsB(vectorsLoadFrame ? vsapi->getFrameFilter(nleft, d->mvbw, frameCtx) : nullptr, 1, d->prefix, core, vsapi);

        if (vectorsB.IsUsable(d->thscd1, d->thscd2) && vectorsF.IsUsable(d->thscd1, d->thscd2)) {
            // If both are usable, that means both nleft and nright are less than oldvi->numFrames. Thus there is no need to check nleft and nright here.
            FramePyramid src(vsapi->getFrameFilter(nleft, d->super, frameCtx), 1, d->prefix, core, vsapi);
            FramePyramid ref(vsapi->getFrameFilter(nright, d->super, frameCtx), 1, d->prefix, core, vsapi);
            const VSFrame *dstPropSrc = vsapi->getFrameFilter(nleft, d->super, frameCtx);
            VSFrame *dst = vsapi->newVideoFrame(&d->vi.format, d->vi.width, d->vi.height, dstPropSrc, core);
            vsapi->freeFrame(dstPropSrc);

            auto SmallB = vectorsB.MakeSmallVectorMasks();
            auto SmallF = vectorsF.MakeSmallVectorMasks();

            auto MaskSmallB = vectorsB.MakeVectorOcclusionMask<uint16_t>(d->ml, 1.0f, (256 - time256));
            auto MaskSmallF = vectorsF.MakeVectorOcclusionMask<uint16_t>(d->ml, 1.0f, (256 - time256));

            auto tmp = MaskResizer::GetTmpBuffer(std::max(d->maskResizerFull.tmpSize, d->maskResizerSubSampled.tmpSize));

            auto [dstTileMaskF, dstTileMaskB, dstTileXF, dstTileYF, dstTileXB, dstTileYB] = MaskResizer::GetTileBuffers<6>();

            auto srcBufMaskF = MaskResizer::MakeSrcBuffer(MaskSmallF->mask, MaskSmallF->stride);
            auto srcBufMaskB = MaskResizer::MakeSrcBuffer(MaskSmallB->mask, MaskSmallB->stride);

            auto srcBufSmallFX = MaskResizer::MakeSrcBuffer(SmallF->VXSmallY, SmallF->pitchVSmallY);
            auto srcBufSmallFY = MaskResizer::MakeSrcBuffer(SmallF->VYSmallY, SmallF->pitchVSmallY);
            auto srcBufSmallBX = MaskResizer::MakeSrcBuffer(SmallB->VXSmallY, SmallB->pitchVSmallY);
            auto srcBufSmallBY = MaskResizer::MakeSrcBuffer(SmallB->VYSmallY, SmallB->pitchVSmallY);

            auto dstBufMaskF = MaskResizer::MakeDstBuffer(dstTileMaskF.get());
            auto dstBufMaskB = MaskResizer::MakeDstBuffer(dstTileMaskB.get());

            auto dstBufSmallXF = MaskResizer::MakeDstBuffer(dstTileXF.get());
            auto dstBufSmallYF = MaskResizer::MakeDstBuffer(dstTileYF.get());
            auto dstBufSmallXB = MaskResizer::MakeDstBuffer(dstTileXB.get());
            auto dstBufSmallYB = MaskResizer::MakeDstBuffer(dstTileYB.get());

            MotionBlockPyramid vectorsFF(d->extraMask ? vsapi->getFrameFilter(nleft, d->mvfw, frameCtx) : nullptr, 1, d->prefix, core, vsapi);
            MotionBlockPyramid vectorsBB(d->extraMask ? vsapi->getFrameFilter(nright, d->mvbw, frameCtx) : nullptr, 1, d->prefix, core, vsapi);

            if (d->extraMask && vectorsBB.IsUsable(d->thscd1, d->thscd2) && vectorsFF.IsUsable(d->thscd1, d->thscd2)) {
                // get vector mask from extra frames
                auto SmallBB = vectorsBB.MakeSmallVectorMasks();
                auto SmallFF = vectorsFF.MakeSmallVectorMasks();

                auto srcBufSmallFFX = MaskResizer::MakeSrcBuffer(SmallFF->VXSmallY, SmallFF->pitchVSmallY);
                auto srcBufSmallFFY = MaskResizer::MakeSrcBuffer(SmallFF->VYSmallY, SmallFF->pitchVSmallY);
                auto srcBufSmallBBX = MaskResizer::MakeSrcBuffer(SmallBB->VXSmallY, SmallBB->pitchVSmallY);
                auto srcBufSmallBBY = MaskResizer::MakeSrcBuffer(SmallBB->VYSmallY, SmallBB->pitchVSmallY);

                auto [dstTileXFF, dstTileYFF, dstTileXBB, dstTileYBB] = MaskResizer::GetTileBuffers<4>();

                auto dstBufSmallXFF = MaskResizer::MakeDstBuffer(dstTileXFF.get());
                auto dstBufSmallYFF = MaskResizer::MakeDstBuffer(dstTileYFF.get());
                auto dstBufSmallXBB = MaskResizer::MakeDstBuffer(dstTileXBB.get());
                auto dstBufSmallYBB = MaskResizer::MakeDstBuffer(dstTileYBB.get());

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
                         dstTileMaskB.get(), dstTileMaskF.get(), MaskResizer::GetTileBufferStride(),
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
                             dstTileMaskB.get(), dstTileMaskF.get(), MaskResizer::GetTileBufferStride(),
                             tile.dstX, tile.dstY, tile.dstWidth, tile.dstHeight, time256,
                             dstTileXBB.get(), dstTileXFF.get(), dstTileYBB.get(), dstTileYFF.get());

                        FlowInterExtra<PixelType>(dstPtrV + tile.dstX + tile.dstY * dstStrideV, dstStrideV,
                             ref.GetLevel(0).planes[2], src.GetLevel(0).planes[2],
                             dstTileXB.get(), dstTileXF.get(), dstTileYB.get(), dstTileYF.get(),
                             dstTileMaskB.get(), dstTileMaskF.get(), MaskResizer::GetTileBufferStride(),
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
                         dstTileMaskB.get(), dstTileMaskF.get(), MaskResizer::GetTileBufferStride(),
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
                                dstTileMaskB.get(), dstTileMaskF.get(), MaskResizer::GetTileBufferStride(),
                                tile.dstX, tile.dstY, tile.dstWidth, tile.dstHeight, time256);

                        FlowInter<PixelType>(dstPtrV + tile.dstX + tile.dstY * dstStrideV, dstStrideV,
                                ref.GetLevel(0).planes[2], src.GetLevel(0).planes[2],
                                dstTileXB.get(), dstTileXF.get(), dstTileYB.get(), dstTileYF.get(),
                                dstTileMaskB.get(), dstTileMaskF.get(), MaskResizer::GetTileBufferStride(),
                                tile.dstX, tile.dstY, tile.dstWidth, tile.dstHeight, time256);
                    }
                }
            }

            return dst;
        } else {
            if (d->blend) {
                const VSFrame *src = vsapi->getFrameFilter(std::min(nleft, d->oldvi->numFrames - 1), d->node, frameCtx);
                const VSFrame *ref = vsapi->getFrameFilter(std::min(nright, d->oldvi->numFrames - 1), d->node, frameCtx);
                VSFrame *dst = vsapi->newVideoFrame(&d->vi.format, d->vi.width, d->vi.height, src, core);

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

// FIXME, reducerational in vshelper can probably replace it but leave it for now
// general common divisor (from wikipedia)
inline static int64_t gcd(int64_t u, int64_t v) {
    int shift;

    /* GCD(0,x) := x */
    if (u == 0 || v == 0)
        return u | v;

    /* Let shift := lg K, where K is the greatest power of 2
       dividing both u and v. */
    for (shift = 0; ((u | v) & 1) == 0; ++shift) {
        u >>= 1;
        v >>= 1;
    }

    while ((u & 1) == 0)
        u >>= 1;

    /* From here on, u is always odd. */
    do {
        while ((v & 1) == 0) /* Loop X */
            v >>= 1;

        /* Now u and v are both odd, so diff(u, v) is even.
           Let u = min(u, v), v = diff(u, v)/2. */
        if (u < v) {
            v -= u;
        } else {
            int64_t diff = u - v;
            u = v;
            v = diff;
        }
        v >>= 1;
    } while (v != 0);

    return u << shift;
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

        if (!super.IsCompatibleWithSource(d->oldvi))
            throw std::runtime_error("super clip is not compatible with source clip");

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

    // FIXME, verify new fps being correctly set
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
