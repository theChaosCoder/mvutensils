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

struct FlowInterData {
    VSNode *node;
    const VSVideoInfo *vi;

    VSNode *super;
    VSNode *mvbw;
    VSNode *mvfw;

    float time;
    float ml;
    bool blend;
    int64_t thscd1;
    int thscd2;

    int time256;

    int deltaFrame;

    MaskResizer maskResizerFull;
    MaskResizer maskResizerSubSampled;

    std::string prefix;

    const VSAPI *vsapi;

    FlowInterData(const VSAPI *vsapi) : vsapi(vsapi) {};

    ~FlowInterData() {
        vsapi->freeNode(node);
        vsapi->freeNode(super);
        vsapi->freeNode(mvbw);
        vsapi->freeNode(mvfw);
    }
};

typedef uint8_t PixelType;
static const VSFrame *VS_CC flowinterGetFrame(int n, int activationReason, void *instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    FlowInterData *d = reinterpret_cast<FlowInterData *>(instanceData);

    if (activationReason == arInitial) {
        int off = -d->deltaFrame;

        if (n + off < d->vi->numFrames) {
            vsapi->requestFrameFilter(n, d->mvfw, frameCtx);
            vsapi->requestFrameFilter(n + off, d->mvfw, frameCtx);

            vsapi->requestFrameFilter(n, d->mvbw, frameCtx);
            vsapi->requestFrameFilter(n + off, d->mvbw, frameCtx);

            vsapi->requestFrameFilter(n, d->super, frameCtx);
            vsapi->requestFrameFilter(n + off, d->super, frameCtx);
        }

        vsapi->requestFrameFilter(n, d->node, frameCtx);
        vsapi->requestFrameFilter(std::min(n + off, d->vi->numFrames - 1), d->node, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        int off = -d->deltaFrame;

        bool vectorsLoadFrame = (n + off < d->vi->numFrames);

        MotionBlockPyramid vectorsF(vectorsLoadFrame ? vsapi->getFrameFilter(n + off, d->mvfw, frameCtx) : nullptr, 1, d->prefix, core, vsapi);
        MotionBlockPyramid vectorsB(vectorsLoadFrame ? vsapi->getFrameFilter(n, d->mvbw, frameCtx) : nullptr, 1, d->prefix, core, vsapi);

        if (vectorsB.IsUsable(d->thscd1, d->thscd2) && vectorsF.IsUsable(d->thscd1, d->thscd2)) {
            FramePyramid src(vsapi->getFrameFilter(n, d->super, frameCtx), 1, d->prefix, core, vsapi);
            FramePyramid ref(vsapi->getFrameFilter(n + off, d->super, frameCtx), 1, d->prefix, core, vsapi);
            const VSFrame *dstPropSrc = vsapi->getFrameFilter(n, d->super, frameCtx);
            VSFrame *dst = vsapi->newVideoFrame(&d->vi->format, d->vi->width, d->vi->height, dstPropSrc, core);
            vsapi->freeFrame(dstPropSrc);

            auto SmallB = vectorsB.MakeSmallVectorMasks();
            auto SmallF = vectorsF.MakeSmallVectorMasks();

            auto MaskSmallB = vectorsB.MakeVectorOcclusionMask<uint16_t>(d->ml, 1.0f, (256 - d->time256));
            auto MaskSmallF = vectorsF.MakeVectorOcclusionMask<uint16_t>(d->ml, 1.0f, (256 - d->time256));

            auto tmp = MaskResizer::GetTmpBuffer(std::max(d->maskResizerFull.tmpSize, d->maskResizerSubSampled.tmpSize));

            auto dstTileMaskF = MaskResizer::GetTileBuffer();
            auto dstTileMaskB = MaskResizer::GetTileBuffer();
            auto dstTileXF = MaskResizer::GetTileBuffer();
            auto dstTileYF = MaskResizer::GetTileBuffer();
            auto dstTileXB = MaskResizer::GetTileBuffer();
            auto dstTileYB = MaskResizer::GetTileBuffer();

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

            MotionBlockPyramid vectorsFF(vsapi->getFrameFilter(n, d->mvfw, frameCtx), 1, d->prefix, core, vsapi);
            MotionBlockPyramid vectorsBB(vsapi->getFrameFilter(n + off, d->mvbw, frameCtx), 1, d->prefix, core, vsapi);

            if (vectorsBB.IsUsable(d->thscd1, d->thscd2) && vectorsFF.IsUsable(d->thscd1, d->thscd2)) {
                // get vector mask from extra frames
                auto SmallBB = vectorsBB.MakeSmallVectorMasks();
                auto SmallFF = vectorsFF.MakeSmallVectorMasks();

                auto srcBufSmallFFX = MaskResizer::MakeSrcBuffer(SmallFF->VXSmallY, SmallFF->pitchVSmallY);
                auto srcBufSmallFFY = MaskResizer::MakeSrcBuffer(SmallFF->VYSmallY, SmallFF->pitchVSmallY);
                auto srcBufSmallBBX = MaskResizer::MakeSrcBuffer(SmallBB->VXSmallY, SmallBB->pitchVSmallY);
                auto srcBufSmallBBY = MaskResizer::MakeSrcBuffer(SmallBB->VYSmallY, SmallBB->pitchVSmallY);

                auto dstTileXFF = MaskResizer::GetTileBuffer();
                auto dstTileYFF = MaskResizer::GetTileBuffer();
                auto dstTileXBB = MaskResizer::GetTileBuffer();
                auto dstTileYBB = MaskResizer::GetTileBuffer();

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
                         tile.dstX, tile.dstY, tile.dstWidth, tile.dstHeight, d->time256,
                         dstTileXBB.get(), dstTileXFF.get(), dstTileYBB.get(), dstTileYFF.get());
                }

                if (d->vi->format.numPlanes == 3) {
                    AdjustSmallVectorMaskSubSampling(*SmallF, vectorsF.nBlkX, vectorsF.nBlkY, d->vi->format.subSamplingW, d->vi->format.subSamplingH);
                    AdjustSmallVectorMaskSubSampling(*SmallB, vectorsB.nBlkX, vectorsB.nBlkY, d->vi->format.subSamplingW, d->vi->format.subSamplingH);
                    AdjustSmallVectorMaskSubSampling(*SmallFF, vectorsFF.nBlkX, vectorsFF.nBlkY, d->vi->format.subSamplingW, d->vi->format.subSamplingH);
                    AdjustSmallVectorMaskSubSampling(*SmallBB, vectorsBB.nBlkX, vectorsBB.nBlkY, d->vi->format.subSamplingW, d->vi->format.subSamplingH);

                    ptrdiff_t dstStrideU = vsapi->getStride(dst, 1);
                    ptrdiff_t dstStrideV = vsapi->getStride(dst, 2);
                    uint8_t *dstPtrU = vsapi->getWritePtr(dst, 1);
                    uint8_t *dstPtrV = vsapi->getWritePtr(dst, 2);

                    for (auto &tile : (d->vi->format.subSamplingH > 0 || d->vi->format.subSamplingW > 0) ? d->maskResizerSubSampled.tiles : d->maskResizerFull.tiles) {
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
                             tile.dstX, tile.dstY, tile.dstWidth, tile.dstHeight, d->time256,
                             dstTileXBB.get(), dstTileXFF.get(), dstTileYBB.get(), dstTileYFF.get());

                        FlowInterExtra<PixelType>(dstPtrV + tile.dstX + tile.dstY * dstStrideV, dstStrideV,
                             ref.GetLevel(0).planes[2], src.GetLevel(0).planes[2],
                             dstTileXB.get(), dstTileXF.get(), dstTileYB.get(), dstTileYF.get(),
                             dstTileMaskB.get(), dstTileMaskF.get(), MaskResizer::GetTileBufferStride(),
                             tile.dstX, tile.dstY, tile.dstWidth, tile.dstHeight, d->time256,
                             dstTileXBB.get(), dstTileXFF.get(), dstTileYBB.get(), dstTileYFF.get());
                    }
                }
            } else { // bad extra frames, use old method without extra frames
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
                         tile.dstX, tile.dstY, tile.dstWidth, tile.dstHeight, d->time256);

                }

                if (d->vi->format.numPlanes == 3) {
                    AdjustSmallVectorMaskSubSampling(*SmallF, vectorsF.nBlkX, vectorsF.nBlkY, d->vi->format.subSamplingW, d->vi->format.subSamplingH);
                    AdjustSmallVectorMaskSubSampling(*SmallB, vectorsB.nBlkX, vectorsB.nBlkY, d->vi->format.subSamplingW, d->vi->format.subSamplingH);

                    ptrdiff_t dstStrideU = vsapi->getStride(dst, 1);
                    ptrdiff_t dstStrideV = vsapi->getStride(dst, 2);
                    uint8_t *dstPtrU = vsapi->getWritePtr(dst, 1);
                    uint8_t *dstPtrV = vsapi->getWritePtr(dst, 2);

                    for (auto &tile : (d->vi->format.subSamplingH > 0 || d->vi->format.subSamplingW > 0) ? d->maskResizerSubSampled.tiles : d->maskResizerFull.tiles) {
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
                                tile.dstX, tile.dstY, tile.dstWidth, tile.dstHeight, d->time256);

                        FlowInter<PixelType>(dstPtrV + tile.dstX + tile.dstY * dstStrideV, dstStrideV,
                                ref.GetLevel(0).planes[2], src.GetLevel(0).planes[2],
                                dstTileXB.get(), dstTileXF.get(), dstTileYB.get(), dstTileYF.get(),
                                dstTileMaskB.get(), dstTileMaskF.get(), MaskResizer::GetTileBufferStride(),
                                tile.dstX, tile.dstY, tile.dstWidth, tile.dstHeight, d->time256);
                    }
                }
            }


            return dst;
        } else {
            if (d->blend) {
                const VSFrame *src = vsapi->getFrameFilter(n, d->node, frameCtx);
                const VSFrame *ref = vsapi->getFrameFilter(VSMIN(n + off, d->vi->numFrames - 1), d->node, frameCtx);
                VSFrame *dst = vsapi->newVideoFrame(&d->vi->format, d->vi->width, d->vi->height, src, core);

                for (int plane = 0; plane < d->vi->format.numPlanes; plane++)
                    Blend<PixelType>(vsapi->getWritePtr(dst, plane), vsapi->getReadPtr(src, plane), vsapi->getReadPtr(ref, plane), vsapi->getFrameHeight(dst, plane), vsapi->getFrameWidth(dst, plane), vsapi->getStride(dst, plane), d->time256);

                vsapi->freeFrame(src);
                vsapi->freeFrame(ref);

                return dst;
            } else {
                return vsapi->getFrameFilter(n, d->node, frameCtx);
            }
        }
    }

    return nullptr;
}

static void VS_CC flowinterCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    std::unique_ptr<FlowInterData> d(new FlowInterData(vsapi));
    int err;

    d->time = vsapi->mapGetFloatSaturated(in, "time", 0, &err);
    if (err)
        d->time = 50.0f;

    d->ml = vsapi->mapGetFloatSaturated(in, "ml", 0, &err);
    if (err)
        d->ml = 100.0f;

    d->blend = !!vsapi->mapGetInt(in, "blend", 0, &err);
    if (err)
        d->blend = true;

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

    try {

        if (d->time < 0.0f || d->time > 100.0f)
            throw std::runtime_error("time must be between 0 and 100%");

        if (d->ml <= 0.0f)
            throw std::runtime_error("ml must be greater than 0");

        d->time256 = (int)(d->time * 256.0f / 100.0f);

        d->node = vsapi->mapGetNode(in, "clip", 0, nullptr);
        d->vi = vsapi->getVideoInfo(d->node);

        d->super = vsapi->mapGetNode(in, "super", 0, nullptr);

        FramePyramid super(d->super, d->prefix, core, vsapi);

        if (!super.IsCompatibleWithSource(d->vi))
            throw std::runtime_error("super clip is not compatible with source clip");

        d->mvbw = vsapi->mapGetNode(in, "mvbw", 0, nullptr);
        d->mvfw = vsapi->mapGetNode(in, "mvfw", 0, nullptr);

        MotionBlockPyramid vectorsFw(d->mvfw, d->prefix, core, vsapi);
        MotionBlockPyramid vectorsBw(d->mvbw, d->prefix, core, vsapi);

        vectorsFw.ScaleThSCD(d->thscd1, d->thscd2, d->vi->format.bitsPerSample);

        d->deltaFrame = vectorsFw.nDeltaFrame;

        if (!vectorsFw.IsCompatibleForAnalysis(super))
            throw std::runtime_error("wrong source or super clip frame size");

        if (!vectorsFw.IsCompatible(vectorsBw) || (vectorsBw.nDeltaFrame != -vectorsFw.nDeltaFrame) || vectorsFw.nDeltaFrame > 0 || vectorsBw.nDeltaFrame < 0)
            throw std::runtime_error("mvfw and mvbw must be compatible with each other and have opposite sign delta");

        d->maskResizerFull.Init(vectorsFw.nBlkX, vectorsFw.nBlkY, vectorsFw.nBlkSizeX, vectorsFw.nBlkSizeY, vectorsFw.nOverlapX, vectorsFw.nOverlapY,
            d->vi->width, d->vi->height);

        if (d->vi->format.subSamplingH > 0 || d->vi->format.subSamplingW > 0)
            d->maskResizerSubSampled.Init(vectorsFw.nBlkX, vectorsFw.nBlkY, vectorsFw.nBlkSizeX >> d->vi->format.subSamplingW, vectorsFw.nBlkSizeY >> d->vi->format.subSamplingH, vectorsFw.nOverlapX >> d->vi->format.subSamplingW, vectorsFw.nOverlapY >> d->vi->format.subSamplingH,
                d->vi->width >> d->vi->format.subSamplingW, d->vi->height >> d->vi->format.subSamplingH);

    } catch (std::runtime_error &e) {
        vsapi->mapSetError(out, ("FlowInter: " + std::string(e.what())).c_str());
        return;
    }

    // FIXME, check actual use
    VSFilterDependency deps[4] = { 
        {d->node, rpGeneral}, 
        {d->super, rpGeneral},
        {d->mvbw, rpGeneral}, 
        {d->mvfw, rpGeneral}, 
    };

    vsapi->createVideoFilter(out, "FlowInter", d->vi, flowinterGetFrame, filterFree<FlowInterData>, fmParallel, deps, ARRAY_SIZE(deps), d.get(), core);
    d.release();
}

void flowinterRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi) {
    vspapi->registerFunction("FlowInter",
                 "clip:vnode;"
                 "super:vnode;"
                 "mvbw:vnode;"
                 "mvfw:vnode;"
                 "time:float:opt;"
                 "ml:float:opt;"
                 "blend:int:opt;"
                 "thscd1:int:opt;"
                 "thscd2:int:opt;"
                 "prefix:data:opt;",
                 "clip:vnode;",
                 flowinterCreate, 0, plugin);
}
