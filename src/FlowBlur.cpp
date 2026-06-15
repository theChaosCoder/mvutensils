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

#include <cstdint>
#include <algorithm>

#include <VapourSynth4.h>
#include <VSHelper4.h>

#include "Common.h"
#include "SuperPyramid.h"
#include "MotionBlockPyramid.h"
#include "MaskResize.h"

struct FlowBlurData {
    VSNode *node;
    const VSVideoInfo *vi;
    VSNode *super;
    VSNode *mvbw;
    VSNode *mvfw;

    float blur;
    int prec;
    int64_t thscd1;
    int thscd2;

    int blur256;

    int deltaFrame;

    MaskResizer maskResizerFull;
    MaskResizer maskResizerSubSampled;

    std::string prefix;

    const VSAPI *vsapi;

    FlowBlurData(const VSAPI *vsapi) : vsapi(vsapi) {};

    ~FlowBlurData() {
        vsapi->freeNode(node);
        vsapi->freeNode(super);
        vsapi->freeNode(mvbw);
        vsapi->freeNode(mvfw);
    }
};

template<typename PixelType>
static void FlowBlur(uint8_t * VS_RESTRICT pdst8, ptrdiff_t dst_pitch, const PyramidPlane &pref,
                         const uint16_t * VS_RESTRICT VXFullB, const uint16_t *VS_RESTRICT VXFullF, const uint16_t *VS_RESTRICT VYFullB, const uint16_t *VS_RESTRICT VYFullF,
                         int tilePitch, int dstX, int dstY, int width, int height, int blur256, int prec) {
    PixelType *pdst = (PixelType *)pdst8;

    dst_pitch /= sizeof(PixelType);
    tilePitch /= sizeof(int16_t);
    int nPelLog = ilog2(pref.nPel);

    /* very slow, but precise motion blur */
    for (int h = 0; h < height; h++) {
        for (int w = 0; w < width; w++) {
            // FIXME, only accesses pel1 data, maybe add a faster function to access it
            int bluredsum = *reinterpret_cast<const PixelType *>(pref.GetPointer<PixelType>((w + dstX) << nPelLog, (h + dstY) << nPelLog));
            int vxF0 = (static_cast<int>(VXFullF[w]) - (1 << 15)) * blur256;
            int vyF0 = (static_cast<int>(VYFullF[w]) - (1 << 15)) * blur256;
            int mF = (std::max(abs(vxF0), abs(vyF0)) / prec) >> 8;
            if (mF > 0) {
                vxF0 /= mF;
                vyF0 /= mF;
                int vxF = vxF0;
                int vyF = vyF0;
                for (int i = 0; i < mF; i++) {
                    int dstF = *reinterpret_cast<const PixelType *>(pref.GetPointer<PixelType>((vxF >> 8) + ((w + dstX) << nPelLog), (vyF >> 8) + ((h + dstY) << nPelLog)));
                    bluredsum += dstF;
                    vxF += vxF0;
                    vyF += vyF0;
                }
            }
            int vxB0 = (static_cast<int>(VXFullB[w]) - (1 << 15)) * blur256;
            int vyB0 = (static_cast<int>(VYFullB[w]) - (1 << 15)) * blur256;
            int mB = (std::max(abs(vxB0), abs(vyB0)) / prec) >> 8;
            if (mB > 0) {
                vxB0 /= mB;
                vyB0 /= mB;
                int vxB = vxB0;
                int vyB = vyB0;
                for (int i = 0; i < mB; i++) {
                    int dstB = *reinterpret_cast<const PixelType *>(pref.GetPointer<PixelType>((vxB >> 8) + ((w + dstX) << nPelLog), (vyB >> 8) + ((h + dstY) << nPelLog)));
                    bluredsum += dstB;
                    vxB += vxB0;
                    vyB += vyB0;
                }
            }
            pdst[w] = bluredsum / (mF + mB + 1);
        }
        pdst += dst_pitch;
        VXFullB += tilePitch;
        VYFullB += tilePitch;
        VXFullF += tilePitch;
        VYFullF += tilePitch;
    }
}

template<typename PixelType>
static const VSFrame *VS_CC flowblurGetFrame(int n, int activationReason, void *instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    FlowBlurData *d = reinterpret_cast<FlowBlurData *>(instanceData);

    if (activationReason == arInitial) {
        int off = d->deltaFrame; // integer offset of reference frame

        if (n + off >= 0 && n - off < d->vi->numFrames) {
            vsapi->requestFrameFilter(n + off, d->mvbw, frameCtx);
            vsapi->requestFrameFilter(n - off, d->mvfw, frameCtx);
        }

        vsapi->requestFrameFilter(n, d->super, frameCtx);
        vsapi->requestFrameFilter(n, d->node, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        int off = d->deltaFrame;
        bool vectorsLoadFrame = (n + off >= 0 && n - off < d->vi->numFrames);

        MotionBlockPyramid vectorsfw(vectorsLoadFrame ? vsapi->getFrameFilter(n - off, d->mvfw, frameCtx) : nullptr, 1, d->prefix, core, vsapi);
        MotionBlockPyramid vectorsbw(vectorsLoadFrame ? vsapi->getFrameFilter(n + off, d->mvbw, frameCtx) : nullptr, 1, d->prefix, core, vsapi);

        if (vectorsfw.IsUsable(d->thscd1, d->thscd2) && vectorsbw.IsUsable(d->thscd1, d->thscd2)) {
            const VSFrame *src = vsapi->getFrameFilter(n, d->node, frameCtx);
            VSFrame *dst = vsapi->newVideoFrame(&d->vi->format, d->vi->width, d->vi->height, src, core);
            vsapi->freeFrame(src);

            try {
                const VSFrame *ref = vsapi->getFrameFilter(n, d->super, frameCtx);
                FramePyramid refGOF(ref, 1, d->prefix, core, vsapi);

                auto smallMasksFw = vectorsfw.MakeSmallVectorMasks();
                auto smallMasksBw = vectorsbw.MakeSmallVectorMasks();

                auto tmp = MaskResizer::GetTmpBuffer(std::max(d->maskResizerFull.tmpSize, d->maskResizerSubSampled.tmpSize));

                auto dstTileVXFw = MaskResizer::GetTileBuffer();
                auto dstTileVYFw = MaskResizer::GetTileBuffer();
                auto dstTileVXBw = MaskResizer::GetTileBuffer();
                auto dstTileVYBw = MaskResizer::GetTileBuffer();

                auto srcBufVXFw = MaskResizer::MakeSrcBuffer(smallMasksFw->VXSmallY, smallMasksFw->pitchVSmallY);
                auto srcBufVYFw = MaskResizer::MakeSrcBuffer(smallMasksFw->VYSmallY, smallMasksFw->pitchVSmallY);
                auto srcBufVXBw = MaskResizer::MakeSrcBuffer(smallMasksBw->VXSmallY, smallMasksBw->pitchVSmallY);
                auto srcBufVYBw = MaskResizer::MakeSrcBuffer(smallMasksBw->VYSmallY, smallMasksBw->pitchVSmallY);

                auto dstBufVXFw = MaskResizer::MakeDstBuffer(dstTileVXFw.get(), MaskResizer::GetTileBufferStride());
                auto dstBufVYFw = MaskResizer::MakeDstBuffer(dstTileVYFw.get(), MaskResizer::GetTileBufferStride());
                auto dstBufVXBw = MaskResizer::MakeDstBuffer(dstTileVXBw.get(), MaskResizer::GetTileBufferStride());
                auto dstBufVYBw = MaskResizer::MakeDstBuffer(dstTileVYBw.get(), MaskResizer::GetTileBufferStride());

                ptrdiff_t dstStrideY = vsapi->getStride(dst, 0);
                uint8_t *dstPtrY = vsapi->getWritePtr(dst, 0);

                for (auto &tile : d->maskResizerFull.tiles) {
                    tile.graph.process(srcBufVXFw, dstBufVXFw, tmp.get());
                    tile.graph.process(srcBufVYFw, dstBufVYFw, tmp.get());
                    tile.graph.process(srcBufVXBw, dstBufVXBw, tmp.get());
                    tile.graph.process(srcBufVYBw, dstBufVYBw, tmp.get());

                    FlowBlur<PixelType>(dstPtrY + tile.dstX + tile.dstY * dstStrideY, dstStrideY, refGOF.GetLevel(0).planes[0],
                             dstTileVXBw.get(), dstTileVXFw.get(), dstTileVYBw.get(), dstTileVYFw.get(), MaskResizer::GetTileBufferStride(),
                             tile.dstX, tile.dstY, tile.dstWidth, tile.dstHeight, d->blur256, d->prec);
                }

                if (d->vi->format.numPlanes == 3) {
                    AdjustSmallVectorMaskSubSampling(*smallMasksFw, vectorsfw.nBlkX, vectorsfw.nBlkY, d->vi->format.subSamplingW, d->vi->format.subSamplingH);
                    AdjustSmallVectorMaskSubSampling(*smallMasksBw, vectorsbw.nBlkX, vectorsbw.nBlkY, d->vi->format.subSamplingW, d->vi->format.subSamplingH);

                    ptrdiff_t dstStrideU = vsapi->getStride(dst, 1);
                    ptrdiff_t dstStrideV = vsapi->getStride(dst, 2);
                    uint8_t *dstPtrU = vsapi->getWritePtr(dst, 1);
                    uint8_t *dstPtrV = vsapi->getWritePtr(dst, 2);

                    for (auto &tile : (d->vi->format.subSamplingH > 0 || d->vi->format.subSamplingW > 0) ? d->maskResizerSubSampled.tiles : d->maskResizerFull.tiles) {
                        tile.graph.process(srcBufVXFw, dstBufVXFw, tmp.get());
                        tile.graph.process(srcBufVYFw, dstBufVYFw, tmp.get());
                        tile.graph.process(srcBufVXBw, dstBufVXBw, tmp.get());
                        tile.graph.process(srcBufVYBw, dstBufVYBw, tmp.get());

                        FlowBlur<PixelType>(dstPtrU + tile.dstX + tile.dstY * dstStrideU, dstStrideU, refGOF.GetLevel(0).planes[1],
                             dstTileVXBw.get(), dstTileVXFw.get(), dstTileVYBw.get(), dstTileVYFw.get(), MaskResizer::GetTileBufferStride(),
                             tile.dstX, tile.dstY, tile.dstWidth, tile.dstHeight, d->blur256, d->prec);
                        FlowBlur<PixelType>(dstPtrV + tile.dstX + tile.dstY * dstStrideV, dstStrideV, refGOF.GetLevel(0).planes[2],
                             dstTileVXBw.get(), dstTileVXFw.get(), dstTileVYBw.get(), dstTileVYFw.get(), MaskResizer::GetTileBufferStride(),
                             tile.dstX, tile.dstY, tile.dstWidth, tile.dstHeight, d->blur256, d->prec);
                    }
                }
            } catch (std::runtime_error &e) {
                vsapi->freeFrame(dst);
                vsapi->setFilterError(("FlowBlur: " + std::string(e.what())).c_str(), frameCtx);
                return nullptr;
            }

            return dst;
        } else {
            return vsapi->getFrameFilter(n, d->node, frameCtx);
        }
    }

    return nullptr;
}

static void VS_CC flowblurCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    std::unique_ptr<FlowBlurData> d(new FlowBlurData(vsapi));
    int err;

    d->blur = (float)vsapi->mapGetFloat(in, "blur", 0, &err);
    if (err)
        d->blur = 50.0f;

    d->prec = vsapi->mapGetIntSaturated(in, "prec", 0, &err);
    if (err)
        d->prec = 1;

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

        if (d->blur < 0.0f || d->blur > 200.0f)
            throw std::runtime_error("blur must be between 0 and 200");

        if (d->prec < 1)
            throw std::runtime_error("prec must be at least 1");

        d->blur256 = (int)(d->blur * 256.0f / 200.0f);

        d->super = vsapi->mapGetNode(in, "super", 0, nullptr);

        FramePyramid super(d->super, d->prefix, core, vsapi);

        d->node = vsapi->mapGetNode(in, "clip", 0, nullptr);
        d->vi = vsapi->getVideoInfo(d->node);

        if (!super.IsCompatibleWithSource(d->vi))
            throw std::runtime_error("source clip isn't compatible with super clip");

        d->mvfw = vsapi->mapGetNode(in, "mvfw", 0, nullptr);
        d->mvbw = vsapi->mapGetNode(in, "mvbw", 0, nullptr);

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
        vsapi->mapSetError(out, ("FlowBlur: " + std::string(e.what())).c_str());
        return;
    }

    VSFilterDependency deps[4] = { 
        {d->node, rpStrictSpatial}, 
        {d->super, rpStrictSpatial},
        {d->mvbw, rpGeneral}, 
        {d->mvfw, rpGeneral}, 
    };

    vsapi->createVideoFilter(out, "FlowBlur", d->vi, (d->vi->format.bitsPerSample == 8) ? flowblurGetFrame<uint8_t> : flowblurGetFrame<uint16_t>, filterFree<FlowBlurData>, fmParallel, deps, ARRAY_SIZE(deps), d.get(), core);
    d.release();
}

void flowblurRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi) {
    vspapi->registerFunction("FlowBlur",
                 "clip:vnode;"
                 "super:vnode;"
                 "mvbw:vnode;"
                 "mvfw:vnode;"
                 "blur:float:opt;"
                 "prec:int:opt;"
                 "thscd1:int:opt;"
                 "thscd2:int:opt;"
                 "prefix:data:opt;",
                 "clip:vnode;",
                 flowblurCreate, 0, plugin);
}
