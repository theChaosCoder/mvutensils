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
        vsapi->requestFrameFilter(VSMIN(n + off, d->vi->numFrames - 1), d->node, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        fgopInit(&fgopF, &d->mvfw_data);
        fgopInit(&fgopB, &d->mvbw_data);

        int isUsableB = 0;
        int isUsableF = 0;

        int off = -d->deltaFrame;

        if (n + off < d->vi->numFrames) {
            const VSFrame *mvF = vsapi->getFrameFilter(n + off, d->mvfw, frameCtx);
            const VSMap *mvprops = vsapi->getFramePropertiesRO(mvF);
            fgopUpdate(&fgopF, (const uint8_t *)vsapi->mapGetData(mvprops, prop_MVTools_vectors, 0, NULL));
            vsapi->freeFrame(mvF);
            isUsableF = fgopIsUsable(&fgopF, d->thscd1, d->thscd2);

            const VSFrame *mvB = vsapi->getFrameFilter(n, d->mvbw, frameCtx);
            mvprops = vsapi->getFramePropertiesRO(mvB);
            fgopUpdate(&fgopB, (const uint8_t *)vsapi->mapGetData(mvprops, prop_MVTools_vectors, 0, NULL));
            vsapi->freeFrame(mvB);
            isUsableB = fgopIsUsable(&fgopB, d->thscd1, d->thscd2);
        }

        const int nWidth = d->mvbw_data.nWidth;
        const int nHeight = d->mvbw_data.nHeight;
        const int nWidthUV = d->nWidthUV;
        const int nHeightUV = d->nHeightUV;
        const int time256 = d->time256;
        const int blend = d->blend;

        int bitsPerSample = d->vi->format.bitsPerSample;
        int bytesPerSample = d->vi->format.bytesPerSample;

        if (isUsableB && isUsableF) {
            const VSFrame *src = vsapi->getFrameFilter(n, d->finest, frameCtx);
            const VSFrame *ref = vsapi->getFrameFilter(n + off, d->finest, frameCtx); //  ref for  compensation
            VSFrame *dst = vsapi->newVideoFrame(&d->vi->format, d->vi->width, d->vi->height, src, core);

            for (int i = 0; i < d->vi->format.numPlanes; i++) {
                pDst[i] = vsapi->getWritePtr(dst, i);
                pRef[i] = vsapi->getReadPtr(ref, i);
                pSrc[i] = vsapi->getReadPtr(src, i);
                nDstPitches[i] = vsapi->getStride(dst, i);
                nRefPitches[i] = vsapi->getStride(ref, i);
                nSrcPitches[i] = vsapi->getStride(src, i);
            }

            const float ml = d->ml;
            const int xRatioUV = d->mvbw_data.xRatioUV;
            const int yRatioUV = d->mvbw_data.yRatioUV;
            const int nBlkX = d->mvbw_data.nBlkX;
            const int nBlkY = d->mvbw_data.nBlkY;
            const int nBlkSizeX = d->mvbw_data.nBlkSizeX;
            const int nBlkSizeY = d->mvbw_data.nBlkSizeY;
            const int nOverlapX = d->mvbw_data.nOverlapX;
            const int nOverlapY = d->mvbw_data.nOverlapY;
            const int nVPadding = d->mvbw_data.nVPadding;
            const int nHPadding = d->mvbw_data.nHPadding;
            const int nVPaddingUV = d->nVPaddingUV;
            const int nHPaddingUV = d->nHPaddingUV;
            const int nPel = d->mvbw_data.nPel;
            const int VPitchY = d->VPitchY;
            const int VPitchUV = d->VPitchUV;
            const int nHeightP = d->nHeightP;
            const int nHeightPUV = d->nHeightPUV;
            const int nBlkXP = d->nBlkXP;
            const int nBlkYP = d->nBlkYP;
            SimpleResize *upsizer = &d->upsizer;
            SimpleResize *upsizerUV = &d->upsizerUV;

            ptrdiff_t nOffsetY = nRefPitches[0] * nVPadding * nPel + nHPadding * bytesPerSample * nPel;
            ptrdiff_t nOffsetUV = nRefPitches[1] * nVPaddingUV * nPel + nHPaddingUV * bytesPerSample * nPel;


            int16_t *VXFullYB = (int16_t *)malloc(nHeightP * VPitchY * sizeof(int16_t));
            int16_t *VYFullYB = (int16_t *)malloc(nHeightP * VPitchY * sizeof(int16_t));
            int16_t *VXFullYF = (int16_t *)malloc(nHeightP * VPitchY * sizeof(int16_t));
            int16_t *VYFullYF = (int16_t *)malloc(nHeightP * VPitchY * sizeof(int16_t));
            int16_t *VXSmallYB = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
            int16_t *VYSmallYB = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
            int16_t *VXSmallYF = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
            int16_t *VYSmallYF = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
            uint8_t *MaskSmallB = (uint8_t *)malloc(nBlkXP * nBlkYP);
            uint8_t *MaskFullYB = (uint8_t *)malloc(nHeightP * VPitchY);
            uint8_t *MaskSmallF = (uint8_t *)malloc(nBlkXP * nBlkYP);
            uint8_t *MaskFullYF = (uint8_t *)malloc(nHeightP * VPitchY);
            int16_t *VXFullUVB = NULL;
            int16_t *VYFullUVB = NULL;
            int16_t *VXFullUVF = NULL;
            int16_t *VYFullUVF = NULL;
            int16_t *VXSmallUVB = NULL;
            int16_t *VYSmallUVB = NULL;
            int16_t *VXSmallUVF = NULL;
            int16_t *VYSmallUVF = NULL;
            uint8_t *MaskFullUVB = NULL;
            uint8_t *MaskFullUVF = NULL;


            // make  vector vx and vy small masks
            MakeVectorSmallMasks(&fgopB, nBlkX, nBlkY, VXSmallYB, nBlkXP, VYSmallYB, nBlkXP);
            MakeVectorSmallMasks(&fgopF, nBlkX, nBlkY, VXSmallYF, nBlkXP, VYSmallYF, nBlkXP);

            CheckAndPadSmallY(VXSmallYB, VYSmallYB, nBlkXP, nBlkYP, nBlkX, nBlkY);
            CheckAndPadSmallY(VXSmallYF, VYSmallYF, nBlkXP, nBlkYP, nBlkX, nBlkY);

            // analyse vectors field to detect occlusion
            //      double occNormB = (256-time256)/(256*ml);
            MakeVectorOcclusionMaskTime(&fgopB, 1, nBlkX, nBlkY, ml, 1.0, nPel, MaskSmallB, nBlkXP, (256 - time256), nBlkSizeX - nOverlapX, nBlkSizeY - nOverlapY);
            //      double occNormF = time256/(256*ml);
            MakeVectorOcclusionMaskTime(&fgopF, 0, nBlkX, nBlkY, ml, 1.0, nPel, MaskSmallF, nBlkXP, time256, nBlkSizeX - nOverlapX, nBlkSizeY - nOverlapY);

            CheckAndPadMaskSmall(MaskSmallB, nBlkXP, nBlkYP, nBlkX, nBlkY);
            CheckAndPadMaskSmall(MaskSmallF, nBlkXP, nBlkYP, nBlkX, nBlkY);

            // upsize (bilinear interpolate) vector masks to fullframe size


            upsizer->simpleResize_int16_t(upsizer, VXFullYB, VPitchY, VXSmallYB, nBlkXP, 1);
            upsizer->simpleResize_int16_t(upsizer, VYFullYB, VPitchY, VYSmallYB, nBlkXP, 0);
            upsizer->simpleResize_int16_t(upsizer, VXFullYF, VPitchY, VXSmallYF, nBlkXP, 1);
            upsizer->simpleResize_int16_t(upsizer, VYFullYF, VPitchY, VYSmallYF, nBlkXP, 0);
            upsizer->simpleResize_uint8_t(upsizer, MaskFullYB, VPitchY, MaskSmallB, nBlkXP, 0);
            upsizer->simpleResize_uint8_t(upsizer, MaskFullYF, VPitchY, MaskSmallF, nBlkXP, 0);

            if (d->vi->format.colorFamily != cfGray) {
                VXFullUVB = (int16_t *)malloc(nHeightPUV * VPitchUV * sizeof(int16_t));
                VYFullUVB = (int16_t *)malloc(nHeightPUV * VPitchUV * sizeof(int16_t));
                VXFullUVF = (int16_t *)malloc(nHeightPUV * VPitchUV * sizeof(int16_t));
                VYFullUVF = (int16_t *)malloc(nHeightPUV * VPitchUV * sizeof(int16_t));
                VXSmallUVB = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
                VYSmallUVB = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
                VXSmallUVF = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
                VYSmallUVF = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
                MaskFullUVB = (uint8_t *)malloc(nHeightPUV * VPitchUV);
                MaskFullUVF = (uint8_t *)malloc(nHeightPUV * VPitchUV);

                VectorSmallMaskYToHalfUV(VXSmallYB, nBlkXP, nBlkYP, VXSmallUVB, xRatioUV);
                VectorSmallMaskYToHalfUV(VYSmallYB, nBlkXP, nBlkYP, VYSmallUVB, yRatioUV);
                VectorSmallMaskYToHalfUV(VXSmallYF, nBlkXP, nBlkYP, VXSmallUVF, xRatioUV);
                VectorSmallMaskYToHalfUV(VYSmallYF, nBlkXP, nBlkYP, VYSmallUVF, yRatioUV);

                upsizerUV->simpleResize_int16_t(upsizerUV, VXFullUVB, VPitchUV, VXSmallUVB, nBlkXP, 1);
                upsizerUV->simpleResize_int16_t(upsizerUV, VYFullUVB, VPitchUV, VYSmallUVB, nBlkXP, 0);
                upsizerUV->simpleResize_int16_t(upsizerUV, VXFullUVF, VPitchUV, VXSmallUVF, nBlkXP, 1);
                upsizerUV->simpleResize_int16_t(upsizerUV, VYFullUVF, VPitchUV, VYSmallUVF, nBlkXP, 0);
                upsizerUV->simpleResize_uint8_t(upsizerUV, MaskFullUVB, VPitchUV, MaskSmallB, nBlkXP, 0);
                upsizerUV->simpleResize_uint8_t(upsizerUV, MaskFullUVF, VPitchUV, MaskSmallF, nBlkXP, 0);
            }


            {
                const VSFrame *mvFF = vsapi->getFrameFilter(n, d->mvfw, frameCtx);
                const VSMap *mvprops = vsapi->getFramePropertiesRO(mvFF);
                fgopUpdate(&fgopF, (const uint8_t *)vsapi->mapGetData(mvprops, prop_MVTools_vectors, 0, NULL));
                isUsableF = fgopIsUsable(&fgopF, d->thscd1, d->thscd2);
                vsapi->freeFrame(mvFF);

                const VSFrame *mvBB = vsapi->getFrameFilter(n + off, d->mvbw, frameCtx);
                mvprops = vsapi->getFramePropertiesRO(mvBB);
                fgopUpdate(&fgopB, (const uint8_t *)vsapi->mapGetData(mvprops, prop_MVTools_vectors, 0, NULL));
                isUsableB = fgopIsUsable(&fgopB, d->thscd1, d->thscd2);
                vsapi->freeFrame(mvBB);
            }


            if (isUsableF && isUsableB) {
                int16_t *VXFullYBB = (int16_t *)malloc(nHeightP * VPitchY * sizeof(int16_t));
                int16_t *VYFullYBB = (int16_t *)malloc(nHeightP * VPitchY * sizeof(int16_t));
                int16_t *VXFullYFF = (int16_t *)malloc(nHeightP * VPitchY * sizeof(int16_t));
                int16_t *VYFullYFF = (int16_t *)malloc(nHeightP * VPitchY * sizeof(int16_t));
                int16_t *VXSmallYBB = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
                int16_t *VYSmallYBB = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
                int16_t *VXSmallYFF = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
                int16_t *VYSmallYFF = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));

                // get vector mask from extra frames
                MakeVectorSmallMasks(&fgopB, nBlkX, nBlkY, VXSmallYBB, nBlkXP, VYSmallYBB, nBlkXP);
                MakeVectorSmallMasks(&fgopF, nBlkX, nBlkY, VXSmallYFF, nBlkXP, VYSmallYFF, nBlkXP);

                CheckAndPadSmallY(VXSmallYBB, VYSmallYBB, nBlkXP, nBlkYP, nBlkX, nBlkY);
                CheckAndPadSmallY(VXSmallYFF, VYSmallYFF, nBlkXP, nBlkYP, nBlkX, nBlkY);

                // upsize vectors to full frame
                upsizer->simpleResize_int16_t(upsizer, VXFullYBB, VPitchY, VXSmallYBB, nBlkXP, 1);
                upsizer->simpleResize_int16_t(upsizer, VYFullYBB, VPitchY, VYSmallYBB, nBlkXP, 0);
                upsizer->simpleResize_int16_t(upsizer, VXFullYFF, VPitchY, VXSmallYFF, nBlkXP, 1);
                upsizer->simpleResize_int16_t(upsizer, VYFullYFF, VPitchY, VYSmallYFF, nBlkXP, 0);

                d->FlowInterExtra(pDst[0], nDstPitches[0],
                                  pRef[0] + nOffsetY, pSrc[0] + nOffsetY, nRefPitches[0],
                                  VXFullYB, VXFullYF, VYFullYB, VYFullYF,
                                  MaskFullYB, MaskFullYF, VPitchY,
                                  nWidth, nHeight, time256, nPel,
                                  VXFullYBB, VXFullYFF, VYFullYBB, VYFullYFF);

                if (d->vi->format.colorFamily != cfGray) {
                    int16_t *VXFullUVFF = (int16_t *)malloc(nHeightPUV * VPitchUV * sizeof(int16_t));
                    int16_t *VXFullUVBB = (int16_t *)malloc(nHeightPUV * VPitchUV * sizeof(int16_t));
                    int16_t *VYFullUVBB = (int16_t *)malloc(nHeightPUV * VPitchUV * sizeof(int16_t));
                    int16_t *VYFullUVFF = (int16_t *)malloc(nHeightPUV * VPitchUV * sizeof(int16_t));
                    int16_t *VXSmallUVBB = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
                    int16_t *VYSmallUVBB = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
                    int16_t *VXSmallUVFF = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));
                    int16_t *VYSmallUVFF = (int16_t *)malloc(nBlkXP * nBlkYP * sizeof(int16_t));

                    VectorSmallMaskYToHalfUV(VXSmallYBB, nBlkXP, nBlkYP, VXSmallUVBB, xRatioUV);
                    VectorSmallMaskYToHalfUV(VYSmallYBB, nBlkXP, nBlkYP, VYSmallUVBB, yRatioUV);
                    VectorSmallMaskYToHalfUV(VXSmallYFF, nBlkXP, nBlkYP, VXSmallUVFF, xRatioUV);
                    VectorSmallMaskYToHalfUV(VYSmallYFF, nBlkXP, nBlkYP, VYSmallUVFF, yRatioUV);

                    upsizerUV->simpleResize_int16_t(upsizerUV, VXFullUVBB, VPitchUV, VXSmallUVBB, nBlkXP, 1);
                    upsizerUV->simpleResize_int16_t(upsizerUV, VYFullUVBB, VPitchUV, VYSmallUVBB, nBlkXP, 0);
                    upsizerUV->simpleResize_int16_t(upsizerUV, VXFullUVFF, VPitchUV, VXSmallUVFF, nBlkXP, 1);
                    upsizerUV->simpleResize_int16_t(upsizerUV, VYFullUVFF, VPitchUV, VYSmallUVFF, nBlkXP, 0);

                    d->FlowInterExtra(pDst[1], nDstPitches[1],
                                      pRef[1] + nOffsetUV, pSrc[1] + nOffsetUV, nRefPitches[1],
                                      VXFullUVB, VXFullUVF, VYFullUVB, VYFullUVF,
                                      MaskFullUVB, MaskFullUVF, VPitchUV,
                                      nWidthUV, nHeightUV, time256, nPel,
                                      VXFullUVBB, VXFullUVFF, VYFullUVBB, VYFullUVFF);
                    d->FlowInterExtra(pDst[2], nDstPitches[2],
                                      pRef[2] + nOffsetUV, pSrc[2] + nOffsetUV, nRefPitches[2],
                                      VXFullUVB, VXFullUVF, VYFullUVB, VYFullUVF,
                                      MaskFullUVB, MaskFullUVF, VPitchUV,
                                      nWidthUV, nHeightUV, time256, nPel,
                                      VXFullUVBB, VXFullUVFF, VYFullUVBB, VYFullUVFF);
                }

            } else { // bad extra frames, use old method without extra frames
                d->FlowInter(pDst[0], nDstPitches[0],
                             pRef[0] + nOffsetY, pSrc[0] + nOffsetY, nRefPitches[0],
                             VXFullYB, VXFullYF, VYFullYB, VYFullYF,
                             MaskFullYB, MaskFullYF, VPitchY,
                             nWidth, nHeight, time256, nPel);
                if (d->vi->format.colorFamily != cfGray) {
                    d->FlowInter(pDst[1], nDstPitches[1],
                                 pRef[1] + nOffsetUV, pSrc[1] + nOffsetUV, nRefPitches[1],
                                 VXFullUVB, VXFullUVF, VYFullUVB, VYFullUVF,
                                 MaskFullUVB, MaskFullUVF, VPitchUV,
                                 nWidthUV, nHeightUV, time256, nPel);
                    d->FlowInter(pDst[2], nDstPitches[2],
                                 pRef[2] + nOffsetUV, pSrc[2] + nOffsetUV, nRefPitches[2],
                                 VXFullUVB, VXFullUVF, VYFullUVB, VYFullUVF,
                                 MaskFullUVB, MaskFullUVF, VPitchUV,
                                 nWidthUV, nHeightUV, time256, nPel);
                }
            }


            return dst;
        } else {
            if (d->blend) {
                const VSFrame *src = vsapi->getFrameFilter(n, d->node, frameCtx);
                const VSFrame *ref = vsapi->getFrameFilter(VSMIN(n + off, d->vi->numFrames - 1), d->node, frameCtx);
                VSFrame *dst = vsapi->newVideoFrame(&d->vi->format, d->vi->width, d->vi->height, src, core);

                for (int plane = 0; plane < d->vi->format.numPlanes; plane++)
                    Blend<PixelType>(vsapi->getWritePtr(dst, plane), vsapi->getReadPtr(src, plane), vsapi->getReadPtr(ref, plane), vsapi->getFrameHeight(dst, plane), vsapi->getFrameWidth(dst, plane), vsapi->getStride(dst, plane), time256);

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

void mvflowinterRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi) {
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
