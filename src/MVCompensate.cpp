// Make a motion compensate temporal denoiser
// Author: Manao
// Copyright(c)2006 A.G.Balakhnin aka Fizick (YUY2, overlap, edges processing)
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

#include <climits>
#include <memory>
#include <VapourSynth4.h>
#include <VSHelper4.h>

#include "CopyCode.h"
#include "Overlap.h"
//#include "MaskFun.h"
#include "CommonMacros.h"
#include "SuperPyramid.h"
#include "MotionBlockPyramid.h"



typedef struct MVCompensateData {
    VSNode *node;
    VSVideoInfo vi;
    const VSVideoInfo *supervi;

    VSNode *super;
    VSNode *vectors;

    int scBehavior;
    int64_t thSAD;
    int fields;
    int time256;
    int64_t nSCD1;
    int nSCD2;
    int tff;
    int tff_exists;
    int deltaFrame;

    bool chroma;

    int dstTempPitch;
    int dstTempPitchUV;

    OverlapWindows OverWins;
    OverlapWindows OverWinsUV;

    OverlapsFunction OVERS[3];
    COPYFunction BLIT[3];
    ToPixelsFunction ToPixels;
} MVCompensateData;


template<typename PixelType>
static const VSFrame *VS_CC mvcompensateGetFrame(int n, int activationReason, void *instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    MVCompensateData *d = (MVCompensateData *)instanceData;
    int nref = n + d->deltaFrame;

    if (activationReason == arInitial) {
        vsapi->requestFrameFilter(n, d->vectors, frameCtx);

        if (nref < n && nref >= 0)
            vsapi->requestFrameFilter(nref, d->super, frameCtx);

        vsapi->requestFrameFilter(n, d->super, frameCtx);

        if (nref >= n && nref < d->vi.numFrames)
            vsapi->requestFrameFilter(nref, d->super, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        uint8_t *pDst[3] = { NULL };
        uint8_t *pDstCur[3] = { NULL };
        const uint8_t *pRef[3] = { NULL };
        ptrdiff_t nDstPitches[3] = { 0 };
        ptrdiff_t nRefPitches[3] = { 0 };
        const uint8_t *pSrc[3] = { NULL };
        ptrdiff_t nSrcPitches[3] = { 0 };

        const VSFrame *mvn = vsapi->getFrameFilter(n, d->vectors, frameCtx);
        MotionBlockPyramid vectors(mvn, 0, "MVUtensils", core, vsapi);
        vsapi->freeFrame(mvn);

        const int xRatioUV = vectors.xRatioUV;
        const int yRatioUV = vectors.yRatioUV;
        const int ySubUV = (yRatioUV == 2) ? 1 : 0;
        const int xSubUV = (xRatioUV == 2) ? 1 : 0;
        const int nWidth[3] = { vectors.nWidth, nWidth[0] >> xSubUV, nWidth[1] };
        const int nHeight[3] = { vectors.nHeight, nHeight[0] >> ySubUV, nHeight[1] };
        const int nOverlapX[3] = { vectors.nOverlapX, nOverlapX[0] >> xSubUV, nOverlapX[1] };
        const int nOverlapY[3] = { vectors.nOverlapY, nOverlapY[0] >> ySubUV, nOverlapY[1] };
        const int nBlkSizeX[3] = { vectors.nBlkSizeX, nBlkSizeX[0] >> xSubUV, nBlkSizeX[1] };
        const int nBlkSizeY[3] = { vectors.nBlkSizeY, nBlkSizeY[0] >> ySubUV, nBlkSizeY[1] };
        const int nBlkX = vectors.nBlkX;
        const int nBlkY = vectors.nBlkY;
        const int64_t thSAD = d->thSAD;
        const int dstTempPitch[3] = { d->dstTempPitch, d->dstTempPitchUV, d->dstTempPitchUV };
        const bool chroma = d->chroma;
        const int nPel = vectors.nPel;
        const int nHPadding[3] = { vectors.nHPadding, nHPadding[0] >> xSubUV, nHPadding[1] };
        const int nVPadding[3] = { vectors.nVPadding, nVPadding[0] >> ySubUV, nVPadding[1] };
        const int scBehavior = d->scBehavior;
        const int fields = d->fields;
        const int time256 = d->time256;

        int bitsPerSample = d->supervi->format.bitsPerSample;

        int nWidth_B[3] = { nBlkX * (nBlkSizeX[0] - nOverlapX[0]) + nOverlapX[0], nWidth_B[0] >> xSubUV, nWidth_B[1] };
        int nHeight_B[3] = { nBlkY * (nBlkSizeY[0] - nOverlapY[0]) + nOverlapY[0], nHeight_B[0] >> ySubUV, nHeight_B[1] };


        int num_planes = chroma ? 3 : 1;

        const VSFrame *realSrc = vsapi->getFrameFilter(n, d->node, frameCtx);
        VSFrame *dst = vsapi->newVideoFrame(&d->vi.format, d->vi.width, d->vi.height, realSrc, core);
        vsapi->freeFrame(realSrc);
        const VSFrame *src = vsapi->getFrameFilter(n, d->super, frameCtx);
        FramePyramid pSrcGOF(src, "MVUtensils", core, vsapi);
        const auto &pSrcPlanes = pSrcGOF.GetLevel(0).planes;

        if (nref >= 0 && nref < d->vi.numFrames && vectors.IsUsable(d->nSCD1, d->nSCD2)) {
            const VSFrame *ref = vsapi->getFrameFilter(nref, d->super, frameCtx);
            FramePyramid pRefGOF(ref, "MVUtensils", core, vsapi);
            const auto &pRefPlanes = pRefGOF.GetLevel(0).planes;

            for (int i = 0; i < d->supervi->format.numPlanes; i++) {
                pDstCur[i] = pDst[i] = vsapi->getWritePtr(dst, i);
                nDstPitches[i] = vsapi->getStride(dst, i);
                pSrc[i] = pSrcGOF.GetLevel(0).planes[i].pPlane[0];
                nSrcPitches[i] = pSrcGOF.GetLevel(0).planes[i].nPitch;
                pRef[i] = pRefGOF.GetLevel(0).planes[i].pPlane[0];
                nRefPitches[i] = pRefGOF.GetLevel(0).planes[i].nPitch;
            }


            int fieldShift = 0;
            if (fields && nPel > 1 && ((nref - n) % 2 != 0)) {
                int err;
                const VSMap *props = vsapi->getFramePropertiesRO(src);
                int src_top_field = !!vsapi->mapGetInt(props, "_Field", 0, &err);
                if (err && !d->tff_exists) {
                    vsapi->setFilterError("Compensate: _Field property not found in input frame. Therefore, you must pass tff argument.", frameCtx);
                    vsapi->freeFrame(src);
                    vsapi->freeFrame(dst);
                    vsapi->freeFrame(ref);
                    return NULL;
                }

                if (d->tff_exists)
                    src_top_field = d->tff ^ (n % 2);

                props = vsapi->getFramePropertiesRO(ref);
                int ref_top_field = !!vsapi->mapGetInt(props, "_Field", 0, &err);
                if (err && !d->tff_exists) {
                    vsapi->setFilterError("Compensate: _Field property not found in input frame. Therefore, you must pass tff argument.", frameCtx);
                    vsapi->freeFrame(src);
                    vsapi->freeFrame(dst);
                    vsapi->freeFrame(ref);
                    return NULL;
                }

                if (d->tff_exists)
                    ref_top_field = d->tff ^ (nref % 2);

                fieldShift = (src_top_field && !ref_top_field) ? nPel / 2 : ((ref_top_field && !src_top_field) ? -(nPel / 2) : 0);
                // vertical shift of fields for fieldbased video at finest level pel2
            }

            if (nOverlapX[0] == 0 && nOverlapY[0] == 0) {
                // FIXME, don't write beyond destination with no overlap
                for (int by = 0; by < nBlkY; by++) {
                    int xx[3] = { 0 };

                    for (int bx = 0; bx < nBlkX; bx++) {
                        int i = by * nBlkX + bx;
                        const BlockData block = vectors.GetBlock(i);

                        int blx[3], bly[3];

                        if (block.vector.sad < thSAD) {
                            blx[0] = block.x * nPel + block.vector.x * time256 / 256;
                            bly[0] = block.y * nPel + block.vector.y * time256 / 256 + fieldShift;
                        } else {
                            blx[0] = bx * nBlkSizeX[0] * nPel;
                            bly[0] = by * nBlkSizeY[0] * nPel + fieldShift;
                        }

                        const auto &pPlanes = (block.vector.sad < thSAD) ? pRefPlanes : pSrcPlanes;

                        blx[1] = blx[2] = blx[0] >> xSubUV;
                        bly[1] = bly[2] = bly[0] >> ySubUV;

                        for (int plane = 0; plane < num_planes; plane++) {
                            d->BLIT[plane](pDstCur[plane] + xx[plane], nDstPitches[plane], pPlanes[plane].GetPointer<PixelType>(blx[plane], bly[plane]), pPlanes[plane].nPitch);

                            xx[plane] += nBlkSizeX[plane] * sizeof(PixelType);
                        }
                    }

                    for (int plane = 0; plane < num_planes; plane++)
                        pDstCur[plane] += nBlkSizeY[plane] * nDstPitches[plane];
                }
            } else { // overlap
                uint8_t *DstTemp[3] = { NULL };
                uint8_t *pDstTemp[3] = { NULL };
                for (int plane = 0; plane < num_planes; plane++) {
                    pDstTemp[plane] = DstTemp[plane] = (uint8_t *)malloc(nHeight[plane] * dstTempPitch[plane]);
                    memset(DstTemp[plane], 0, nHeight_B[plane] * dstTempPitch[plane]);
                }

                for (int by = 0; by < nBlkY; by++) {
                    int wby = ((by + nBlkY - 3) / (nBlkY - 2)) * 3;
                    int wbx = 0;
                    int xx[3] = { 0 };

                    for (int bx = 0; bx < nBlkX; bx++) {
                        // select window
                        wbx = bx == nBlkX - 1 ? 2 : wbx; //(bx + nBlkX - 3) / (nBlkX - 2);
                        const int16_t *winOver[3] = { d->OverWins.GetWindow(wby + wbx) };
                        if (chroma)
                            winOver[1] = winOver[2] = d->OverWinsUV.GetWindow(wby + wbx);

                        int i = by * nBlkX + bx;
                        const BlockData block = vectors.GetBlock(i);

                        int blx[3], bly[3];

                        if (block.vector.sad < thSAD) {
                            blx[0] = block.x * nPel + block.vector.x * time256 / 256;
                            bly[0] = block.y * nPel + block.vector.y * time256 / 256 + fieldShift;
                        } else {
                            blx[0] = bx * (nBlkSizeX[0] - nOverlapX[0]) * nPel;
                            bly[0] = by * (nBlkSizeY[0] - nOverlapY[0]) * nPel + fieldShift;
                        }

                        const auto &pPlanes = (block.vector.sad < thSAD) ? pRefPlanes : pSrcPlanes;

                        blx[1] = blx[2] = blx[0] >> xSubUV;
                        bly[1] = bly[2] = bly[0] >> ySubUV;

                        for (int plane = 0; plane < num_planes; plane++) {
                            d->OVERS[plane](pDstTemp[plane] + xx[plane] * 2, dstTempPitch[plane], pPlanes[plane].GetPointer<PixelType>(blx[plane], bly[plane]), pPlanes[plane].nPitch, winOver[plane], nBlkSizeX[plane]);

                            xx[plane] += (nBlkSizeX[plane] - nOverlapX[plane]) * sizeof(PixelType);
                        }
                        wbx = 1;
                    }

                    for (int plane = 0; plane < num_planes; plane++) {
                        pDstTemp[plane] += dstTempPitch[plane] * (nBlkSizeY[plane] - nOverlapY[plane]);
                        pDstCur[plane] += nDstPitches[plane] * (nBlkSizeY[plane] - nOverlapY[plane]);
                    }
                }

                for (int plane = 0; plane < num_planes; plane++) {
                    d->ToPixels(pDst[plane], nDstPitches[plane], DstTemp[plane], dstTempPitch[plane], std::min(vsapi->getFrameWidth(dst, plane), nWidth_B[plane]), std::min(vsapi->getFrameHeight(dst, plane), nHeight_B[plane]), bitsPerSample);
                    free(DstTemp[plane]);
                }
            }

            const uint8_t **scSrc;
            ptrdiff_t *scPitches;

            if (scBehavior) {
                scSrc = pSrc;
                scPitches = nSrcPitches;
            } else {
                scSrc = pRef;
                scPitches = nRefPitches;
            }

            for (int plane = 0; plane < num_planes; plane++) {
                if (nWidth_B[0] < nWidth[0]) { // padding of right non-covered region
                    vsh::bitblt(pDst[plane] + nWidth_B[plane] * sizeof(PixelType), nDstPitches[plane],
                              scSrc[plane] + (nWidth_B[plane] + nHPadding[plane]) * sizeof(PixelType) + nVPadding[plane] * scPitches[plane], scPitches[plane],
                              (nWidth[plane] - nWidth_B[plane]) * sizeof(PixelType), nHeight_B[plane]);
                }

                if (nHeight_B[0] < nHeight[0]) { // padding of bottom non-covered region
                    vsh::bitblt(pDst[plane] + nHeight_B[plane] * nDstPitches[plane], nDstPitches[plane],
                              scSrc[plane] + nHPadding[plane] * sizeof(PixelType) + (nHeight_B[plane] + nVPadding[plane]) * scPitches[plane], scPitches[plane],
                              nWidth[plane] * sizeof(PixelType), nHeight[plane] - nHeight_B[plane]);
                }
            }


            vsapi->freeFrame(ref);
        } else {
           // FIXME, maybe return original frame without copy
            // Copy image
            for (int plane = 0; plane < num_planes; plane++)
                vsh::bitblt(vsapi->getWritePtr(dst, plane), vsapi->getStride(dst, plane), pSrcPlanes[plane].GetPointer<PixelType>(0, 0), pSrcPlanes[plane].nPitch, pSrcPlanes[plane].nWidth * sizeof(PixelType), vsapi->getFrameHeight(dst, plane));
        }

        vsapi->freeFrame(src);

        return dst;
    }

    return nullptr;
}


static void VS_CC mvcompensateFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    MVCompensateData *d = (MVCompensateData *)instanceData;

    vsapi->freeNode(d->super);
    vsapi->freeNode(d->vectors);
    vsapi->freeNode(d->node);
    delete d;
}


static void VS_CC mvcompensateCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    std::unique_ptr<MVCompensateData> d(new MVCompensateData());
    int err;

    d->scBehavior = !!vsapi->mapGetInt(in, "scbehavior", 0, &err);
    if (err)
        d->scBehavior = 1;

    d->thSAD = vsapi->mapGetInt(in, "thsad", 0, &err);
    if (err)
        d->thSAD = 10000;

    d->fields = !!vsapi->mapGetInt(in, "fields", 0, &err);

    double time = vsapi->mapGetFloat(in, "time", 0, &err);
    if (err)
        time = 100.0;

    d->nSCD1 = vsapi->mapGetInt(in, "thscd1", 0, &err);
    if (err)
        d->nSCD1 = MV_DEFAULT_SCD1;

    d->nSCD2 = vsapi->mapGetIntSaturated(in, "thscd2", 0, &err);
    if (err)
        d->nSCD2 = MV_DEFAULT_SCD2;

    d->tff = !!vsapi->mapGetInt(in, "tff", 0, &err);
    d->tff_exists = !err;


    if (time < 0.0 || time > 100.0) {
        vsapi->mapSetError(out, "Compensate: time must be between 0.0 and 100.0 (inclusive).");
        return;
    }


    d->super = vsapi->mapGetNode(in, "super", 0, NULL);

#define ERROR_SIZE 1024
    char errorMsg[ERROR_SIZE] = "Compensate: failed to retrieve first frame from super clip. Error message: ";
    size_t errorLen = strlen(errorMsg);
    const VSFrame *evil = vsapi->getFrame(0, d->super, errorMsg + errorLen, ERROR_SIZE - (int)errorLen);
#undef ERROR_SIZE
    if (!evil) {
        vsapi->mapSetError(out, errorMsg);
        vsapi->freeNode(d->super);
        return;
    }

    FramePyramid super(evil, "MVUtensils", core, vsapi);

    // Note that this invalidates all data pointers in super
    vsapi->freeFrame(evil);

    d->vectors = vsapi->mapGetNode(in, "vectors", 0, NULL);

    d->node = vsapi->mapGetNode(in, "clip", 0, 0);
    d->vi = *vsapi->getVideoInfo(d->node);

#define ERROR_SIZE 512
    char error[ERROR_SIZE + 1] = { 0 };

    const VSFrame *evil2 = vsapi->getFrame(0, d->vectors, errorMsg + errorLen, ERROR_SIZE - (int)errorLen);
    MotionBlockPyramid vectors(evil2, 0, "MVUtensils", core, vsapi);

    int64_t nSCD1_old = d->nSCD1;
    vectors.ScaleThSCD(d->nSCD1, d->nSCD2, d->vi.format.bitsPerSample);
#undef ERROR_SIZE

    if (error[0]) {
        vsapi->mapSetError(out, error);

        vsapi->freeNode(d->super);
        vsapi->freeNode(d->vectors);
        vsapi->freeNode(d->node);
        return;
    }

    d->deltaFrame = vectors.nDeltaFrame;

    if (d->fields && vectors.nPel < 2) {
        vsapi->mapSetError(out, "Compensate: fields option requires pel > 1.");
        vsapi->freeNode(d->super);
        vsapi->freeNode(d->vectors);
        vsapi->freeNode(d->node);
        return;
    }

    d->thSAD = d->thSAD * d->nSCD1 / nSCD1_old; // normalize to block SAD

    d->dstTempPitch = ((vectors.nWidth + 15) / 16) * 16 * d->vi.format.bytesPerSample * 2;
    d->dstTempPitchUV = (((vectors.nWidth / vectors.xRatioUV) + 15) / 16) * 16 * d->vi.format.bytesPerSample * 2;

    d->supervi = vsapi->getVideoInfo(d->super);
    
    if (vectors.nHeight != super.nHeight[0] || vectors.nWidth != super.nWidth[0]  || vectors.nRealHeight != d->vi.height || vectors.nRealWidth != d->vi.width || vectors.nPel != super.nPel) {
        vsapi->mapSetError(out, "Compensate: wrong source or super clip frame size.");
        vsapi->freeNode(d->super);
        vsapi->freeNode(d->vectors);
        vsapi->freeNode(d->node);
        return;
    }

    if (!vsh::isConstantVideoFormat(&d->vi) || d->vi.format.bitsPerSample > 16 || d->vi.format.sampleType != stInteger || d->vi.format.subSamplingW > 1 || d->vi.format.subSamplingH > 1 || (d->vi.format.colorFamily != cfYUV && d->vi.format.colorFamily != cfGray)) {
        vsapi->mapSetError(out, "Compensate: input clip must be GRAY, 420, 422, 440, or 444, up to 16 bits, with constant dimensions.");
        vsapi->freeNode(d->super);
        vsapi->freeNode(d->vectors);
        vsapi->freeNode(d->node);
        return;
    }

    d->chroma = (d->vi.format.colorFamily != cfGray);

    if (vectors.nOverlapX > 0 || vectors.nOverlapY > 0) {
        d->OverWins.Init(vectors.nBlkSizeX, vectors.nBlkSizeY, vectors.nOverlapX, vectors.nOverlapY);
        if (d->chroma)
            d->OverWinsUV.Init(vectors.nBlkSizeX / vectors.xRatioUV, vectors.nBlkSizeY / vectors.yRatioUV, vectors.nOverlapX / vectors.xRatioUV, vectors.nOverlapY / vectors.yRatioUV);
    }

    const unsigned bits = d->vi.format.bytesPerSample * 8;

    if (d->vi.format.bitsPerSample == 8) {
        d->ToPixels = ToPixels<uint16_t, uint8_t>;
    } else {
        d->ToPixels = ToPixels<uint32_t, uint16_t>;
    }

    d->OVERS[0] = selectOverlapsFunction(vectors.nBlkSizeX, vectors.nBlkSizeY, bits);
    d->BLIT[0] = selectCopyFunction(vectors.nBlkSizeX, vectors.nBlkSizeY, bits);

    d->OVERS[1] = d->OVERS[2] = selectOverlapsFunction(vectors.nBlkSizeX / vectors.xRatioUV, vectors.nBlkSizeY / vectors.yRatioUV, bits);
    d->BLIT[1] = d->BLIT[2] = selectCopyFunction(vectors.nBlkSizeX / vectors.xRatioUV, vectors.nBlkSizeY / vectors.yRatioUV, bits);

    d->time256 = (int)(time * 256 / 100);


    VSFilterDependency deps[3] = { 
        {d->node, rpStrictSpatial},
        {d->super, rpStrictSpatial},
        {d->vectors, rpNoFrameReuse},
    };

    vsapi->createVideoFilter(out, "Compensate", &d->vi, d->vi.format.bytesPerSample == 1 ? mvcompensateGetFrame<uint8_t> : mvcompensateGetFrame<uint16_t>, mvcompensateFree, fmParallel, deps,  ARRAY_SIZE(deps), d.get(), core);
    d.release();
}


void mvcompensateRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi) {
    vspapi->registerFunction("Compensate",
                 "clip:vnode;"
                 "super:vnode;"
                 "vectors:vnode;"
                 "scbehavior:int:opt;"
                 "thsad:int:opt;"
                 "fields:int:opt;"
                 "time:float:opt;"
                 "thscd1:int:opt;"
                 "thscd2:int:opt;"
                 "tff:int:opt;",
                 "clip:vnode;",
                 mvcompensateCreate, 0, plugin);
}
