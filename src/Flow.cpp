// Pixels flow motion function
// Copyright(c)2005 A.G.Balakhnin aka Fizick

// See legal notice in Copying.txt for more information

// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; version 2 of the License.
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
#include <cstring>
#include <memory>
#include <algorithm>

#include <VapourSynth4.h>
#include <VSHelper4.h>

#include "Common.h"
#include "SuperPyramid.h"
#include "MotionBlockPyramid.h"

// FIXME, break this out later when more resizing bits are known

#define ZIMGXX_NAMESPACE mvuzimgxx
#include <zimg++.hpp>

class MaskTile {
public:
    // Remember to calculate shared temp space once and for all
    int dstX;
    int dstY;
    int dstWidth;
    int dstHeight;
    mvuzimgxx::FilterGraph graph;
};

class MaskResizer {
public:
    constexpr static int TileSize = 64;
    std::vector<MaskTile> tiles;
    size_t tmpSize = 0;

    void Init(int nBlkX, int nBlkY, int nBlkSizeX, int nBlkSizeY, int nOverlapX, int nOverlapY, int dstWidth, int dstHeight) {
        int nWidth_B = (nBlkSizeX - nOverlapX) * nBlkX + nOverlapX;
        int nHeight_B = (nBlkSizeY - nOverlapY) * nBlkY + nOverlapY;
        int nWidthTiles = (dstWidth + TileSize - 1) / TileSize;
        int nHeightTiles = (dstHeight + TileSize - 1) / TileSize;
        tiles.reserve(nWidthTiles * nHeightTiles);

        const double srcWidthScale = static_cast<double>(nBlkX) / nWidth_B;
        const double srcHeightScale = static_cast<double>(nBlkY) / nHeight_B;

        mvuzimgxx::zfilter_graph_builder_params params;
        params.resample_filter = ZIMG_RESIZE_BILINEAR;
        params.cpu_type = ZIMG_CPU_AUTO_64B;

        for (int y = 0; y < nHeightTiles; y++) {
            for (int x = 0; x < nWidthTiles; x++) {
                MaskTile tile;
                tile.dstX = x * TileSize;
                tile.dstY = y * TileSize;
                tile.dstWidth = std::min(TileSize, dstWidth - tile.dstX);
                tile.dstHeight = std::min(TileSize, dstHeight - tile.dstY);

                mvuzimgxx::zimage_format srcFmt;
                srcFmt.width = nBlkX;
                srcFmt.height = nBlkY;
                srcFmt.pixel_type = ZIMG_PIXEL_WORD;
                srcFmt.color_family = ZIMG_COLOR_GREY;
                srcFmt.pixel_range = ZIMG_RANGE_FULL;
                srcFmt.active_region.left = tile.dstX * srcWidthScale;
                srcFmt.active_region.top = tile.dstY * srcHeightScale;
                srcFmt.active_region.width = tile.dstWidth * srcWidthScale;
                srcFmt.active_region.height = tile.dstHeight * srcHeightScale;

                mvuzimgxx::zimage_format dstFmt;
                dstFmt.width = tile.dstWidth;
                dstFmt.height = tile.dstHeight;
                dstFmt.pixel_type = ZIMG_PIXEL_WORD;
                dstFmt.color_family = ZIMG_COLOR_GREY;
                dstFmt.pixel_range = ZIMG_RANGE_FULL;
                try {
                    tile.graph = mvuzimgxx::FilterGraph::build(srcFmt, dstFmt, &params);
                } catch (mvuzimgxx::zerror &e) {
                    throw std::runtime_error(std::string("Error building filter graph for mask tile: ") + e.msg);
                }
                tmpSize = std::max(tmpSize, tile.graph.get_tmp_size());
                tiles.push_back(std::move(tile));
            }
        }
    }
};

static void AdjustSmallVectorMaskSubSampling(SmallVectorMasks &masks, int nBlkX, int nBlkY, int subSamplingW, int subSamplingH) {
    ptrdiff_t pitch = masks.pitchVSmallY / sizeof(uint16_t);

    if (subSamplingW > 0) {
        uint16_t *VXSmallY = masks.VXSmallY;
        for (int by = 0; by < nBlkY; by++) {
            for (int bx = 0; bx < nBlkX; bx++)
                VXSmallY[bx] = (static_cast<int16_t>(VXSmallY[bx] - (1 << 15)) >> subSamplingW) + (1 << 15);
            VXSmallY += pitch;
        }
    }

    if (subSamplingH > 0) {
        uint16_t *VYSmallY = masks.VXSmallY;
        for (int by = 0; by < nBlkY; by++) {
            for (int bx = 0; bx < nBlkX; bx++)
                VYSmallY[bx] = (static_cast<int16_t>(VYSmallY[bx] - (1 << 15)) >> subSamplingH) + (1 << 15);
            VYSmallY += pitch;
        }
    }
}

struct FlowData {
    VSNode *clip;
    VSNode *super;
    VSNode *vectors;
    const VSVideoInfo *vi;
    
    int deltaFrame;
    int time256;
    int fields;
    int64_t thscd1;
    int thscd2;
    bool tff;
    bool tff_exists;

    int pixel_max;

    MaskResizer maskResizerFull;
    MaskResizer maskResizerSubSampled;

    std::string prefix;

    const VSAPI *vsapi;

    FlowData(const VSAPI *vsapi) : vsapi(vsapi) {};

    ~FlowData() {
        vsapi->freeNode(clip);
        vsapi->freeNode(super);
        vsapi->freeNode(vectors);
    }
};


template <typename PixelType>
static void flowFetch(uint8_t *VS_RESTRICT pdst8, ptrdiff_t dst_pitch, const PyramidPlane &pref, const uint16_t *VS_RESTRICT VXFull, const uint16_t *VS_RESTRICT VYFull, ptrdiff_t tilePitch, int dstX, int dstY, int width, int height, int time256) {
    PixelType *pdst = (PixelType *)pdst8;

    dst_pitch /= sizeof(PixelType);
    tilePitch /= sizeof(int16_t);
    int nPelLog = ilog2(pref.nPel);

    // fetch mode
    for (int h = 0; h < height; h++) {
        for (int w = 0; w < width; w++) {
            // use interpolated image
            int vx = ((int(VXFull[w]) - (1 << 15)) * time256 + 128) >> 8;
            int vy = ((int(VYFull[w]) - (1 << 15)) * time256 + 128) >> 8;
            // FIXME, maybe template this on npel as well for speed?
            // FIXME, should shift w and h by ilog2(npel) to have 
            //pdst[w] = *reinterpret_cast<const PixelType *>(pref.GetPointer<PixelType>(((dstX + w) << nPelLog) + vx, ((dstY + h) << nPelLog) + vy));
        
            int pelX = ((dstX + w) << nPelLog) + vx;
            int pelY = ((dstY + h) << nPelLog) + vy;

            pelX = std::clamp(pelX, -pref.nHPaddingPel, (pref.nWidth + pref.nHPadding) * pref.nPel - 1);
            pelY = std::clamp(pelY, -pref.nVPaddingPel, (pref.nHeight + pref.nVPadding) * pref.nPel - 1);

            pdst[w] = *reinterpret_cast<const PixelType *>(pref.GetPointer<PixelType>(pelX, pelY));
        }
        pdst += dst_pitch;
        VXFull += tilePitch;
        VYFull += tilePitch;
    }
}


typedef uint8_t PixelType;

static const VSFrame *VS_CC flowGetFrame(int n, int activationReason, void *instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    FlowData *d =  reinterpret_cast<FlowData *>(instanceData);

    int nref = n + d->deltaFrame;

    if (activationReason == arInitial) {
        vsapi->requestFrameFilter(n, d->clip, frameCtx);
        vsapi->requestFrameFilter(n, d->vectors, frameCtx);

        if (nref >= 0 && nref < d->vi->numFrames) {
            if (n < nref) {
                vsapi->requestFrameFilter(n, d->super, frameCtx);
                vsapi->requestFrameFilter(nref, d->super, frameCtx);
            } else {
                vsapi->requestFrameFilter(nref, d->super, frameCtx);
                vsapi->requestFrameFilter(n, d->super, frameCtx);
            }
        }
    } else if (activationReason == arAllFramesReady) {
        const VSFrame *mvn = vsapi->getFrameFilter(n, d->vectors, frameCtx);
        MotionBlockPyramid vectors(mvn, 1, d->prefix, core, vsapi);
        vsapi->freeFrame(mvn);

        if (vectors.IsUsable(d->thscd1, d->thscd2)) {
            const VSFrame *ref = vsapi->getFrameFilter(nref, d->super, frameCtx);
            FramePyramid refGOF(ref, 1, d->prefix, core, vsapi);

            const VSFrame *propSrc = vsapi->getFrameFilter(n, d->clip, frameCtx);
            VSFrame *dst = vsapi->newVideoFrame(&d->vi->format, d->vi->width, d->vi->height, propSrc, core);
            vsapi->freeFrame(propSrc);


            // FIXME, this field shift and combined tff logic is a even more of a mess than usueal
            int fieldShift = 0;
            if (d->fields && vectors.nPel > 1 && ((nref - n) % 2 != 0)) {
                const VSFrame *src = vsapi->getFrameFilter(n, d->super, frameCtx);

                int err;
                const VSMap *props = vsapi->getFramePropertiesRO(src);
                int src_top_field = !!vsapi->mapGetInt(props, "_Field", 0, &err);
                vsapi->freeFrame(src);
                if (err && !d->tff_exists)
                    throw std::runtime_error("_Field property not found in super frame. Therefore, you must pass tff argument");

                if (d->tff_exists)
                    src_top_field = d->tff ^ (n % 2);

                props = vsapi->getFramePropertiesRO(ref);
                int ref_top_field = !!vsapi->mapGetInt(props, "_Field", 0, &err);
                if (err && !d->tff_exists) 
                    throw std::runtime_error("_Field property not found in super frame. Therefore, you must pass tff argument");

                if (d->tff_exists)
                    ref_top_field = d->tff ^ (nref % 2);

                fieldShift = (src_top_field && !ref_top_field) ? vectors.nPel / 2 : ((ref_top_field && !src_top_field) ? -(vectors.nPel / 2) : 0);
                // vertical shift of fields for fieldbased video at finest level pel2
            }

            auto smallMasks = vectors.MakeSmallVectorMasks(fieldShift);

            std::unique_ptr<void, decltype(&vsh::vsh_aligned_free)> tmp{
                vsh::vsh_aligned_malloc(std::max(d->maskResizerFull.tmpSize, d->maskResizerSubSampled.tmpSize), 64),
                vsh::vsh_aligned_free
            };

            ptrdiff_t dstTileStride = roundUpTo64(d->maskResizerFull.TileSize * sizeof(uint16_t));

            std::unique_ptr<uint16_t, decltype(&vsh::vsh_aligned_free)> dstTileVX{
                vsh::vsh_aligned_malloc<uint16_t>(dstTileStride * d->maskResizerFull.TileSize, 64),
                vsh::vsh_aligned_free
            };

            std::unique_ptr<uint16_t, decltype(&vsh::vsh_aligned_free)> dstTileVY{
                vsh::vsh_aligned_malloc<uint16_t>(dstTileStride * d->maskResizerFull.TileSize, 64),
                vsh::vsh_aligned_free
            };

            mvuzimgxx::zimage_buffer_const srcBufVX;
            srcBufVX.plane[0].data = smallMasks->VXSmallY;
            srcBufVX.plane[0].stride = smallMasks->pitchVSmallY;
            srcBufVX.plane[0].mask = ZIMG_BUFFER_MAX;

            mvuzimgxx::zimage_buffer_const srcBufVY;
            srcBufVY.plane[0].data = smallMasks->VYSmallY;
            srcBufVY.plane[0].stride = smallMasks->pitchVSmallY;
            srcBufVY.plane[0].mask = ZIMG_BUFFER_MAX;

            mvuzimgxx::zimage_buffer dstBufVX;
            dstBufVX.plane[0].data = dstTileVX.get();
            dstBufVX.plane[0].stride = dstTileStride;
            dstBufVX.plane[0].mask = ZIMG_BUFFER_MAX;

            mvuzimgxx::zimage_buffer dstBufVY;
            dstBufVY.plane[0].data = dstTileVY.get();
            dstBufVY.plane[0].stride = dstTileStride;
            dstBufVY.plane[0].mask = ZIMG_BUFFER_MAX;

            ptrdiff_t dstStrideY = vsapi->getStride(dst, 0);
            uint8_t *dstPtrY = vsapi->getWritePtr(dst, 0);

            for (auto &tile : d->maskResizerFull.tiles) {
                tile.graph.process(srcBufVX, dstBufVX, tmp.get());
                tile.graph.process(srcBufVY, dstBufVY, tmp.get());

                flowFetch<PixelType>(dstPtrY + tile.dstX + tile.dstY * dstStrideY, dstStrideY, refGOF.GetLevel(0).planes[0],
                    dstTileVX.get(), dstTileVY.get(), dstTileStride,
                    tile.dstX, tile.dstY, tile.dstWidth, tile.dstHeight, d->time256);
            }

            if (d->vi->format.numPlanes == 3) {
                AdjustSmallVectorMaskSubSampling(*smallMasks, vectors.nBlkX, vectors.nBlkY, d->vi->format.subSamplingW, d->vi->format.subSamplingH);

                ptrdiff_t dstStrideU = vsapi->getStride(dst, 1);
                ptrdiff_t dstStrideV = vsapi->getStride(dst, 2);
                uint8_t *dstPtrU = vsapi->getWritePtr(dst, 1);
                uint8_t *dstPtrV = vsapi->getWritePtr(dst, 2);

                for (auto &tile : (d->vi->format.subSamplingH > 0 || d->vi->format.subSamplingW > 0) ? d->maskResizerSubSampled.tiles : d->maskResizerFull.tiles) {
                    tile.graph.process(srcBufVX, dstBufVX, tmp.get());
                    tile.graph.process(srcBufVY, dstBufVY, tmp.get());

                    flowFetch<PixelType>(dstPtrU + tile.dstX + tile.dstY * dstStrideU, dstStrideU, refGOF.GetLevel(0).planes[1],
                        dstTileVX.get(), dstTileVY.get(), dstTileStride,
                        tile.dstX, tile.dstY, tile.dstWidth, tile.dstHeight, d->time256);

                    flowFetch<PixelType>(dstPtrV + tile.dstX + tile.dstY * dstStrideV, dstStrideV, refGOF.GetLevel(0).planes[2],
                        dstTileVX.get(), dstTileVY.get(), dstTileStride,
                        tile.dstX, tile.dstY, tile.dstWidth, tile.dstHeight, d->time256);
                }
            }

            return dst;
        } else { // not usable
            return vsapi->getFrameFilter(n, d->clip, frameCtx);
        }
    }

    return nullptr;
}


static void VS_CC flowCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    std::unique_ptr<FlowData> d(new FlowData(vsapi));

    int err;

    double time = vsapi->mapGetFloat(in, "time", 0, &err);
    if (err)
        time = 100.0;

    d->fields = !!vsapi->mapGetInt(in, "fields", 0, &err);

    d->thscd1 = vsapi->mapGetInt(in, "thscd1", 0, &err);
    if (err)
        d->thscd1 = MV_DEFAULT_SCD1;

    d->thscd2 = vsapi->mapGetIntSaturated(in, "thscd2", 0, &err);
    if (err)
        d->thscd2 = MV_DEFAULT_SCD2;

    d->tff = !!vsapi->mapGetInt(in, "tff", 0, &err);
    d->tff_exists = !err;

    try {

        if (time < 0.0 || time > 100.0) {
            vsapi->mapSetError(out, "Flow: time must be between 0 and 100 % (inclusive).");
            return;
        }

        d->time256 = (int)(time * 256.0 / 100.0);

        const char *prefix = vsapi->mapGetData(in, "prefix", 0, &err);
        if (prefix)
            d->prefix = prefix;
        else
            d->prefix = DEFAULT_MVUTENSILS_PREFIX;

        d->super = vsapi->mapGetNode(in, "super", 0, nullptr);

        char errorMsg[ERROR_SIZE] = {};
        const VSFrame *evil = vsapi->getFrame(0, d->super, errorMsg, ERROR_SIZE);
        if (!evil)
            throw std::runtime_error("failed to retrieve first frame from super clip. Error message: " + std::string(errorMsg));

        FramePyramid super(evil, 0, d->prefix, core, vsapi);

        d->vectors = vsapi->mapGetNode(in, "vectors", 0, nullptr);

        d->clip = vsapi->mapGetNode(in, "clip", 0, nullptr);
        d->vi = vsapi->getVideoInfo(d->clip);

        if (!super.IsCompatibleWithSource(d->vi))
            throw std::runtime_error("source clip isn't compatible with super clip");

        const VSFrame *evil2 = vsapi->getFrame(0, d->vectors, errorMsg, ERROR_SIZE);
        if (!evil2)
            throw std::runtime_error("failed to retrieve first frame from vectors clip. Error message: " + std::string(errorMsg));

        MotionBlockPyramid vectors(evil2, 0, d->prefix, core, vsapi);
        vsapi->freeFrame(evil2);

        vectors.ScaleThSCD(d->thscd1, d->thscd2, d->vi->format.bitsPerSample);

        d->deltaFrame = vectors.nDeltaFrame;

        if (!vectors.IsCompatibleForAnalysis(super))
            throw std::runtime_error("wrong source or super clip frame size");

        d->maskResizerFull.Init(vectors.nBlkX, vectors.nBlkY, vectors.nBlkSizeX, vectors.nBlkSizeY, vectors.nOverlapX, vectors.nOverlapY,
            d->vi->width, d->vi->height);

        if (d->vi->format.subSamplingH > 0 || d->vi->format.subSamplingW > 0)
            d->maskResizerSubSampled.Init(vectors.nBlkX, vectors.nBlkY, vectors.nBlkSizeX >> d->vi->format.subSamplingW, vectors.nBlkSizeY >> d->vi->format.subSamplingH, vectors.nOverlapX >> d->vi->format.subSamplingW, vectors.nOverlapY >> d->vi->format.subSamplingH,
                d->vi->width >> d->vi->format.subSamplingW, d->vi->height >> d->vi->format.subSamplingH);

    } catch (std::runtime_error &e) {
        vsapi->mapSetError(out, ("Flow: " + std::string(e.what())).c_str());
        return;
    }

    VSFilterDependency deps[3] = {
        {d->clip, rpStrictSpatial},
        {d->super, rpStrictSpatial},
        {d->vectors, rpStrictSpatial},
    };

    vsapi->createVideoFilter(out, "Flow", d->vi, flowGetFrame, filterFree<FlowData>, fmParallel, deps, ARRAY_SIZE(deps), d.get(), core);
    d.release();
}

void flowRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi) {
    vspapi->registerFunction("Flow",
                 "clip:vnode;"
                 "super:vnode;"
                 "vectors:vnode;"
                 "time:float:opt;"
                 "mode:int:opt;"
                 "fields:int:opt;"
                 "thscd1:int:opt;"
                 "thscd2:int:opt;"
                 "tff:int:opt;"
                 "prefix:data:opt;",
                 "clip:vnode;",
                 flowCreate, nullptr, plugin);
}
