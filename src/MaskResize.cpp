
#include "MaskResize.h"

#include <memory>
#include <stdexcept>
#include <VSHelper4.h>

void BilinearUpsizeBlockMask(uint8_t *dst, ptrdiff_t dststride, int dstwidth, int dstheight, const void *src, ptrdiff_t srcstride, int nBlkX, int nBlkY, int nBlkSizeX, int nBlkSizeY, int nOverlapX, int nOverlapY, int bitsPerSample) {
    int nWidth_B = (nBlkSizeX - nOverlapX) * nBlkX + nOverlapX;
    int nHeight_B = (nBlkSizeY - nOverlapY) * nBlkY + nOverlapY;

    mvuzimgxx::zimage_format srcFmt;
    srcFmt.width = nBlkX;
    srcFmt.height = nBlkY;
    srcFmt.pixel_type = (bitsPerSample == 8) ? ZIMG_PIXEL_BYTE : ZIMG_PIXEL_WORD;
    srcFmt.color_family = ZIMG_COLOR_GREY;
    srcFmt.pixel_range = ZIMG_RANGE_FULL;
    srcFmt.depth = bitsPerSample;

    // Adjust active region to cut off the padding part of the edge blocks and properly scale the mask to the original frame size
    srcFmt.active_region.width = (static_cast<double>(dstwidth) / nWidth_B) * nBlkX;
    srcFmt.active_region.height = (static_cast<double>(dstheight) / nHeight_B) * nBlkY;

    mvuzimgxx::zimage_format dstFmt;
    dstFmt.width = dstwidth;
    dstFmt.height = dstheight;
    dstFmt.pixel_type = (bitsPerSample == 8) ? ZIMG_PIXEL_BYTE : ZIMG_PIXEL_WORD;
    dstFmt.color_family = ZIMG_COLOR_GREY;
    dstFmt.pixel_range = ZIMG_RANGE_FULL;
    dstFmt.depth = bitsPerSample;

    mvuzimgxx::zfilter_graph_builder_params params;
    params.resample_filter = ZIMG_RESIZE_BILINEAR;
    params.cpu_type = ZIMG_CPU_AUTO_64B;

    mvuzimgxx::FilterGraph graph = mvuzimgxx::FilterGraph::build(srcFmt, dstFmt, &params);

    std::unique_ptr<void, decltype(&vsh::vsh_aligned_free)> tmp{
        vsh::vsh_aligned_malloc(graph.get_tmp_size(), 64),
        vsh::vsh_aligned_free
    };

    mvuzimgxx::zimage_buffer_const srcBuf;
    srcBuf.plane[0].data = src;
    srcBuf.plane[0].stride = srcstride;
    srcBuf.plane[0].mask = ZIMG_BUFFER_MAX;

    mvuzimgxx::zimage_buffer dstBuf;
    dstBuf.plane[0].data = dst;
    dstBuf.plane[0].stride = dststride;
    dstBuf.plane[0].mask = ZIMG_BUFFER_MAX;

    graph.process(srcBuf, dstBuf, tmp.get());
}

void MaskResizer::Init(int nBlkX, int nBlkY, int nBlkSizeX, int nBlkSizeY, int nOverlapX, int nOverlapY, int dstWidth, int dstHeight) {
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