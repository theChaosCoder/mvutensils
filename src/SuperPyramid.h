#pragma once

#include <vector>
#include <cstdint>
#include <cstddef>
#include <VapourSynth4.h>

enum class SharpParam {
    Bilinear = 0,
    Bicubic = 1,
    Wiener = 2
};


enum class RFilterParam {
    Simple = 0,
    Triangle = 1,
    Bilinear = 2,
    Quadratic = 3,
    Cubic = 4
};

// Important note on data storage: Pyramid level 0 isn't just the original unreduced frame with padding,
// if nPel > 1 it also stores the relevant offset images in pPlane[1..3] or pPlane[1..15] for pel 2 and 4 respectively
// If an external pel clip is supplied the data at (0, 0) is discarded and the original size level 0 frame data is used instead
// The pPlane allocations are completely independent of each other and can be exported

// FIXME, note that nPel is duplicated and weird everywhere if anything it should go in the FramePyramidLevel or something

struct PyramidPlane {
    // All manipulations are done on the VSFrame storage before assigning it here
    const uint8_t *pPlane[16] = {};
    // in bytes
    ptrdiff_t nPitch = -1;

    // in bytes
    ptrdiff_t nOffsetPadding = -1;

    // This is the origingal vidth and height
    int nRealWidth = -1;
    int nRealHeight = -1;

    // This is the width and height of the plane after adding nBlkSizePadX and nBlkSizePadY
    int nWidth = -1;
    int nHeight = -1;

    int nPaddedWidth = -1;
    int nPaddedHeight = -1;

    int nHPadding = -1;
    int nVPadding = -1;

    // FIXME, why is nPel duplicated?
    int nPel = 1; // 1 on all planes except the topmost where it can be 1, 2 or 4

    //int nPaddedWidth; // nWidth + 2 * nHPadding
    //int nPaddedHeight; // nHeight + 2 * nVPadding

    //int nHPaddingPel; // nPel * nHPadding
    //int nVPaddingPel; // nPel * nVPadding

    const VSFrame *storage[16] = {};

    void CopyAndPadPlane(const VSFrame *src, int plane, int hPad, int vPad, int nBlkSizePadX, int nBlkSizePadY, VSCore *core, const VSAPI *vsapi);
    void ReducePlane(const PyramidPlane &src, int ratioUV, RFilterParam rFilter, VSCore *core, const VSAPI *vsapi);
    void GeneratePelPlanes(const VSFrame *pelFrame, int pel, SharpParam sharp, VSCore *core, const VSAPI *vsapi);
    // Only unpadded pel clips supported, this differs from original MVTools
    void SetExternalPelPlanes(const VSFrame *pelFrame, int pel, int plane, VSCore *core, const VSAPI *vsapi);

private:
    void SetExtPel2(const VSFrame *pelFrame, int plane, VSCore *core, const VSAPI *vsapi);
    void SetExtPel4(const VSFrame *pelFrame, int plane, VSCore *core, const VSAPI *vsapi);
    // This function has the ugliest implementation and internally casts away consst but nobody cares
    void PadPlaneData(int plane);
};

struct FramePyramidLevel {
    PyramidPlane planes[3] = {};
};

class FramePyramid {
private:
    std::vector<FramePyramidLevel> pyramidLevels; // 0 is the orignal padded frame, higher levels are n times reduced
    int nPel = 1; // Why is nPel stored here as well? It's trivial to get with an accessor from the top level plane

    int nWidth[3]; // The original width of the frame, including padding to reach a multiple of blksize-overlap (true input dimensions are nWidth[plane] - nBlkSizePadX
    int nHeight[3];

    int nHPad[3];
    int nVPad[3];

    int nBlkSizePadX[3]; // amount of padding added to the right and bottom of the original frame to reach a multiple of blksize-overlap
    int nBlkSizePadY[3]; // divide by ratiouv

    int xRatioUV; // Subsampling ratio for chroma planes
    int yRatioUV;

    bool chroma;

    VSCore *core;
    const VSAPI *vsapi;
public:
    FramePyramid(const VSFrame *srcFrame, int levels, int blkSizeX, int blkSizeY, int overlapX, int overlapY, int hPad, int vPad, RFilterParam rFilter, bool chroma, VSCore *core, const VSAPI *vsapi); // constructor to build from source frames

    FramePyramid(const VSFrame *srcFrame, VSCore *core, const VSAPI *vsapi); // constructor to reconstruct from frame properties
    ~FramePyramid();
    void ExportFrameData(VSFrame *dst); // Stores all levels as frame properties of the output frame, note that each used plane is stored as a separate property
};