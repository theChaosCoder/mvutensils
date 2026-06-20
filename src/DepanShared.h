#pragma once

#include <VapourSynth4.h>

// Frame properties produced by DepanAnalyse / DepanEstimate and consumed by
// DepanCompensate / DepanStabilise.
constexpr char prop_Depan_dx[] = "Depan_dx";
constexpr char prop_Depan_dy[] = "Depan_dy";
constexpr char prop_Depan_zoom[] = "Depan_zoom";
constexpr char prop_Depan_rot[] = "Depan_rot";
constexpr char prop_Depan_goodmotion[] = "Depan_goodmotion";


struct MotionData {
    float dx = 0.0f;
    float dy = 0.0f;
    float rot = 0.0f;
    float zoom = 1.0f;
    bool badMotion = false;
};

struct transform {
    // structure of global motion transform
    //  which defines source (xsrc, ysrc)  for current destination (x,y)
    //   xsrc = dxc + dxx*x + dxy*y
    //   ysrc = dyc + dyx*x + dyy*y
    // But really only 4  parameters (dxc, dxx, dxy, dyc) are independent in used model
    float dxc;
    float dxx;
    float dxy;
    float dyc;
    float dyx;
    float dyy;

    void setNull() {
        dxc = 0.0f;
        dxx = 1.0f;
        dxy = 0.0f;
        dyc = 0.0f;
        dyx = 0.0f;
        dyy = 1.0f;
    }

    transform() {
        setNull();
    }
};


// Convert between the global-motion transform and ordinary dx/dy/rot/zoom motion.
void transform2motion(const transform *tr, bool forward, float xcenter, float ycenter, float pixaspect, float *dx, float *dy, float *rot, float *zoom);
void inversetransform(const transform *ta, transform *tinv) noexcept;
void motion2transform(float dx1, float dy1, float rot, float zoom1, float pixaspect, float xcenter, float ycenter, bool forward, float fractoffset, transform *tr);
void sumtransform(const transform *ta, const transform *tb, transform *tba);

bool mapGetMotion(MotionData &m, const VSMap *props, const VSAPI *vsapi);
void mapSetMotion(VSMap *props, const MotionData &m, const VSAPI *vsapi);

// Invoke text.FrameProps on the "clip" stored in `out` so the info property is
// rendered onto the frame. Returns false and sets the error on `out` on failure.
bool invokeFrameProps(const char *prop, VSMap *out, VSCore *core, const VSAPI *vsapi) noexcept;

// Per-filter registration, gathered by depanRegister().
void depanAnalyseRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void depanEstimateRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void depanCompensateRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void depanStabiliseRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void depanRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
