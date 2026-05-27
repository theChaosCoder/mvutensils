#include <VapourSynth4.h>
#include <cstdlib>

#include "CPU.h"


// Extra indirection to keep the parameter lists with the respective filters.


void superRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void analyseRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void degrainsRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void compensateRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void mvrecalculateRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void mvmaskRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void mvfinestRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void mvflowRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void mvflowblurRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void mvflowinterRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void mvflowfpsRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void mvblockfpsRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void scdetectionRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void mvdepanRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);


uint32_t g_cpuinfo = 0;

VS_EXTERNAL_API(void)
VapourSynthPluginInit2(VSPlugin *plugin, const VSPLUGINAPI *vspapi) {
    const int packageVersion = atoi(PACKAGE_VERSION);

    vspapi->configPlugin("com.vapoursynth.mvutensils", "mvu", "MVUtensils v" PACKAGE_VERSION, VS_MAKE_VERSION(packageVersion, 0), VAPOURSYNTH_API_VERSION, 0, plugin);

    superRegister(plugin, vspapi);
    analyseRegister(plugin, vspapi);
    degrainsRegister(plugin, vspapi);
    compensateRegister(plugin, vspapi);
    //mvrecalculateRegister(plugin, vspapi);
    //mvmaskRegister(plugin, vspapi);
    //mvfinestRegister(plugin, vspapi);
    //mvflowRegister(plugin, vspapi);
    //mvflowblurRegister(plugin, vspapi);
    //mvflowinterRegister(plugin, vspapi);
    //mvflowfpsRegister(plugin, vspapi);
    //mvblockfpsRegister(plugin, vspapi);
    scdetectionRegister(plugin, vspapi);
    //mvdepanRegister(plugin, vspapi);

    g_cpuinfo = cpu_detect();
}
