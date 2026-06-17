#include <VapourSynth4.h>
#include <cstdlib>

#include "CPU.h"


// Extra indirection to keep the parameter lists with the respective filters.


void superRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void analyseRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void degrainsRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void compensateRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void recalculateRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void maskRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void flowRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void flowblurRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void flowinterRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void flowfpsRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void scdetectionRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);
void depanRegister(VSPlugin *plugin, const VSPLUGINAPI *vspapi);


uint32_t g_cpuinfo = 0;

VS_EXTERNAL_API(void)
VapourSynthPluginInit2(VSPlugin *plugin, const VSPLUGINAPI *vspapi) {
    const int packageVersion = atoi(PACKAGE_VERSION);

    vspapi->configPlugin("com.vapoursynth.mvutensils", "mvu", "MVUtensils v" PACKAGE_VERSION, VS_MAKE_VERSION(packageVersion, 0), VAPOURSYNTH_API_VERSION, 0, plugin);

    superRegister(plugin, vspapi);
    analyseRegister(plugin, vspapi);
    degrainsRegister(plugin, vspapi);
    compensateRegister(plugin, vspapi);
    recalculateRegister(plugin, vspapi);
    maskRegister(plugin, vspapi);
    flowRegister(plugin, vspapi);
    flowblurRegister(plugin, vspapi);
    flowinterRegister(plugin, vspapi);
    flowfpsRegister(plugin, vspapi);
    scdetectionRegister(plugin, vspapi);
    depanRegister(plugin, vspapi);

    g_cpuinfo = cpu_detect();
}
