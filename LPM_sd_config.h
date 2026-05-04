#pragma once

// Try to read config.csv on SD; updates g_pressureMinPsi and gBeam1RoughRefCounts when present.
void tryLoadSdConfig();
