#include "LPM_sd_config.h"
#include "LPM_config.h"

#include <SD.h>
#include <SPI.h>

#include <Arduino.h>

namespace {

bool parseKeyValue(const String& line, String& key, String& val) {
	int eq = line.indexOf('=');
	if (eq <= 0) {
		return false;
	}
	key = line.substring(0, eq);
	key.trim();
	val = line.substring(eq + 1);
	val.trim();
	return key.length() > 0;
}

}  // namespace

void tryLoadSdConfig() {
	// Same as Teknic ReadWrite example: default CS comes from the ClearCore board package.
	if (!SD.begin()) {
		Serial.println("SD: begin failed, using compile-time defaults");
		return;
	}
	File f = SD.open("config.csv", FILE_READ);
	if (!f) {
		Serial.println("SD: config.csv not found, using defaults");
		return;
	}
	while (f.available()) {
		String line = f.readStringUntil('\n');
		line.trim();
		if (line.length() == 0 || line[0] == '#') {
			continue;
		}
		String key, val;
		if (!parseKeyValue(line, key, val)) {
			continue;
		}
		if (key.equalsIgnoreCase("pressure_min_psi")) {
			g_pressureMinPsi = val.toFloat();
		} else if (key.equalsIgnoreCase("beam1_rough_ref_counts")) {
			gBeam1RoughRefCounts = (int32_t)val.toInt();
		}
	}
	f.close();
	Serial.println("SD: loaded config.csv");
	Serial.print("  pressure_min_psi=");
	Serial.println(g_pressureMinPsi, 1);
	Serial.print("  beam1_rough_ref_counts=");
	Serial.println((long)gBeam1RoughRefCounts);
}
