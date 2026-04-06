// Central build configuration for selecting MQTT topic root.
#pragma once

// Define DEBUG_BUILD in the build system (platformio.ini) to enable debug topics.
#ifdef DEBUG_BUILD
#define ADEBAR_ROOT "adebar-debug"
#define DEYE12K_ROOT "deye12k-debug"
#define ENERGYMETER_ROOT "energymeter-debug"
#else
#define ADEBAR_ROOT "adebar"
#define DEYE12K_ROOT "deye12k"
#define ENERGYMETER_ROOT "energymeter"
#endif

#define ADEBAR_TOPIC(x) ADEBAR_ROOT "/" x
#define DEYE12K_TOPIC(x) DEYE12K_ROOT "/" x
#define ENERGYMETER_TOPIC(x) ENERGYMETER_ROOT "/" x
