#ifndef BOARD_H
#define BOARD_H

#include "task_monitor.h"

/* 0 = binary STATUS for the upper controller; 1 = VOFA FireWater ASCII. */
#define BOARD_HOST_FEEDBACK_DEFAULT 0U

/* Called once after CubeMX GPIO initialization and before the scheduler starts. */
bool BoardTaskMonitorInit(TaskMonitorInstance *monitor, uint32_t now_ms);

#endif
