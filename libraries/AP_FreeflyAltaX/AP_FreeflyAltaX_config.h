#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_Freefly_Alta_X_ENABLED
#define AP_Freefly_Alta_X_ENABLED HAL_ENABLE_DRONECAN_DRIVERS  && (HAL_PROGRAM_SIZE_LIMIT_KB > 1024)
#endif
