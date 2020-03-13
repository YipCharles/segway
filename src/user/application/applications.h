#pragma once

#include "datatypes.h"
#include "platform.h"
#include "mavlink.h"

#define RTOS_PRIORITY_HIGH		4
#define RTOS_PRIORITY_NORMAL	3
#define RTOS_PRIORITY_LOW		2
#define RTOS_PRIORITY_LOWEST	0

extern mavlink_system_t mavlink_system;
