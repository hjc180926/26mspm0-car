#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>
#include <stdbool.h>

void     odo_capture_start(void);
bool     odo_update_and_check(void);
uint32_t odo_get_dist_cm_x10(void);

#endif
