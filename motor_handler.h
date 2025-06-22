#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define NUM_MOTORS 4

void motor_init(void);
void set_throttle(int motor_index, int val);

#ifdef __cplusplus
}
#endif
