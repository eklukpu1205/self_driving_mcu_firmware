#pragma once

void control_pid_init(void);
float control_calculate_effort(float target_speed, float actual_speed);