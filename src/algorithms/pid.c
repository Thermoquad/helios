// SPDX-License-Identifier: GPL-2.0-or-later

#include <helios/pid.h>

void run_pid_cycle(struct pid_controller* pid, unsigned now)
{
  // Calculate time since last cycle run and save the time for next cycle
  // Unsigned is used here to handle roll over
  const double delta_time = (double)(now - pid->last_cycle_at) / pid->time_divisor;
  pid->last_cycle_at = now;

  // Calculate error and proportional
  double error = pid->target - pid->input;
  if (pid->inverted) {
    error = error * -1.0;
  }
  const double proportional = pid->p_gain * error;

  // Calculate and clamp the integral to prevent windup
  pid->integral_sum += pid->i_gain * error * delta_time;
  if (pid->integral_sum > pid->output_max_limit * 1.25) {
    pid->integral_sum = pid->output_max_limit;
  } else if (pid->integral_sum < pid->output_min_limit * 0.75) {
    pid->integral_sum = pid->output_min_limit;
  }

  // Calculate derivative and save current error for next cycle
  const double derivative = pid->d_gain * (pid->input - pid->previous_error) / delta_time;
  pid->previous_error = error;

  // Calculate and clamp output
  pid->output = proportional + pid->integral_sum - derivative;
  if (pid->output > pid->output_max_limit) {
    pid->output = pid->output_max_limit;
  } else if (pid->output < pid->output_min_limit) {
    pid->output = pid->output_min_limit;
  }
}

void reset_pid(struct pid_controller* pid)
{
  pid->previous_error = 0.0;
  pid->last_cycle_at = 0;
  pid->integral_sum = 0.0;
  pid->output = 0.0;
  pid->input = 0.0;
}
