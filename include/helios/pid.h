// SPDX-License-Identifier: GPL-2.0-or-later

#ifndef HELIOS_PID
#define HELIOS_PID
#include <stdbool.h>

struct pid_controller {
  double input;
  double output;
  double target;
  bool inverted;

  double output_min_limit;
  double output_max_limit;

  double time_divisor;
  double p_gain;
  double i_gain;
  double d_gain;

  double integral_sum;
  double previous_error;
  unsigned last_cycle_at;
};

void run_pid_cycle(struct pid_controller* pid, unsigned current_micros);
void reset_pid(struct pid_controller* pid);

#endif
