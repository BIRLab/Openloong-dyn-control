#pragma once

#include <string>

namespace realtime_tools {

enum class SchedulerPolicy {
    OTHER,
    FIFO,
    RR,
    BATCH,
    IDLE,
    UNKNOWN
};

SchedulerPolicy get_scheduler_policy();
int set_scheduler_policy(SchedulerPolicy sched, int prio);
bool is_success(int err);
std::string error_to_string(int err);
std::string scheduler_policy_to_string(SchedulerPolicy sched);
void try_set_realtime(int prio);
void try_set_nice(int inc);

} // realtime_tools
