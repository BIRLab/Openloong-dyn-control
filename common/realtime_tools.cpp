#include "realtime_tools.h"
#include <unistd.h>
#include <sched.h>
#include <pthread.h>
#include <string.h>
#include <errno.h>
#include <iostream>
#include "cl_color.h"

namespace realtime_tools {

SchedulerPolicy get_scheduler_policy() {
    switch (sched_getscheduler(getpid())) {
    case SCHED_OTHER: return SchedulerPolicy::OTHER;
    case SCHED_FIFO:  return SchedulerPolicy::FIFO;
    case SCHED_RR:    return SchedulerPolicy::RR;
    case SCHED_BATCH: return SchedulerPolicy::BATCH;
    case SCHED_IDLE:  return SchedulerPolicy::IDLE;
    default:          return SchedulerPolicy::UNKNOWN;
    }
}

int set_scheduler_policy(SchedulerPolicy sched, int prio) {
    int policy;
    switch (sched) {
    case SchedulerPolicy::OTHER:
        policy = SCHED_OTHER;
        break;
    case SchedulerPolicy::FIFO:
        policy = SCHED_FIFO;
        break;
    case SchedulerPolicy::RR:
        policy = SCHED_RR;
        break;
    case SchedulerPolicy::BATCH:
        policy = SCHED_BATCH;
        break;
    case SchedulerPolicy::IDLE:
        policy = SCHED_IDLE;
        break;
    default:
        return EINVAL;
    }

    sched_param param{ .sched_priority = prio };
    errno = 0;
    if (sched_setscheduler(getpid(), policy, &param) != 0)
        return errno;
    return 0;
}

bool is_success(int err) {
    return err == 0;
}

std::string error_to_string(int err) {
    return strerror(err);
}

std::string scheduler_policy_to_string(SchedulerPolicy sched) {
    switch (sched) {
    case SchedulerPolicy::OTHER:
        return "SCHED_OTHER";
    case SchedulerPolicy::FIFO:
        return "SCHED_FIFO";
    case SchedulerPolicy::RR:
        return "SCHED_RR";
    case SchedulerPolicy::BATCH:
        return "SCHED_BATCH";
    case SchedulerPolicy::IDLE:
        return "SCHED_IDLE";
    default:
        return "SCHED_UNKNOWN";
    }
}

void try_set_realtime(int prio) {
    int err = set_scheduler_policy(SchedulerPolicy::RR, 20);
    if (!is_success(err))
        std::cout << CL_YELLOW << "Cannot set process (" << getpid() << ") scheduling policy (SCHED_RR): " << realtime_tools::error_to_string(err) << CL_RESET << std::endl;
    std::cout << "Current scheduling policy: " << realtime_tools::scheduler_policy_to_string(realtime_tools::get_scheduler_policy()) << std::endl;
}

void try_set_nice(int inc) {
    errno = 0;
    int nic = nice(inc);
    if (nic == -1) {
        int err = errno;
        if (err != 0) {
            std::cout << CL_YELLOW << "Cannot set nice: " << realtime_tools::error_to_string(err) << CL_RESET << std::endl;
            return;
        }
    }
    std::cout << "Current nice: " << nic << std::endl;
}

}; // realtime_tools
