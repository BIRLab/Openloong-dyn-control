#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__

#define WAKEUP_TIME_HISTORY_SIZE 100

#include <thread>
#include <chrono>
#include <deque>

class Rate {
public:
    Rate(double frequency) : time_step(std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1 / frequency))), begin(std::chrono::steady_clock::now()), last_wakeup(begin) { };

    void sleep() {
        std::this_thread::sleep_until(last_wakeup + time_step);
        last_wakeup += time_step;
        history.emplace_back(std::chrono::steady_clock::now());
        if (history.size() > WAKEUP_TIME_HISTORY_SIZE) {
            history.pop_front();
        }
    }

    double now() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
    }

    double frequency() {
        if (history.size() < 2)
            return 0;
        return history.size() / std::chrono::duration<double>(history.back() - history.front()).count();
    }

private:
    const std::chrono::steady_clock::duration time_step;
    const std::chrono::steady_clock::time_point begin;
    std::chrono::steady_clock::time_point last_wakeup;
    std::deque<std::chrono::steady_clock::time_point> history;
};

#endif // __SCHEDULER_H__
