#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__

#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <deque>
#include <list>
#include <functional>
#include <memory>

namespace scheduler {

class Rate {
public:
    explicit Rate(double frequency) : time_step_(std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1 / frequency))), begin_(std::chrono::steady_clock::now()), last_wakeup_(begin_) { };

    void sleep() {
        {
            std::scoped_lock<std::mutex> lock { presleep_hook_mutex_ };
            std::list<std::function<bool(const double)>>::iterator it { presleep_hooks_.begin() };
            while (it != presleep_hooks_.end()) {
                if (!(*it)(now())) {
                    it = presleep_hooks_.erase(it);
                    continue;
                }
                ++it;
            }
        }

        std::this_thread::sleep_until(last_wakeup_ + time_step_);
        last_wakeup_ += time_step_;

        {
            std::scoped_lock<std::mutex> lock { postsleep_hook_mutex_ };
            std::list<std::function<bool(const double)>>::iterator it { postsleep_hooks_.begin() };
            while (it != postsleep_hooks_.end()) {
                if (!(*it)(now())) {
                    it = postsleep_hooks_.erase(it);
                    continue;
                }
                ++it;
            }
        }
    }

    double now() const noexcept {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - begin_).count();
    }

    void register_presleep_hook(std::function<bool(const double)> hook_function) noexcept {
        std::scoped_lock<std::mutex> lock { presleep_hook_mutex_ };
        presleep_hooks_.emplace_back(std::move(hook_function));
    }

    void register_postsleep_hook(std::function<bool(const double)> hook_function) noexcept {
        std::scoped_lock<std::mutex> lock { postsleep_hook_mutex_ };
        postsleep_hooks_.emplace_back(std::move(hook_function));
    }

private:
    const std::chrono::steady_clock::duration time_step_;
    const std::chrono::steady_clock::time_point begin_;
    std::chrono::steady_clock::time_point last_wakeup_;

    std::list<std::function<bool(const double)>> presleep_hooks_;
    std::mutex presleep_hook_mutex_;

    std::list<std::function<bool(const double)>> postsleep_hooks_;
    std::mutex postsleep_hook_mutex_;
};

class HookHelper : public std::enable_shared_from_this<HookHelper> {
public:
    template <typename T, typename... Args>
    static std::shared_ptr<T> create(Args... args) {
        static_assert(std::is_base_of<HookHelper, T>::value, "T must derive from HookHelper");
        return std::make_shared<T>(std::forward<Args>(args)...);
    }

    void register_hooks(Rate& rate) noexcept {
        rate.register_presleep_hook(PresleepHook_ { shared_from_this() });
        rate.register_postsleep_hook(PostsleepHook_ { shared_from_this() });
    }

protected:
    HookHelper() = default;

    virtual bool on_presleep_(const double current) {
        return false;
    }

    virtual bool on_postsleep_(const double current) {
        return false;
    }

private:
    struct PresleepHook_ {
        std::shared_ptr<HookHelper> ptr;

        bool operator()(const double currnet) {
            return ptr->on_presleep_(currnet);
        }
    };

    struct PostsleepHook_ {
        std::shared_ptr<HookHelper> ptr;

        bool operator()(const double currnet) {
            return ptr->on_postsleep_(currnet);
        }
    };
};

class FrequencyMonitorHelper : public HookHelper {
public:
    explicit FrequencyMonitorHelper(const size_t history_size) : history_size_(history_size), is_alive_(true) { }

    double frequency() noexcept {
        std::scoped_lock<std::mutex> lock { history_mutex_ };
        if (history_.size() < 2)
            return 0;
        return (history_.size() - 1) / (history_.back() - history_.front());
    }

    void detach() {
        is_alive_.store(false);
    }

private:
    size_t history_size_;
    std::atomic_bool is_alive_;
    std::deque<double> history_;
    std::mutex history_mutex_;

    bool on_postsleep_(const double current) override {
        std::scoped_lock<std::mutex> lock { history_mutex_ };
        history_.emplace_back(current);
        if (history_.size() > history_size_) {
            history_.pop_front();
        }
        return is_alive_.load();
    }
};

class FrequencyMonitor {
public:
    explicit FrequencyMonitor(Rate& rate, const size_t history_size) {
        helper_ = HookHelper::create<FrequencyMonitorHelper>(history_size);
        helper_->register_hooks(rate);
    }

    ~FrequencyMonitor() {
        helper_->detach();
    }

    double operator()() {
        return helper_->frequency();
    }

private:
    std::shared_ptr<FrequencyMonitorHelper> helper_;
};

class OnceHelper : public HookHelper {
public:
    explicit OnceHelper(const double time, const std::function<void()> callable) : time_(time), callable_(std::move(callable)) { }

private:
    const double time_;
    const std::function<void()> callable_;
    std::once_flag flag_;

    bool on_presleep_(const double current) override {
        if (current > time_) {
            std::call_once(flag_, callable_);
            return false;
        }
        return true;
    }

    bool on_postsleep_(const double current) override {
        if (current > time_) {
            std::call_once(flag_, callable_);
            return false;
        }
        return true;
    }
};

class Once {
public:
    explicit Once(Rate& rate, const double time, const std::function<void()>& callable) {
        std::shared_ptr<OnceHelper> once = HookHelper::create<OnceHelper>(time, callable);
        once->register_hooks(rate);
    }
};

} // namespace scheduler

#endif // __SCHEDULER_H__
