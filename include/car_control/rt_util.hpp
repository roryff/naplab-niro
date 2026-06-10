#pragma once

#include <rclcpp/rclcpp.hpp>
#include <pthread.h>
#include <sched.h>
#include <cstring>

/**
 * @file rt_util.hpp
 * @brief Helpers to promote threads to real-time (SCHED_FIFO) scheduling.
 *
 * Why: under heavy load (lidar + cameras + DDS) the default SCHED_OTHER
 * scheduler can starve a node's time-critical threads for tens to hundreds of
 * milliseconds. That is exactly what stalls the comma link (receive thread
 * scheduled late -> TCP back-pressure) and trips the comma's 500 ms joystick
 * watchdog (send thread scheduled late). Putting the control/IO threads on
 * SCHED_FIFO lets them preempt the throughput work and hold their cadence.
 *
 * Requirements (no sudo): the process needs RLIMIT_RTPRIO >= requested priority
 * (or CAP_SYS_NICE). In this workspace that is granted at container creation via
 * ~/.isaac_ros_dev-dockerargs (`--ulimit rtprio=99`). If the limit is missing,
 * promotion fails gracefully and the thread keeps running at normal priority.
 *
 * Priority guidance (1 = low, 99 = high; stay below kernel threads ~50-99):
 *   - control / link IO threads (comma, gnss):      ~80
 *   - controllers (mpc, path follower):             ~70-75
 *   - throughput nodes (camera, foxglove relay):    low or disabled — a
 *     CPU-bound FIFO thread can monopolise a core, so keep these BELOW the
 *     control threads (or 0 to disable) so control always preempts them.
 */
namespace rt
{

/**
 * @brief Promote the calling thread to SCHED_FIFO at @p priority.
 *
 * @param logger    Logger for status/warning output.
 * @param priority  SCHED_FIFO priority (1-99). <= 0 disables (no-op).
 * @param name      Human-readable thread name for logging.
 * @return true if the thread is now SCHED_FIFO; false otherwise.
 */
inline bool set_realtime_priority(const rclcpp::Logger& logger,
                                  int priority,
                                  const char* name = "thread")
{
    if (priority <= 0) {
        RCLCPP_DEBUG(logger, "RT scheduling disabled for '%s' (priority=%d)",
                     name, priority);
        return false;
    }

    const int max_prio = sched_get_priority_max(SCHED_FIFO);
    const int min_prio = sched_get_priority_min(SCHED_FIFO);
    if (priority > max_prio) priority = max_prio;
    if (priority < min_prio) priority = min_prio;

    struct sched_param param;
    std::memset(&param, 0, sizeof(param));
    param.sched_priority = priority;

    // pthread_setschedparam returns the error number directly (does not set errno).
    int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (rc != 0) {
        RCLCPP_WARN(logger,
            "Could not set SCHED_FIFO priority %d for '%s': %s. Running at "
            "normal priority. Grant RLIMIT_RTPRIO (container arg "
            "--ulimit rtprio=99) or CAP_SYS_NICE.",
            priority, name, std::strerror(rc));
        return false;
    }

    RCLCPP_INFO(logger, "'%s' promoted to SCHED_FIFO priority %d", name, priority);
    return true;
}

}  // namespace rt
