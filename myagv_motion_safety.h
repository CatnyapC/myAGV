#ifndef MYAGV_WEB_MOTION_SAFETY_H
#define MYAGV_WEB_MOTION_SAFETY_H
#include <algorithm>
#include <array>
#include <cmath>

// Runs at the sole serial writer, including keyboard teleop and navigation.
struct MyAGVMotionSafety {
    using Velocity = std::array<double, 3>;
    Velocity sent{{0, 0, 0}};
    bool braking = true;
    const char* fault = nullptr;
    double lastFeedback = -1, quietSince = -1, brakeSince = -1;
    double angularLimit = .15, previousLimit = .15, limitUntil = -1;

    static int direction(double value) {
        int raw = static_cast<int>(value * 100);
        return (raw > 0) - (raw < 0);
    }
    Velocity command(Velocity requested, double now) {
        for (double value : requested)
            if (!std::isfinite(value) || std::abs(value) > 1)
                fault = "invalid velocity command";
        if (fault) return sent = {{0, 0, 0}};
        bool changed = false;
        for (int i = 0; i < 3; ++i)
            changed = changed || direction(requested[i]) != direction(sent[i]);
        if (!braking && changed) {
            braking = true;
            brakeSince = now;
            quietSince = -1;
        }
        if (brakeSince < 0) brakeSince = now;
        if (braking) {
            if (quietSince >= 0 && now - quietSince >= .5 &&
                    lastFeedback >= 0 && now - lastFeedback <= .15) {
                braking = false;
            } else {
                if (now - brakeSince > 5) fault = "stop confirmation timed out";
                requested = {{0, 0, 0}};
            }
        }
        if (fault) requested = {{0, 0, 0}};
        double limit = std::max(.15, 2 * std::abs(requested[2]) + .05);
        if (limit < angularLimit && now > limitUntil) {
            previousLimit = angularLimit;
            limitUntil = now + .25;
        }
        angularLimit = limit;
        return sent = requested;
    }
    void observe(Velocity measured, bool fresh, double now) {
        if (!fresh) fault = "invalid or timed-out chassis feedback";
        for (double value : measured)
            if (!std::isfinite(value)) fault = "non-finite chassis feedback";
        if (fault) { quietSince = -1; return; }
        // Allow only a bounded deceleration transient after reducing the command.
        double limit = now <= limitUntil ? std::max(angularLimit, previousLimit) : angularLimit;
        if (std::abs(measured[2]) > limit) fault = "angular overspeed";
        bool quiet = std::hypot(measured[0], measured[1]) <= .01 && std::abs(measured[2]) <= .02;
        if (!quiet || (lastFeedback >= 0 && now - lastFeedback > .15)) quietSince = -1;
        if (braking && quiet && quietSince < 0) quietSince = now;
        lastFeedback = now;
    }
};
#endif
