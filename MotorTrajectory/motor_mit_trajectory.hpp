/**
 * @file    motor_mit_trajectory.hpp
 * @author  Fang_Shuohan
 * @date    2026-01-29
 */

#ifndef MOTOR_MIT_TRAJECTORY_HPP
#define MOTOR_MIT_TRAJECTORY_HPP

#include "motor_mit_controller.hpp"
#include "s_curve.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits.h>

namespace trajectory
{

template<std::size_t N>
class MotorMITTrajectory{
    static_assert(N > 0);

public:
    using Controller = controllers::MotorMITController;
    using Profile = velocity_profile::SCurveProfile;
    using ProfileConfig = Profile::Config;
    using Values = std::array<float, N>;

    struct Axis {
        Controller* ctrl;
        ProfileConfig limits;   //速度规划参数，deg/s,deg/s^2,deg/s^3
        float position_tolerance = 2.0f;
        float stopped_speed = 1.0f;
    };

    enum class Result {
        Accepted,
        Unchanged,
        Busy,
        NotEnabled,
        InvalidTarget,
        PlanFailed
    };

    explicit MotorMITTrajectory(const std::array<Axis, N>& axes)
        : axes_(axes) {}

    MotorMITTrajectory(const MotorMITTrajectory&) = delete;
    MotorMITTrajectory& operator=(const MotorMITTrajectory&) = delete;  //防止直接复制集中式速度规划器

    bool initialize(const Values& ff = {}) {
        if(initialized_)
            return false;   //防止重复使能

        Values initial{};

        //下面全是防御性编程

        for(std::size_t i = 0; i < N; ++i) {
            const auto& axis = axes_[i];

            if (!axis.ctrl || !axis.ctrl->getMotor() ||
                axis.ctrl->enabled() ||
                !positive(axis.limits.max_spd) ||
                !positive(axis.limits.max_acc) ||
                !positive(axis.limits.max_jerk) ||
                !positive(axis.position_tolerance) ||
                !positive(axis.stopped_speed) ||
                !std::isfinite(ff[i]))
                return false;

    //同一个电机不可以重复挂入
            for (std::size_t j = 0; j < i; ++j)
                if (axis.ctrl->getMotor() ==
                    axes_[j].ctrl->getMotor())
                    return false;
        
            initial[i] = axis.ctrl->getMotor()->getAngle();

            const float speed = axis.ctrl->getMotor()->getVelocity() * 6.0f; // rpm -> deg/s

            if(!std::isfinite(initial[i]) ||
            !std::isfinite(speed) ||
            std::fabs(speed) > axis.stopped_speed)   //等待电机停稳
                return false;
        }

        for (std::size_t i = 0; i < N; ++i) {
            auto* ctrl = axes_[i].ctrl;

            ctrl->setRef(initial[i], 0,
                         Controller::kNoLimit, ff[i]);

            if (!ctrl->enable()) {
                for (std::size_t j = 0; j < i; ++j)
                    axes_[j].ctrl->disable();
                return false;
            }
        }

        position_ = goal_ = initial;
        velocity_.fill(0.0f);
        elapsed_ = 0.0f;
        initialized_ = true;
        return true;

    }

    Result setTarget(const Values& target) {
        if(!ready())
            return Result::NotEnabled;

        for (float value : target)
            if (!std::isfinite(value))
                return Result::InvalidTarget;

        if(target == goal_)
            return Result::Unchanged;

        if(running_)
            return Result::Busy;

        const float largest = std::numeric_limits<float>::max();
        ProfileConfig common{largest, largest, largest};
        Values delta{};

        for (std::size_t i = 0; i < N; ++i) {
            delta[i] = target[i] - position_[i];
            const float distance = std::fabs(delta[i]);

            if (!std::isfinite(distance))
                return Result::InvalidTarget;

            if (distance == 0)
                continue;

            common.max_spd = std::min(
                common.max_spd,
                axes_[i].limits.max_spd / distance);

            common.max_acc = std::min(
                common.max_acc,
                axes_[i].limits.max_acc / distance);

            common.max_jerk = std::min(
                common.max_jerk,
                axes_[i].limits.max_jerk / distance);
            }
         Profile next(common, 0, 0, 0, 1);

        if (!next.success() || !positive(next.getTotalTime()))
            return Result::PlanFailed;

        // 规划成功后才替换当前状态。
        profile_ = next;
        start_ = position_;
        delta_ = delta;
        goal_ = target;
        elapsed_ = 0;
        running_ = true;

        return Result::Accepted;
    
    }

    bool update(float dt, const Values& ff = {}) {
        if (!ready() || !positive(dt))
            return false;

        for (float value : ff)
            if (!std::isfinite(value))
                return false;

        if (running_) {
            elapsed_ = std::min(
                elapsed_ + dt, profile_.getTotalTime());

            const bool done =
                elapsed_ >= profile_.getTotalTime();

            const float s =
                done ? 1.0f : profile_.CalcX(elapsed_);
            const float ds =
                done ? 0.0f : profile_.CalcV(elapsed_);

            for (std::size_t i = 0; i < N; ++i) {
                position_[i] = done
                    ? goal_[i]
                    : start_[i] + delta_[i] * s;

                velocity_[i] = delta_[i] * ds;
            }

            running_ = !done;
        }

        for (std::size_t i = 0; i < N; ++i) {
            axes_[i].ctrl->setRef(
                position_[i],
                velocity_[i],
                Controller::kNoLimit,
                ff[i]);
        }

        return true;
    }

     // 当前已接受目标：规划结束，反馈位置与速度均满足条件。
    bool isFinished() const {
        if (!ready() || running_)
            return false;

        for (std::size_t i = 0; i < N; ++i) {
            const auto& axis = axes_[i];
            const float p = axis.ctrl->getMotor()->getAngle();
            const float v =
                axis.ctrl->getMotor()->getVelocity() * 6.0f;

            if (!std::isfinite(p) || !std::isfinite(v) ||
                std::fabs(p - goal_[i]) >
                    axis.position_tolerance ||
                std::fabs(v) > axis.stopped_speed)
                return false;
        }

        return true;
    }

    const Values& positionRef() const {
        return position_;
    }

    const Values& velocityRef() const {
        return velocity_;
    }

    
    
private:
    static bool positive(float value) {
        return std::isfinite(value) && value > 0.0f;
    }

    bool ready() const {
        if(!initialized_)
            return false;

        for(const auto& axis : axes_)
            if (!axis.ctrl->enabled()) {
                return false;
            }

        return true;
    }

    std::array<Axis, N> axes_;

    Profile profile_{{1,1,1},0,0,0,0};

    Values position_{};
    Values velocity_{};
    Values start_{};
    Values delta_{};
    Values goal_{};

    float elapsed_ = 0;
    bool initialized_ = false;
    bool running_ = false;
};

} // namespace trajectory

#endif // MOTOR_MIT_TRAJECTORY_HPP