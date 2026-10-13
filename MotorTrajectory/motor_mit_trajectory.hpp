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

class MotorMITTrajectory
{
public:
    using ProfileConfig = velocity_profile::SCurveProfile::Config;

    MotorMITTrajectory(controllers::MotorMITController& ctrl,
                       const ProfileConfig& cfg,
                       float finish_threshold = 2.0f)
        : ctrl_(ctrl),
          cfg_(cfg),
          profile_(cfg, 0, 0, 0, 0),
          finish_threshold_(finish_threshold)
    {
    }

    // 先保持实际当前位置，再使能控制器。
    void enable()
    {
        stop();
        ctrl_.setRef(p_ref_, 0.0f);
        ctrl_.enable();
    }

    void disable()
    {
        stop();
        ctrl_.disable();
    }

    // 绝对目标角度，单位 deg。
    // 起点衔接当前曲线参考，终点速度、加速度均为零。
    void setTarget(float target_deg)
    {
        profile_ = velocity_profile::SCurveProfile(
            cfg_, p_ref_, v_ref_, a_ref_, target_deg);

        now_ = 0.0f;
        stopped_ = false;
    }

    // 相对位移以当前曲线参考位置为基准，单位 deg。
    void setRelativeTarget(float delta_deg)
    {
        setTarget(p_ref_ + delta_deg);
    }

    // 切换到实际当前位置保持，不生成减速轨迹。
    // 新参考在下一次 update() 中下发。
    void stop()
    {
        p_ref_ = getCurrentPosition();
        v_ref_ = 0.0f;
        a_ref_ = 0.0f;
        now_ = 0.0f;
        stopped_ = true;
    }

    // dt：秒；t_ff：前馈力矩，达妙 InternalMIT 下单位为 Nm。
    void update(float dt, float t_ff = 0.0f)
    {
        if (!ctrl_.enabled())
            return;

        if (!stopped_)
        {
            now_ += dt;
            p_ref_ = profile_.CalcX(now_); // deg
            v_ref_ = profile_.CalcV(now_); // deg/s
            a_ref_ = profile_.CalcA(now_); // deg/s²
        }

        ctrl_.setRef(
            p_ref_,
            v_ref_,
            controllers::MotorMITController::kNoLimit,
            t_ff);
    }

    float getCurrentPosition() const
    {
        return ctrl_.getMotor()->getAngle();
    }

    float getTotalTime() const
    {
        return profile_.getTotalTime();
    }

    // 判断规划结束且实际位置进入误差阈值。
    bool isFinished() const
    {
        return (stopped_ || now_ >= getTotalTime()) &&
               std::fabs(p_ref_ - getCurrentPosition()) < finish_threshold_;
    }

private:
    controllers::MotorMITController& ctrl_;
    ProfileConfig cfg_;
    velocity_profile::SCurveProfile profile_;

    float finish_threshold_;
    float now_ = 0.0f;
    float p_ref_ = 0.0f;
    float v_ref_ = 0.0f;
    float a_ref_ = 0.0f;
    bool stopped_ = true;
};

} // namespace trajectory

#endif // MOTOR_MIT_TRAJECTORY_HPP