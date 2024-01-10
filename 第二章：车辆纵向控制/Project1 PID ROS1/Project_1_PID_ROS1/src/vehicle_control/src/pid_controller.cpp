#include "pid_controller.h"

namespace shenlan
{
  namespace control
  {

    PIDController::PIDController(const double kp, const double ki,
                                 const double kd)
    {
      kp_ = kp;
      ki_ = ki;
      kd_ = kd;
      previous_error_ = 0.0;
      previous_output_ = 0.0;
      integral_ = 0.0;
      first_hit_ = true; // 是否是第一次迭代
    }

    // 计算 PID 输出，实现PID控制
    double PIDController::Control(const double error, const double dt)
    {
      double differential = 0.0;
      double PID_out = 0.0;

      /*首次执行 Control 函数时，first_hit_ 为 true，因此进入 if 分支，将 first_hit_ 设置为 false，
      以后每次进入 Control 函数时，由于 first_hit_ 已经被设置为 false，程序会进入 else 分支，执行微分项的计算*/
      if (first_hit_)
      {
        first_hit_ = false;
      }
      else
      {
        first_hit_ = true;
        differential = (error - previous_error_) / dt;
      }

      integral_ = integral_ + ki_ * error * dt;
      PID_out = kp_ * error + ki_ * integral_ + kd_ * differential;

      previous_error_ = error;
      previous_output_ = PID_out;

      return PID_out;
    }

    // /**to-do**/ 重置PID参数
    void PIDController::Reset()
    {

      integral_ = 0;
      current_output = 0;
      previous_error_ = 0;
      first_hit_ = true;
    }

  } // namespace control
} // namespace shenlan
