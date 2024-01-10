#pragma once
#include <math.h>

#include <fstream>
#include <iomanip>
#include <memory>
#include <string>

#include "Eigen/Core"
#include "common.h"
#include "mpc_osqp.h"




namespace shenlan {
namespace control {

using Matrix = Eigen::MatrixXd;

class MPCController {
 public:
  MPCController();
  ~MPCController();

  void LoadControlConf();
  void Init();

  bool ComputeControlCommand(
      const VehicleState &localization,
      const TrajectoryData &planning_published_trajectory, ControlCmd &cmd);

 protected:
  double Wheel2SteerPct(const double wheel_angle);
  void UpdateState(const VehicleState &vehicle_state);

  void UpdateMatrix(const VehicleState &vehicle_state);

  
  // 计算横向误差
  void ComputeLateralErrors(const double x, const double y, const double theta,
                            const double linear_v, const double angular_v,
                            const double linear_a,
                            LateralControlErrorPtr &lat_con_err);

  // 计算纵向误差
  void ComputeLongitudinalErrors(const VehicleState &vehicle_state);
  void ComputeErrors(const double x, const double y, const double theta,
                            const double linear_v, const double angular_v,
                            const double linear_a,
                            LateralControlErrorPtr &lat_con_err);



  void ToTrajectoryFrame(const double x, const double y, const double theta,
                         const double v, const TrajectoryPoint &ref_point,
                         double *ptr_s, double *ptr_s_dot, double *ptr_d,
                         double *ptr_d_dot);

  TrajectoryPoint QueryNearestPointByPosition(const double x, const double y);

  std::vector<TrajectoryPoint> trajectory_points_;

  // the following parameters are vehicle physics related.
  // control time interval
  double ts_ = 0.0;
  // corner stiffness; front
  double cf_ = 0.0;
  // corner stiffness; rear
  double cr_ = 0.0;
  // distance between front and rear wheel center
  double wheelbase_ = 0.0;
  // mass of the vehicle
  double mass_ = 0.0;
  // distance from front wheel center to COM
  double lf_ = 0.0;
  // distance from rear wheel center to COM
  double lr_ = 0.0;
  // rotational inertia
  double iz_ = 0.0;
  // the ratio between the turn of the steering wheel and the turn of the wheels
  double steer_ratio_ = 0.0;
  // the maximum turn of steer
  double steer_single_direction_max_degree_ = 0.0;
  // the maximum turn of vehicle wheel
  double wheel_single_direction_max_degree_ = 0.0;

  // limit steering to maximum theoretical lateral acceleration
  double max_lat_acc_ = 0.0;

  // number of states, includes
  // lateral error, lateral error rate, heading error, heading error rate,
  // station error, velocity error,
  const int basic_state_size_ = 6;  // 状态空间的大小
  const int controls_ = 2;        // 控制量分别为车辆的转角/汽车前进的加速度(正负皆可)
  const int horizon_ = 10;  

  // vehicle state matrix
  Eigen::MatrixXd matrix_a_;
  // vehicle state matrix (discrete-time)
  Eigen::MatrixXd matrix_ad_;
  // control matrix
  Eigen::MatrixXd matrix_b_;
  // control matrix (discrete-time)
  Eigen::MatrixXd matrix_bd_;
  // control authority weighting matrix
  Eigen::MatrixXd matrix_r_;
  // state weighting matrix
  Eigen::MatrixXd matrix_q_;
  // updated state weighting matrix
  Eigen::MatrixXd matrix_q_updated_;
  // vehicle state matrix coefficients
  Eigen::MatrixXd matrix_a_coeff_;
  // 4 by 1 matrix; state matrix
  Eigen::MatrixXd matrix_state_;

  // parameters for mpc solver; number of iterations
  int mpc_max_iteration_ = 0;
  // parameters for mpc solver; threshold for computation
  double mpc_eps_ = 0.0;

  double max_acceleration_ = 0.0;
  double max_deceleration_ = 0.0;
  double minimum_speed_protection_ = 0.1;

  double station_error_ = 0.0;
  double speed_error_ = 0.0;
};

}  // namespace control
}  // namespace shenlan
