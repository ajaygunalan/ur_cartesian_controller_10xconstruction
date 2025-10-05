#pragma once
// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

// KDL for FK/Jac only
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/Sparse>

// OSQP
#include <OsqpEigen/OsqpEigen.h>

#include <optional>
#include <unordered_map>
#include <vector>
#include <string>
#include <chrono>

namespace cartesian_controller {

// Controllable joints (elevator + 6 UR joints; excludes ft_sensor_joint)
inline const std::array<std::string, 7> CONTROLLED_JOINTS{{
  "elevator_joint",
  "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
}};

// ---------- Helpers (SE(3) math) ----------
Eigen::Vector3d so3Log(const Eigen::Matrix3d& R);
Eigen::Matrix3d so3Exp(const Eigen::Vector3d& phi);
inline double clamp(double x, double lo, double hi) { return std::min(std::max(x, lo), hi); }

// ---------- Trapezoidal profile on s ∈ [0,1] ----------
struct TrapProfile {
  double sdot_max{0.5}, sddot_max{1.0};
  double ta{0}, tc{0}, T{0};
  rclcpp::Time t0;
  bool triangular{false};

  void configure(double sdot_max_in, double sddot_max_in);
  void start(const rclcpp::Time& now);
  void eval(const rclcpp::Time& now, double& s, double& sdot) const;
  bool finished(const rclcpp::Time& now) const;
};

// ---------- QP-based diff-IK (matches notes.md) ----------
struct QPParams {
  // Weights
  Eigen::Matrix<double,7,1> Wj;
  double lambda_ns{1e-2};
  double rho{1e3};
  double kappa{100.0};
  Eigen::Matrix<double,7,1> Kp;
  Eigen::Matrix<double,7,1> q0;
  double eps_u{1e-9};
  double pinv_damping{1e-6};
  double h{0.01};
  double H_reg{1e-9};
  double lambda_omega{1.0};
  double lambda_v{1.0};
};

struct Limits {
  Eigen::Matrix<double,7,1> qmin, qmax;
  Eigen::Matrix<double,7,1> dqmin, dqmax;
  Eigen::Matrix<double,7,1> ddqmin, ddqmax;
  double v_lin_max{1.5};
  double a_lin_max{7.5};
  double w_ang_max{6.0};
  double alpha_ang_max{30.0};
};

struct QPResult {
  Eigen::Matrix<double,7,1> dq;
  double alpha{0.0};
  Eigen::Matrix<double,6,1> w;
  bool ok{false};
};

class DiffIK_QP {
public:
  explicit DiffIK_QP(const QPParams& p): P_(p) {}

  QPResult solve(const Eigen::Matrix<double,7,1>& q,
                 const Eigen::Matrix<double,7,1>& dq_meas,
                 const Eigen::Matrix<double,6,7>& J,
                 const Eigen::Matrix<double,6,1>& u,
                 const Limits& lim);

private:
  QPParams P_;
  Eigen::Matrix<double,7,6> pinvRightDamped_(const Eigen::Matrix<double,6,7>& J) const;
  void buildBounds_(const Eigen::Matrix<double,7,1>& q,
                    const Eigen::Matrix<double,7,1>& dq_meas,
                    const Limits& lim,
                    Eigen::Matrix<double,7,1>& lb,
                    Eigen::Matrix<double,7,1>& ub) const;
};

// ---------- ROS2 Node ----------
class CartesianController : public rclcpp::Node {
public:
  explicit CartesianController(const rclcpp::NodeOptions& opt = rclcpp::NodeOptions());

private:
  // Params/frames
  std::string urdf_param_{"robot_description"};
  std::string root_link_{"world"};
  std::string tip_link_{"tool0"};
  int control_hz_{100};

  double lambda_omega_{1.0}, lambda_v_{1.0};

  Limits lim_;
  QPParams qp_;

  // KDL
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;

  // Joint state
  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, size_t> name_to_idx_;
  Eigen::Matrix<double,7,1> q_{Eigen::Matrix<double,7,1>::Zero()};
  Eigen::Matrix<double,7,1> dq_meas_{Eigen::Matrix<double,7,1>::Zero()};
  bool have_js_{false};

  // Goal/path
  bool active_goal_{false};
  Eigen::Vector3d p0_W_{Eigen::Vector3d::Zero()}, pstar_W_{Eigen::Vector3d::Zero()}, dp_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d R0_W_{Eigen::Matrix3d::Identity()}, Rstar_W_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d phi_{Eigen::Vector3d::Zero()};
  TrapProfile prof_;

  // ROS IO
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_point_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  sub_pose_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;

  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg);
  void onTargetPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void onTargetPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onTimer();

  bool initKDLfromURDF_();
  bool mapJointOrder_(const sensor_msgs::msg::JointState& js);
  bool extractQ_(const sensor_msgs::msg::JointState& js, Eigen::Matrix<double,7,1>& q);
  KDL::Frame fk_(const Eigen::Matrix<double,7,1>& q) const;
  Eigen::Matrix<double,6,7> jac_(const Eigen::Matrix<double,7,1>& q) const;
  void startPathFromCurrent_(const Eigen::Isometry3d& T_now, const Eigen::Isometry3d& T_star);
  bool convergenceCheck_(const Eigen::Isometry3d& T_now, double s) const;
};

} // namespace cartesian_controller
