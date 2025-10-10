#pragma once
// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
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

#include <array>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>

namespace cartesian_controller {

// ======= Type Aliases for Cleaner Code =======
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Vec7 = Eigen::Matrix<double, 7, 1>;
using Mat6x7 = Eigen::Matrix<double, 6, 7>;
using Mat7x6 = Eigen::Matrix<double, 7, 6>;
using Mat7x7 = Eigen::Matrix<double, 7, 7>;
using Mat6x6 = Eigen::Matrix<double, 6, 6>;
using Transform = Eigen::Isometry3d;

// ======= Constants =======
inline constexpr std::array<std::string_view, 7> CONTROLLED_JOINTS{
  "elevator_joint",
  "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
};

inline constexpr double DEFAULT_EPS = 1e-12;
inline constexpr double POS_TOL = 1e-3;    // 1mm position tolerance
inline constexpr double ANG_TOL = 1e-2;    // ~0.5 degree angle tolerance

// ======= SE(3) Math Utilities =======
[[nodiscard]] Vec3 so3Log(const Mat3& R);
[[nodiscard]] Mat3 so3Exp(const Vec3& phi);

// ======= Trapezoidal Velocity Profile =======
// Handles timing of motion along geometric path with smooth accel/decel
class TrapProfile {
public:
  void configure(double sdot_max, double sddot_max);
  void start(const rclcpp::Time& now) noexcept { t0_ = now; }
  [[nodiscard]] bool finished(const rclcpp::Time& now) const noexcept { 
    return (now - t0_).seconds() >= T_; 
  }
  void eval(const rclcpp::Time& now, double& s, double& sdot) const noexcept;

private:
  double sdot_max_{0.5}, sddot_max_{1.0};
  double ta_{0}, tc_{0}, T_{0};
  rclcpp::Time t0_;
  bool triangular_{false};
};

// ======= QP Parameters Structure =======
struct QPParams {
  Vec7 Wj = Vec7::Ones();           // Joint effort weights
  Vec7 Kp = Vec7::Constant(0.5);    // Posture restoration gains
  Vec7 q0 = Vec7::Zero();            // Comfortable posture
  double lambda_ns{1e-2};            // Nullspace objective weight
  double rho{1e3};                   // Path tracking weight
  double kappa{100.0};               // Speed maximization weight
  double lambda_omega{1.0};          // Angular velocity weight
  double lambda_v{1.0};              // Linear velocity weight
  double eps_u{1e-9};                // Unit vector numerical stability
  double pinv_damping{1e-6};        // Singularity damping
  double h{0.01};                    // Control timestep
  double H_reg{1e-9};                // QP regularization
};

// ======= Physical Limits Structure =======
struct Limits {
  Vec7 qmin = Vec7::Constant(-1e9);     // Joint position limits
  Vec7 qmax = Vec7::Constant(1e9);
  Vec7 dqmin = Vec7::Constant(-1.5);    // Joint velocity limits  
  Vec7 dqmax = Vec7::Constant(1.5);
  Vec7 ddqmin = Vec7::Constant(-5.0);   // Joint acceleration limits
  Vec7 ddqmax = Vec7::Constant(5.0);
  double v_lin_max{1.5};                // Cartesian linear velocity limit
  double a_lin_max{7.5};                // Cartesian linear acceleration limit
  double w_ang_max{6.0};                // Cartesian angular velocity limit
  double alpha_ang_max{30.0};           // Cartesian angular acceleration limit
};

// ======= QP Solution Result =======
struct QPResult {
  Vec7 dq = Vec7::Zero();
  double alpha{0.0};
  Vec6 w = Vec6::Zero();
  bool ok{false};
};

// ======= Differential IK QP Solver =======
// Formulates and solves QP for optimal joint velocities with constraints
class DiffIK_QP {
public:
  explicit DiffIK_QP(const QPParams& params) : params_(params) {}

  [[nodiscard]] QPResult solve(const Vec7& q,
                               const Vec7& dq_meas,
                               const Mat6x7& J,
                               const Vec6& u,
                               const Limits& lim) const;

private:
  [[nodiscard]] Mat7x6 computeDampedPinv(const Mat6x7& J) const;
  [[nodiscard]] std::pair<Vec7, Vec7> computeBounds(const Vec7& q,
                                                     const Vec7& dq_meas,
                                                     const Limits& lim) const;
  
  QPParams params_;
  mutable OsqpEigen::Solver solver_;  // Mutable for warm-starting
};

// ======= Main Cartesian Controller Node =======
class CartesianController : public rclcpp::Node {
public:
  explicit CartesianController(const rclcpp::NodeOptions& opt = rclcpp::NodeOptions());

private:
  // ===== Configuration =====
  struct Config {
    std::string urdf_param{"robot_description"};
    std::string root_link{"world"};
    std::string tip_link{"tool0"};
    int control_hz{100};
    double lambda_omega{1.0};
    double lambda_v{1.0};
  } config_;

  Limits limits_;
  QPParams qp_params_;

  // ===== Kinematics =====
  struct Kinematics {
    KDL::Chain chain;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
    std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver;
  } kin_;

  // ===== Robot State =====
  struct State {
    Vec7 q = Vec7::Zero();
    Vec7 dq_meas = Vec7::Zero();
    std::vector<std::string> joint_names;
    std::unordered_map<std::string, size_t> name_to_idx;
    bool initialized{false};
  } state_;

  // ===== Path Planning =====
  struct PathData {
    Vec3 p0_W = Vec3::Zero();      // Start position
    Vec3 pstar_W = Vec3::Zero();   // Goal position
    Vec3 dp = Vec3::Zero();        // Translation vector
    Mat3 R0_W = Mat3::Identity();   // Start rotation
    Mat3 Rstar_W = Mat3::Identity(); // Goal rotation
    Vec3 phi = Vec3::Zero();        // Rotation axis-angle
    TrapProfile profile;
    bool active{false};
  } path_;

  // ===== ROS Interface =====
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_point_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_cartesian_vel_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ===== Core Methods =====
  void onJointState(const sensor_msgs::msg::JointState::ConstSharedPtr msg);
  void onTargetPoint(const geometry_msgs::msg::PointStamped::ConstSharedPtr msg);
  void onTargetPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void onTimer();

  // ===== Helper Methods =====
  [[nodiscard]] bool initKinematics();
  [[nodiscard]] bool mapJointOrder(const sensor_msgs::msg::JointState& js);
  [[nodiscard]] bool extractJointState(const sensor_msgs::msg::JointState& js);
  [[nodiscard]] KDL::Frame computeFK(const Vec7& q) const;
  [[nodiscard]] Mat6x7 computeJacobian(const Vec7& q) const;
  [[nodiscard]] Transform kdlToEigen(const KDL::Frame& frame) const;
  void startPath(const Transform& T_now, const Transform& T_target);
  [[nodiscard]] bool checkConvergence(const Transform& T_now, double s) const;
  void publishCommand(const Vec7& dq);
  void loadParameters();
};

} // namespace cartesian_controller
