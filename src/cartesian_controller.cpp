#include "ur_cartesian_controller/cartesian_controller.hpp"
#include <algorithm>
#include <numeric>

namespace cartesian_controller {

// ======= SO(3) Helper Functions =======
// Computes the log map from SO(3) to so(3): extracts axis-angle from rotation matrix
// Returns phi = axis * angle (3D vector representing rotation)
Vec3 so3Log(const Mat3& R) {
  const double cos_theta = std::clamp((R.trace() - 1.0) * 0.5, -1.0, 1.0);
  const double theta = std::acos(cos_theta);
  if (theta < DEFAULT_EPS) return Vec3::Zero();
  
  const Vec3 axis((R(2,1)-R(1,2)), (R(0,2)-R(2,0)), (R(1,0)-R(0,1)));
  return axis * (theta / (2.0 * std::sin(theta)));
}

// Computes the exponential map from so(3) to SO(3): creates rotation matrix from axis-angle
// Takes phi (axis * angle) and returns rotation matrix R
Mat3 so3Exp(const Vec3& phi) {
  const double theta = phi.norm();
  if (theta < DEFAULT_EPS) return Mat3::Identity();
  
  const Vec3 a = phi / theta;
  Mat3 ax;
  ax << 0, -a.z(), a.y(),
        a.z(), 0, -a.x(),
        -a.y(), a.x(), 0;
  
  return Mat3::Identity() + std::sin(theta) * ax + (1 - std::cos(theta)) * (ax * ax);
}

// ======= Trapezoidal Velocity Profile =======
// This class handles the TIMING of the motion along the geometric path.
// It creates a smooth acceleration-cruise-deceleration profile for the path parameter s(t).
void TrapProfile::configure(double sdot_max, double sddot_max) {
  sdot_max_ = std::max(1e-6, sdot_max);   // Max velocity along path
  sddot_max_ = std::max(1e-6, sddot_max); // Max acceleration along path
  
  // Check if we can reach max velocity or if motion is triangular (no cruise phase)
  const double crit = (sdot_max_ * sdot_max_) / sddot_max_;
  
  if (crit > 1.0) {
    // Triangular profile: accelerate then immediately decelerate
    triangular_ = true;
    ta_ = std::sqrt(1.0 / sddot_max_);
    tc_ = 0.0;
    T_ = 2.0 * ta_;
  } else {
    // Trapezoidal profile: accelerate -> cruise -> decelerate
    triangular_ = false;
    ta_ = sdot_max_ / sddot_max_;
    tc_ = (1.0 - crit) / sdot_max_;
    T_ = 2.0 * ta_ + tc_;
  }
}

// Evaluates the profile at current time, returning position s and velocity sdot
void TrapProfile::eval(const rclcpp::Time& now, double& s, double& sdot) const noexcept {
  const double t = (now - t0_).seconds();
  
  if (t <= 0.0) { s = 0.0; sdot = 0.0; return; }
  if (t >= T_)  { s = 1.0; sdot = 0.0; return; }
  
  if (triangular_) {
    if (t < ta_) {
      sdot = sddot_max_ * t;
      s = 0.5 * sddot_max_ * t * t;
    } else {
      const double dt = T_ - t;
      sdot = sddot_max_ * dt;
      s = 1.0 - 0.5 * sddot_max_ * dt * dt;
    }
  } else {
    if (t < ta_) {
      sdot = sddot_max_ * t;
      s = 0.5 * sddot_max_ * t * t;
    } else if (t < ta_ + tc_) {
      sdot = sdot_max_;
      s = 0.5 * sddot_max_ * ta_ * ta_ + sdot_max_ * (t - ta_);
    } else {
      const double dt = T_ - t;
      sdot = sddot_max_ * dt;
      s = 1.0 - 0.5 * sddot_max_ * dt * dt;
    }
  }
}

// ======= Differential IK Quadratic Program Solver =======
// Computes damped right pseudo-inverse of Jacobian
// Damping prevents instability near singularities
Mat7x6 DiffIK_QP::computeDampedPinv(const Mat6x7& J) const {
  const auto svd = J.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Mat7x6 V = svd.matrixV().leftCols<6>();
  const Mat6x6 U = svd.matrixU();
  const auto S = svd.singularValues();
  
  // Damped pseudo-inverse: si/(si^2 + damping^2)
  Mat6x6 Splus = Mat6x6::Zero();
  const double damping_sq = params_.pinv_damping * params_.pinv_damping;
  for (int i = 0; i < 6; ++i) {
    const double si = S(i);
    Splus(i, i) = si / (si * si + damping_sq);
  }
  
  return V * Splus * U.transpose();
}

// Builds joint velocity bounds respecting position, velocity, and acceleration limits
std::pair<Vec7, Vec7> DiffIK_QP::computeBounds(const Vec7& q, 
                                                const Vec7& dq_meas,
                                                const Limits& lim) const {
  Vec7 lb, ub;
  
  for (int i = 0; i < 7; ++i) {
    // Three constraint types: velocity, position-based, acceleration-based
    const double vel_lo = lim.dqmin(i);
    const double vel_hi = lim.dqmax(i);
    const double pos_lo = (lim.qmin(i) - q(i)) / params_.h;
    const double pos_hi = (lim.qmax(i) - q(i)) / params_.h;
    const double acc_lo = dq_meas(i) + params_.h * lim.ddqmin(i);
    const double acc_hi = dq_meas(i) + params_.h * lim.ddqmax(i);
    
    // Take intersection of all constraints
    lb(i) = std::max({vel_lo, pos_lo, acc_lo});
    ub(i) = std::min({vel_hi, pos_hi, acc_hi});
    
    // Handle infeasible bounds
    if (lb(i) > ub(i)) {
      const double mid = 0.5 * (lb(i) + ub(i));
      lb(i) = ub(i) = mid;
    }
  }
  
  return {lb, ub};
}

// Main QP solver - finds optimal joint velocities
QPResult DiffIK_QP::solve(const Vec7& q, const Vec7& dq_meas,
                          const Mat6x7& J, const Vec6& u, const Limits& lim) const {
  QPResult result;
  
  // ===== STEP 1: Compute Nullspace Projector =====
  const Mat7x6 Jplus = computeDampedPinv(J);
  const Mat7x7 Pn = Mat7x7::Identity() - Jplus * J;
  
  // ===== STEP 2: Setup QP Problem =====
  constexpr int n = 14;  // Decision variables: [dq(7); alpha(1); w(6)]
  constexpr int m_eq = 6;
  constexpr int m_ineq = 8;
  constexpr int m = m_eq + m_ineq;
  
  // ===== STEP 3: Build Objective =====
  // Quadratic term H
  const Mat7x7 Wj2 = params_.Wj.cwiseProduct(params_.Wj).asDiagonal();
  const Mat7x7 Hqq = Wj2 + params_.lambda_ns * (Pn.transpose() * Pn);
  
  // Linear term f
  const Vec7 dq_posture = params_.Kp.cwiseProduct(q - params_.q0);
  const Vec7 dq_lin = params_.lambda_ns * (Pn.transpose() * Pn) * dq_posture;
  
  // Build sparse matrices efficiently
  Eigen::SparseMatrix<double> H(n, n);
  std::vector<Eigen::Triplet<double>> H_triplets;
  H_triplets.reserve(7*7 + 1 + 6);  // Pre-allocate
  
  // Add Hqq terms
  for (int i = 0; i < 7; ++i) {
    for (int j = i; j < 7; ++j) {
      if (std::abs(Hqq(i,j)) > 1e-12) {
        H_triplets.emplace_back(i, j, Hqq(i,j));
      }
    }
  }
  
  // Add regularization and slack penalties
  H_triplets.emplace_back(7, 7, params_.H_reg);
  for (int i = 0; i < 6; ++i) {
    H_triplets.emplace_back(8+i, 8+i, params_.rho);
  }
  H.setFromTriplets(H_triplets.begin(), H_triplets.end());
  
  // Linear coefficients
  Eigen::VectorXd f = Eigen::VectorXd::Zero(n);
  f.head<7>() = dq_lin;
  f(7) = -params_.kappa;  // Maximize speed
  
  // ===== STEP 4: Build Constraints =====
  const auto [lb_dq, ub_dq] = computeBounds(q, dq_meas, lim);
  
  Eigen::SparseMatrix<double> A(m, n);
  std::vector<Eigen::Triplet<double>> A_triplets;
  A_triplets.reserve(6*7 + 6 + 6 + 7 + 1);  // Pre-allocate
  
  // Equality constraints: J*dq - alpha*u - w = 0
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 7; ++c) {
      if (std::abs(J(r,c)) > 1e-12) {
        A_triplets.emplace_back(r, c, J(r,c));
      }
    }
    A_triplets.emplace_back(r, 7, -u(r));
    A_triplets.emplace_back(r, 8+r, -1.0);
  }
  
  // Inequality constraints
  for (int i = 0; i < 7; ++i) {
    A_triplets.emplace_back(m_eq + i, i, 1.0);
  }
  A_triplets.emplace_back(m_eq + 7, 7, 1.0);
  A.setFromTriplets(A_triplets.begin(), A_triplets.end());
  
  // Bounds
  Eigen::VectorXd l(m), uvec(m);
  l.head<6>().setZero();
  uvec.head<6>().setZero();
  l.segment<7>(6) = lb_dq;
  uvec.segment<7>(6) = ub_dq;
  l(13) = 0.0;
  uvec(13) = 1.0;
  
  // ===== STEP 5: Solve QP =====
  solver_.settings()->setWarmStart(true);
  solver_.settings()->setAdaptiveRho(true);
  solver_.settings()->setAlpha(1.6);
  solver_.settings()->setPolish(true);
  solver_.settings()->setVerbosity(false);
  
  solver_.data()->setNumberOfVariables(n);
  solver_.data()->setNumberOfConstraints(m);
  solver_.data()->setHessianMatrix(H);
  solver_.data()->setGradient(f);
  solver_.data()->setLinearConstraintsMatrix(A);
  solver_.data()->setLowerBound(l);
  solver_.data()->setUpperBound(uvec);
  
  if (!solver_.initSolver() || 
      solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
    return result;
  }
  
  const auto z = solver_.getSolution();
  result.dq = z.segment<7>(0);
  result.alpha = z(7);
  result.w = z.segment<6>(8);
  result.ok = true;
  
  return result;
}

// ======= ROS2 Node Implementation =======
CartesianController::CartesianController(const rclcpp::NodeOptions& opt)
  : rclcpp::Node("cartesian_controller", opt) {
  
  loadParameters();
  
  if (!initKinematics()) {
    RCLCPP_FATAL(get_logger(), "Kinematics initialization failed");
    throw std::runtime_error("Kinematics init failed");
  }
  
  // Setup publishers and subscribers
  pub_cmd_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/forward_velocity_controller/commands", rclcpp::SystemDefaultsQoS());
    
  pub_cartesian_vel_ = create_publisher<geometry_msgs::msg::TwistStamped>(
    "/cartesian_controller/cartesian_velocity", rclcpp::SystemDefaultsQoS());
    
  sub_js_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SensorDataQoS(),
    [this](const auto& msg) { onJointState(msg); });
    
  sub_point_ = create_subscription<geometry_msgs::msg::PointStamped>(
    "/cartesian_controller/target_point", rclcpp::SystemDefaultsQoS(),
    [this](const auto& msg) { onTargetPoint(msg); });
    
  sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/cartesian_controller/target_pose", rclcpp::SystemDefaultsQoS(),
    [this](const auto& msg) { onTargetPose(msg); });
    
  // Main control loop
  const auto period = std::chrono::milliseconds(1000 / std::max(1, config_.control_hz));
  timer_ = create_wall_timer(period, [this]() { onTimer(); });
  
  RCLCPP_INFO(get_logger(), 
    "QP-based Cartesian controller initialized [%s -> %s @ %d Hz]",
    config_.root_link.c_str(), config_.tip_link.c_str(), config_.control_hz);
}

// Load all parameters from ROS parameter server
void CartesianController::loadParameters() {
  // Lambda helper for vector parameter loading
  auto loadVec7 = [this](const std::string& name, Vec7& target, const Vec7& default_val) {
    std::vector<double> vec(default_val.data(), default_val.data() + 7);
    declare_parameter(name, vec);
    get_parameter(name, vec);
    std::copy_n(vec.begin(), 7, target.data());
  };
  
  // Core configuration
  declare_parameter("urdf_param", config_.urdf_param);
  declare_parameter("root_link", config_.root_link);
  declare_parameter("tip_link", config_.tip_link);
  declare_parameter("control_hz", config_.control_hz);
  declare_parameter("lambda_omega", config_.lambda_omega);
  declare_parameter("lambda_v", config_.lambda_v);
  
  get_parameter("urdf_param", config_.urdf_param);
  get_parameter("root_link", config_.root_link);
  get_parameter("tip_link", config_.tip_link);
  get_parameter("control_hz", config_.control_hz);
  get_parameter("lambda_omega", config_.lambda_omega);
  get_parameter("lambda_v", config_.lambda_v);
  
  // QP parameters
  declare_parameter("lambda_ns", qp_params_.lambda_ns);
  declare_parameter("rho", qp_params_.rho);
  declare_parameter("kappa", qp_params_.kappa);
  declare_parameter("eps_u", qp_params_.eps_u);
  declare_parameter("pinv_damping", qp_params_.pinv_damping);
  declare_parameter("H_reg", qp_params_.H_reg);
  
  get_parameter("lambda_ns", qp_params_.lambda_ns);
  get_parameter("rho", qp_params_.rho);
  get_parameter("kappa", qp_params_.kappa);
  get_parameter("eps_u", qp_params_.eps_u);
  get_parameter("pinv_damping", qp_params_.pinv_damping);
  get_parameter("H_reg", qp_params_.H_reg);
  
  loadVec7("Wj", qp_params_.Wj, Vec7::Ones());
  loadVec7("Kp", qp_params_.Kp, Vec7::Constant(0.5));
  loadVec7("q0", qp_params_.q0, Vec7::Zero());
  
  // Physical limits
  loadVec7("qmin", limits_.qmin, Vec7::Constant(-1e9));
  loadVec7("qmax", limits_.qmax, Vec7::Constant(1e9));
  loadVec7("dqmin", limits_.dqmin, Vec7::Constant(-1.5));
  loadVec7("dqmax", limits_.dqmax, Vec7::Constant(1.5));
  loadVec7("ddqmin", limits_.ddqmin, Vec7::Constant(-5.0));
  loadVec7("ddqmax", limits_.ddqmax, Vec7::Constant(5.0));
  
  declare_parameter("v_lin_max", limits_.v_lin_max);
  declare_parameter("a_lin_max", limits_.a_lin_max);
  declare_parameter("w_ang_max", limits_.w_ang_max);
  declare_parameter("alpha_ang_max", limits_.alpha_ang_max);
  declare_parameter("joint_names", state_.joint_names);
  
  get_parameter("v_lin_max", limits_.v_lin_max);
  get_parameter("a_lin_max", limits_.a_lin_max);
  get_parameter("w_ang_max", limits_.w_ang_max);
  get_parameter("alpha_ang_max", limits_.alpha_ang_max);
  get_parameter("joint_names", state_.joint_names);
  
  // Set derived parameters
  qp_params_.h = 1.0 / std::max(1, config_.control_hz);
  qp_params_.lambda_omega = config_.lambda_omega;
  qp_params_.lambda_v = config_.lambda_v;
}

// Initialize KDL kinematics from URDF
bool CartesianController::initKinematics() {
  auto param_client = std::make_shared<rclcpp::SyncParametersClient>(
    this, "/robot_state_publisher");
    
  if (!param_client->wait_for_service(std::chrono::seconds(5))) {
    return false;
  }
  
  const auto params = param_client->get_parameters({"robot_description"});
  const std::string urdf_xml = params[0].as_string();
  
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(urdf_xml, tree) ||
      !tree.getChain(config_.root_link, config_.tip_link, kin_.chain)) {
    return false;
  }
  
  kin_.fk_solver = std::make_unique<KDL::ChainFkSolverPos_recursive>(kin_.chain);
  kin_.jac_solver = std::make_unique<KDL::ChainJntToJacSolver>(kin_.chain);
  
  return true;
}

// Map joint names to indices
bool CartesianController::mapJointOrder(const sensor_msgs::msg::JointState& js) {
  state_.name_to_idx.clear();
  for (size_t i = 0; i < js.name.size(); ++i) {
    state_.name_to_idx[js.name[i]] = i;
  }
  
  if (state_.joint_names.empty()) {
    state_.joint_names.assign(CONTROLLED_JOINTS.begin(), CONTROLLED_JOINTS.end());
  }
  
  return std::all_of(state_.joint_names.begin(), state_.joint_names.end(),
    [this, &js](const auto& name) {
      if (state_.name_to_idx.count(name) == 0) {
        RCLCPP_ERROR(get_logger(), "Joint '%s' not in joint_states", name.c_str());
        return false;
      }
      return true;
    });
}

// Extract joint state from message
bool CartesianController::extractJointState(const sensor_msgs::msg::JointState& js) {
  for (int i = 0; i < 7; ++i) {
    const auto it = state_.name_to_idx.find(state_.joint_names[i]);
    if (it == state_.name_to_idx.end() || it->second >= js.position.size()) {
      return false;
    }
    state_.q(i) = js.position[it->second];
    
    if (it->second < js.velocity.size()) {
      state_.dq_meas(i) = js.velocity[it->second];
    }
  }
  return true;
}

// Forward kinematics
KDL::Frame CartesianController::computeFK(const Vec7& q) const {
  KDL::JntArray qk(kin_.chain.getNrOfJoints());
  std::copy_n(q.data(), kin_.chain.getNrOfJoints(), qk.data.data());
  
  KDL::Frame frame;
  kin_.fk_solver->JntToCart(qk, frame);
  return frame;
}

// Compute Jacobian (reordered: [linear; angular])
Mat6x7 CartesianController::computeJacobian(const Vec7& q) const {
  KDL::JntArray qk(kin_.chain.getNrOfJoints());
  std::copy_n(q.data(), kin_.chain.getNrOfJoints(), qk.data.data());
  
  KDL::Jacobian J(kin_.chain.getNrOfJoints());
  kin_.jac_solver->JntToJac(qk, J);
  
  // Reorder from KDL [angular; linear] to our format [linear; angular]
  Mat6x7 Je;
  Je.topRows<3>() = J.data.bottomRows<3>();
  Je.bottomRows<3>() = J.data.topRows<3>();
  return Je;
}

// Convert KDL frame to Eigen transform
Transform CartesianController::kdlToEigen(const KDL::Frame& frame) const {
  Transform T = Transform::Identity();
  T.translation() << frame.p.x(), frame.p.y(), frame.p.z();
  
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      T.linear()(i,j) = frame.M(i,j);
    }
  }
  return T;
}

// ===== SETUP FUNCTION (from notes) =====
// Called once when new target received - generates path and timing
void CartesianController::startPath(const Transform& T_now, const Transform& T_target) {
  // Store start and goal poses
  path_.p0_W = T_now.translation();
  path_.R0_W = T_now.linear();
  path_.pstar_W = T_target.translation();
  path_.Rstar_W = T_target.linear();
  
  // Define geometric path
  path_.dp = path_.pstar_W - path_.p0_W;
  path_.phi = so3Log(path_.R0_W.transpose() * path_.Rstar_W);
  
  // Calculate motion profile limits
  const double L = path_.dp.norm();
  const double theta = path_.phi.norm();
  
  const double sdot_lin = (L > DEFAULT_EPS) ? (limits_.v_lin_max / L) : 1e9;
  const double sddot_lin = (L > DEFAULT_EPS) ? (limits_.a_lin_max / L) : 1e9;
  const double sdot_ang = (theta > DEFAULT_EPS) ? (limits_.w_ang_max / theta) : 1e9;
  const double sddot_ang = (theta > DEFAULT_EPS) ? (limits_.alpha_ang_max / theta) : 1e9;
  
  // Configure trapezoidal profile
  path_.profile.configure(
    std::min(sdot_lin, sdot_ang),
    std::min(sddot_lin, sddot_ang)
  );
  path_.profile.start(now());
  path_.active = true;
}

// Check if robot reached target
bool CartesianController::checkConvergence(const Transform& T_now, double s) const {
  const Transform T_des = [this, s]() {
    Transform T = Transform::Identity();
    T.linear() = path_.R0_W * so3Exp(s * path_.phi);
    T.translation() = path_.p0_W + s * path_.dp;
    return T;
  }();
  
  const Vec3 p_err = (T_des.translation() - T_now.translation()).cwiseAbs();
  const Vec3 w_log = so3Log(T_now.linear().transpose() * T_des.linear()).cwiseAbs();
  
  return p_err.maxCoeff() <= POS_TOL && w_log.maxCoeff() <= ANG_TOL;
}

// Publish joint velocity command
void CartesianController::publishCommand(const Vec7& dq) {
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data.assign(dq.data(), dq.data() + 7);
  pub_cmd_->publish(cmd);
}

// Joint state callback - updates robot configuration
void CartesianController::onJointState(const sensor_msgs::msg::JointState::ConstSharedPtr msg) {
  if (!state_.initialized && !mapJointOrder(*msg)) {
    return;
  }
  state_.initialized = true;
  extractJointState(*msg);
}

// Point target callback (position only)
void CartesianController::onTargetPoint(const geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
  if (msg->header.frame_id != config_.root_link) {
    RCLCPP_WARN(get_logger(), "Frame mismatch: %s != %s", 
                msg->header.frame_id.c_str(), config_.root_link.c_str());
  }
  
  if (!state_.initialized) {
    RCLCPP_WARN(get_logger(), "No joint state available");
    return;
  }
  
  const Transform T_now = kdlToEigen(computeFK(state_.q));
  Transform T_target = Transform::Identity();
  T_target.linear() = T_now.linear();  // Keep current orientation
  T_target.translation() << msg->point.x, msg->point.y, msg->point.z;
  
  startPath(T_now, T_target);
  RCLCPP_INFO(get_logger(), "New point target received");
}

// Pose target callback (position and orientation)
void CartesianController::onTargetPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
  if (msg->header.frame_id != config_.root_link) {
    RCLCPP_WARN(get_logger(), "Frame mismatch: %s != %s",
                msg->header.frame_id.c_str(), config_.root_link.c_str());
  }
  
  if (!state_.initialized) {
    RCLCPP_WARN(get_logger(), "No joint state available");
    return;
  }
  
  const Transform T_now = kdlToEigen(computeFK(state_.q));
  
  Transform T_target = Transform::Identity();
  T_target.translation() << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  
  const Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x,
                             msg->pose.orientation.y, msg->pose.orientation.z);
  T_target.linear() = q.normalized().toRotationMatrix();
  
  startPath(T_now, T_target);
  RCLCPP_INFO(get_logger(), "New pose target received");
}

// ===== LOOP FUNCTION (from notes) =====
// Main control loop - runs at control_hz frequency
void CartesianController::onTimer() {
  if (!path_.active || !state_.initialized) return;
  
  // STEP 1: Read current robot state
  const Transform T_now = kdlToEigen(computeFK(state_.q));
  
  // STEP 2: Determine desired direction
  double s = 0.0, sdot = 0.0;
  path_.profile.eval(now(), s, sdot);
  
  // Calculate desired Cartesian velocity
  Vec6 V_path;
  V_path.head<3>() = sdot * path_.phi;  // Angular velocity
  V_path.tail<3>() = sdot * path_.dp;   // Linear velocity
  
  // Weight and normalize to unit direction
  Vec6 W_task;
  W_task << qp_params_.lambda_omega * V_path.head<3>(),
            qp_params_.lambda_v * V_path.tail<3>();
  
  const double norm = W_task.norm() + qp_params_.eps_u;
  const Vec6 u = (norm > 0.0) ? W_task / norm : Vec6::Zero();
  
  // STEP 3: Solve QP for optimal joint velocities
  const Mat6x7 J = computeJacobian(state_.q);
  const DiffIK_QP solver(qp_params_);
  const auto result = solver.solve(state_.q, state_.dq_meas, J, u, limits_);
  
  if (!result.ok) {
    publishCommand(Vec7::Zero());
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, 
                         "QP failed; holding position");
    return;
  }
  
  // STEP 4: Send command
  publishCommand(result.dq);
  
  // Publish Cartesian velocity for monitoring
  const Vec6 V_cartesian = J * result.dq;
  geometry_msgs::msg::TwistStamped twist_msg;
  twist_msg.header.stamp = now();
  twist_msg.header.frame_id = config_.root_link;
  twist_msg.twist.angular.x = V_cartesian(0);
  twist_msg.twist.angular.y = V_cartesian(1);
  twist_msg.twist.angular.z = V_cartesian(2);
  twist_msg.twist.linear.x = V_cartesian(3);
  twist_msg.twist.linear.y = V_cartesian(4);
  twist_msg.twist.linear.z = V_cartesian(5);
  pub_cartesian_vel_->publish(twist_msg);
  
  // STEP 5: Check convergence
  if (checkConvergence(T_now, s) && path_.profile.finished(now())) {
    path_.active = false;
    publishCommand(Vec7::Zero());
    RCLCPP_INFO(get_logger(), "Target reached successfully!");
  }
}

} // namespace cartesian_controller

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cartesian_controller::CartesianController>());
  rclcpp::shutdown();
  return 0;
}
