#include "ur_cartesian_controller/cartesian_controller.hpp"

namespace cartesian_controller {

// ======= SO(3) Helper Functions =======
// Computes the log map from SO(3) to so(3): extracts axis-angle from rotation matrix
// Returns phi = axis * angle (3D vector representing rotation)
Eigen::Vector3d so3Log(const Eigen::Matrix3d& R) {
  double cos_theta = (R.trace() - 1.0) * 0.5;
  cos_theta = std::min(1.0, std::max(-1.0, cos_theta));
  double theta = std::acos(cos_theta);
  if (theta < 1e-12) return Eigen::Vector3d::Zero();
  Eigen::Vector3d axis;
  axis << R(2,1)-R(1,2), R(0,2)-R(2,0), R(1,0)-R(0,1);
  axis /= (2.0*std::sin(theta));
  return axis * theta;
}

// Computes the exponential map from so(3) to SO(3): creates rotation matrix from axis-angle
// Takes phi (axis * angle) and returns rotation matrix R
Eigen::Matrix3d so3Exp(const Eigen::Vector3d& phi) {
  double theta = phi.norm();
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  if (theta < 1e-12) return I;
  Eigen::Vector3d a = phi / theta;
  Eigen::Matrix3d ax;
  ax <<     0, -a.z(),  a.y(),
        a.z(),     0, -a.x(),
       -a.y(),  a.x(),     0;
  return I + std::sin(theta)*ax + (1-std::cos(theta))*(ax*ax);
}

// ======= Trapezoidal Velocity Profile =======
// This class handles the TIMING of the motion along the geometric path.
// It creates a smooth acceleration-cruise-deceleration profile for the path parameter s(t).
// The profile ensures the robot smoothly accelerates to max speed, cruises, then decelerates to stop.
void TrapProfile::configure(double sdot_max_in, double sddot_max_in) {
  sdot_max = std::max(1e-6, sdot_max_in);   // Max velocity along path
  sddot_max = std::max(1e-6, sddot_max_in); // Max acceleration along path
  
  // Check if we can reach max velocity or if motion is triangular (no cruise phase)
  double crit = (sdot_max*sdot_max)/sddot_max;
  if (crit > 1.0) {
    // Triangular profile: accelerate then immediately decelerate (can't reach max velocity)
    triangular = true;
    ta = std::sqrt(1.0 / sddot_max);  // Time to reach peak velocity
    tc = 0.0;                          // No cruise time
    T  = 2.0 * ta;                     // Total motion time
  } else {
    // Trapezoidal profile: accelerate -> cruise at max velocity -> decelerate
    triangular = false;
    ta = sdot_max / sddot_max;        // Acceleration time
    tc = (1.0 - (sdot_max*sdot_max)/sddot_max) / sdot_max;  // Cruise time
    T  = 2.0*ta + tc;                  // Total motion time
  }
}

void TrapProfile::start(const rclcpp::Time& now) { t0 = now; }
bool TrapProfile::finished(const rclcpp::Time& now) const { return (now - t0).seconds() >= T; }

// Evaluates the profile at current time, returning:
// - s: position along path [0,1]
// - sdot: velocity along path
void TrapProfile::eval(const rclcpp::Time& now, double& s, double& sdot) const {
  double t = (now - t0).seconds();
  if (t <= 0.0) { s=0.0; sdot=0.0; return; }
  if (t >= T)   { s=1.0; sdot=0.0; return; }
  
  if (triangular) {
    // Triangular profile: accelerate then decelerate
    double ta_ = ta;
    if (t < ta_) {
      // Acceleration phase
      sdot = sddot_max * t;
      s = 0.5*sddot_max*t*t;
    } else {
      // Deceleration phase
      sdot = sddot_max * (T - t);
      double dt = T - t;
      s = 1.0 - 0.5*sddot_max*dt*dt;
    }
  } else {
    // Trapezoidal profile: accelerate -> cruise -> decelerate
    if (t < ta) {
      // Acceleration phase
      sdot = sddot_max * t;
      s = 0.5*sddot_max*t*t;
    } else if (t < ta + tc) {
      // Cruise phase at max velocity
      sdot = sdot_max;
      s = 0.5*sddot_max*ta*ta + sdot_max*(t - ta);
    } else {
      // Deceleration phase
      sdot = sddot_max * (T - t);
      double dt = T - t;
      s = 1.0 - 0.5*sddot_max*dt*dt;
    }
  }
}

// ======= Differential IK Quadratic Program Solver =======
// This class formulates and solves the QP that finds optimal joint velocities
// while respecting physical constraints and secondary objectives

// Computes damped right pseudo-inverse of Jacobian
// Damping prevents instability near singularities by avoiding infinite joint velocities
Eigen::Matrix<double,7,6> DiffIK_QP::pinvRightDamped_(const Eigen::Matrix<double,6,7>& J) const {
  Eigen::JacobiSVD<Eigen::Matrix<double,6,7>> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix<double,7,6> V = svd.matrixV().leftCols<6>();
  Eigen::Matrix<double,6,6> U = svd.matrixU();
  Eigen::Array<double,6,1> S = svd.singularValues();
  Eigen::Matrix<double,6,6> Splus = Eigen::Matrix<double,6,6>::Zero();
  
  // Damped pseudo-inverse: si/(si^2 + damping^2) prevents division by small singular values
  for (int i=0;i<6;++i) {
    double si = S(i);
    Splus(i,i) = si / (si*si + P_.pinv_damping*P_.pinv_damping);
  }
  return V * Splus * U.transpose();
}

// Builds joint velocity bounds that respect position, velocity, and acceleration limits
// This is crucial for hardware safety - prevents hitting joint limits or demanding impossible accelerations
void DiffIK_QP::buildBounds_(const Eigen::Matrix<double,7,1>& q,
                             const Eigen::Matrix<double,7,1>& dq_meas,
                             const Limits& lim,
                             Eigen::Matrix<double,7,1>& lb,
                             Eigen::Matrix<double,7,1>& ub) const {
  for (int i=0;i<7;++i) {
    // Three types of bounds that must ALL be satisfied:
    
    // 1. Direct velocity limits (hardware maximum speeds)
    double b1_lo = lim.dqmin(i), b1_hi = lim.dqmax(i);
    
    // 2. Position-based velocity limits (slow down to avoid hitting joint limits)
    // If we're close to a position limit, we must limit velocity to avoid overshooting
    double b2_lo = (lim.qmin(i) - q(i)) / P_.h;  // Max velocity toward lower limit
    double b2_hi = (lim.qmax(i) - q(i)) / P_.h;  // Max velocity toward upper limit
    
    // 3. Acceleration-based velocity limits (smooth motion, avoid jerky commands)
    // New velocity must be achievable given current velocity and max acceleration
    double b3_lo = dq_meas(i) + P_.h * lim.ddqmin(i);  // Min reachable velocity
    double b3_hi = dq_meas(i) + P_.h * lim.ddqmax(i);  // Max reachable velocity
    
    // Take the intersection of all constraints (most restrictive wins)
    lb(i) = std::max(std::max(b1_lo, b2_lo), b3_lo);
    ub(i) = std::min(std::min(b1_hi, b2_hi), b3_hi);
    
    // Safety check: if bounds are infeasible, clamp to midpoint
    if (lb(i) > ub(i)) {
      double mid = 0.5*(lb(i)+ub(i));
      lb(i) = mid; ub(i) = mid;
    }
  }
}

// Main QP solver - this is the "brain" that finds optimal joint velocities
QPResult DiffIK_QP::solve(const Eigen::Matrix<double,7,1>& q,
                          const Eigen::Matrix<double,7,1>& dq_meas,
                          const Eigen::Matrix<double,6,7>& J,
                          const Eigen::Matrix<double,6,1>& u,  // Unit direction vector (desired motion direction)
                          const Limits& lim) {
  QPResult out;

  // ===== STEP 1: Compute Nullspace Projector =====
  // This allows us to use the robot's redundancy (7 DOF for 6 DOF task)
  // to achieve secondary objectives without affecting the primary task
  Eigen::Matrix<double,7,6> Jplus = pinvRightDamped_(J);
  Eigen::Matrix<double,7,7> Pn = Eigen::Matrix<double,7,7>::Identity() - Jplus * J;

  // ===== STEP 2: Setup QP Problem =====
  // Decision variables: z = [dq(7); alpha(1); w(6)]
  // - dq: joint velocities to command (what we're solving for)
  // - alpha: speed scaling factor [0,1] (the "gas pedal")
  // - w: slack variables for path deviation (only used when necessary)
  const int n = 14;      // Total decision variables
  const int m_eq = 6;    // Equality constraints (task space equation)
  const int m_ineq = 8;  // Inequality constraints (7 joint bounds + 1 alpha bound)
  const int m = m_eq + m_ineq;

  // ===== STEP 3: Build Quadratic Objective (H matrix and f vector) =====
  // Objective: min 0.5*z'*H*z + f'*z
  
  // Quadratic term for joint velocities (minimize joint effort + maintain posture)
  Eigen::Matrix<double,7,7> Wj2 = P_.Wj.cwiseProduct(P_.Wj).asDiagonal();  // Square the weights
  Eigen::Matrix<double,7,7> Hqq = Wj2 + P_.lambda_ns * (Pn.transpose() * Pn);

  // Linear term for posture maintenance (pulls joints toward comfortable q0)
  Eigen::Matrix<double,7,1> dq_lin = P_.lambda_ns * (Pn.transpose() * Pn) * (P_.Kp.cwiseProduct(q - P_.q0));

  // Build sparse H matrix (quadratic coefficients)
  Eigen::SparseMatrix<double> H(n,n);
  std::vector<Eigen::Triplet<double>> Ht;
  // Joint velocity quadratic terms (objectives 1 & 2: minimize effort + maintain posture)
  for (int i=0;i<7;++i) for (int j=i;j<7;++j) if (std::abs(Hqq(i,j))>0.0) Ht.emplace_back(i,j,Hqq(i,j));
  // Small regularization on alpha to ensure QP is strictly convex
  Ht.emplace_back(7,7,P_.H_reg);
  // Slack penalty (objective 3: heavily penalize path deviation)
  for (int i=0;i<6;++i) Ht.emplace_back(8+i,8+i,P_.rho);
  H.setFromTriplets(Ht.begin(), Ht.end());

  // Build f vector (linear coefficients)
  Eigen::VectorXd f(n); f.setZero();
  for (int i=0;i<7;++i) f(i) = dq_lin(i);  // Posture restoration term
  f(7) = -P_.kappa;  // Objective 4: maximize speed (negative because we minimize)

  // ===== STEP 4: Build Constraints (A matrix and bounds) =====
  // Task constraint: J*dq = alpha*u + w
  // This says: "joint velocities should produce motion in direction u, scaled by alpha,
  // with slack w allowing deviation when necessary"
  
  Eigen::SparseMatrix<double> A(m,n);
  std::vector<Eigen::Triplet<double>> At;
  
  // Equality constraints: J*dq - alpha*u - w = 0
  for (int r=0;r<6;++r) {
    // Jacobian terms
    for (int c=0;c<7;++c) if (std::abs(J(r,c))>0.0) At.emplace_back(r,c,J(r,c));
    // Speed scaling term (negative because on RHS)
    At.emplace_back(r,7,-u(r));
    // Slack term (negative because on RHS)
    At.emplace_back(r,8+r,-1.0);
  }
  
  // Inequality constraints: bounds on joint velocities and alpha
  Eigen::Matrix<double,7,1> lb_dq, ub_dq;
  buildBounds_(q, dq_meas, lim, lb_dq, ub_dq);  // Get safe velocity bounds
  for (int r=0;r<7;++r) At.emplace_back(m_eq + r, r, 1.0);  // Identity for joint bounds
  At.emplace_back(m_eq + 7, 7, 1.0);  // Identity for alpha bound
  A.setFromTriplets(At.begin(), At.end());

  // Set constraint bounds
  Eigen::VectorXd l(m), uvec(m);
  l.segment(0,6).setZero();     // Equality constraints: = 0
  uvec.segment(0,6).setZero();
  for (int i=0;i<7;++i) { l(6+i)=lb_dq(i); uvec(6+i)=ub_dq(i); }  // Joint velocity bounds
  l(13)=0.0; uvec(13)=1.0;  // Alpha bounds: [0,1]

  // ===== STEP 5: Solve QP with OSQP =====
  OsqpEigen::Solver solver;
  solver.settings()->setWarmStart(true);      // Use previous solution as initial guess
  solver.settings()->setAdaptiveRho(true);    // Adapt penalty parameter
  solver.settings()->setAlpha(1.6);           // Relaxation parameter
  solver.settings()->setPolish(true);         // Polish solution for accuracy
  solver.settings()->setVerbosity(false);

  solver.data()->setNumberOfVariables(n);
  solver.data()->setNumberOfConstraints(m);
  solver.data()->setHessianMatrix(H);
  solver.data()->setGradient(f);
  solver.data()->setLinearConstraintsMatrix(A);
  solver.data()->setLowerBound(l);
  solver.data()->setUpperBound(uvec);
  
  if (!solver.initSolver()) {
    return out;  // Failed to initialize
  }

  if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return out;
  
  // Extract solution
  Eigen::VectorXd z = solver.getSolution();
  out.dq = z.segment<7>(0);     // Optimal joint velocities
  out.alpha = z(7);              // Speed scaling achieved
  out.w = z.segment<6>(8);       // Path deviation (ideally zero)
  out.ok = true;
  return out;
}

// ======= ROS2 Node Implementation =======
CartesianController::CartesianController(const rclcpp::NodeOptions& opt)
: rclcpp::Node("cartesian_controller", opt)
{
  // ===== Parameter Declaration =====
  // Core parameters
  this->declare_parameter<std::string>("urdf_param", urdf_param_);
  this->declare_parameter<std::string>("root_link", root_link_);
  this->declare_parameter<std::string>("tip_link", tip_link_);
  this->declare_parameter<int>("control_hz", control_hz_);
  
  // Task space weighting (balance between rotation and translation importance)
  this->declare_parameter<double>("lambda_omega", lambda_omega_);
  this->declare_parameter<double>("lambda_v", lambda_v_);

  // QP objective weights
  std::vector<double> Wjv(7,1.0), Kv(7,0.5), q0v(7,0.0);
  this->declare_parameter<std::vector<double>>("Wj", Wjv);            // Joint effort weights
  this->declare_parameter<double>("lambda_ns", qp_.lambda_ns);        // Nullspace objective weight
  this->declare_parameter<double>("rho", qp_.rho);                    // Path tracking weight (high = stay on path)
  this->declare_parameter<double>("kappa", qp_.kappa);                // Speed maximization weight
  this->declare_parameter<std::vector<double>>("Kp", Kv);             // Posture restoration gains
  this->declare_parameter<std::vector<double>>("q0", q0v);            // Comfortable posture
  this->declare_parameter<double>("eps_u", qp_.eps_u);                // Numerical stability for unit vector
  this->declare_parameter<double>("pinv_damping", qp_.pinv_damping);  // Singularity damping
  this->declare_parameter<double>("H_reg", qp_.H_reg);                // QP regularization

  // Physical limits
  std::vector<double> qmin(7,-1e9), qmax(7,1e9), dqmin(7,-1.5), dqmax(7,1.5), ddqmin(7,-5.0), ddqmax(7,5.0);
  this->declare_parameter<std::vector<double>>("qmin", qmin);         // Joint position limits
  this->declare_parameter<std::vector<double>>("qmax", qmax);
  this->declare_parameter<std::vector<double>>("dqmin", dqmin);       // Joint velocity limits
  this->declare_parameter<std::vector<double>>("dqmax", dqmax);
  this->declare_parameter<std::vector<double>>("ddqmin", ddqmin);     // Joint acceleration limits
  this->declare_parameter<std::vector<double>>("ddqmax", ddqmax);
  this->declare_parameter<double>("v_lin_max", lim_.v_lin_max);       // Cartesian linear velocity limit
  this->declare_parameter<double>("a_lin_max", lim_.a_lin_max);       // Cartesian linear acceleration limit
  this->declare_parameter<double>("w_ang_max", lim_.w_ang_max);       // Cartesian angular velocity limit
  this->declare_parameter<double>("alpha_ang_max", lim_.alpha_ang_max); // Cartesian angular acceleration limit
  this->declare_parameter<std::vector<std::string>>("joint_names", joint_names_);

  // ===== Get Parameters =====
  this->get_parameter("urdf_param", urdf_param_);
  this->get_parameter("root_link", root_link_);
  this->get_parameter("tip_link", tip_link_);
  this->get_parameter("control_hz", control_hz_);
  this->get_parameter("lambda_omega", lambda_omega_);
  this->get_parameter("lambda_v", lambda_v_);

  this->get_parameter("Wj", Wjv); for (int i=0;i<7;++i) qp_.Wj(i)=Wjv[i];
  this->get_parameter("lambda_ns", qp_.lambda_ns);
  this->get_parameter("rho", qp_.rho);
  this->get_parameter("kappa", qp_.kappa);
  this->get_parameter("Kp", Kv); for (int i=0;i<7;++i) qp_.Kp(i)=Kv[i];
  this->get_parameter("q0", q0v); for (int i=0;i<7;++i) qp_.q0(i)=q0v[i];
  this->get_parameter("eps_u", qp_.eps_u);
  this->get_parameter("pinv_damping", qp_.pinv_damping);
  this->get_parameter("H_reg", qp_.H_reg);

  this->get_parameter("qmin", qmin); for (int i=0;i<7;++i) lim_.qmin(i)=qmin[i];
  this->get_parameter("qmax", qmax); for (int i=0;i<7;++i) lim_.qmax(i)=qmax[i];
  this->get_parameter("dqmin", dqmin); for (int i=0;i<7;++i) lim_.dqmin(i)=dqmin[i];
  this->get_parameter("dqmax", dqmax); for (int i=0;i<7;++i) lim_.dqmax(i)=dqmax[i];
  this->get_parameter("ddqmin", ddqmin); for (int i=0;i<7;++i) lim_.ddqmin(i)=ddqmin[i];
  this->get_parameter("ddqmax", ddqmax); for (int i=0;i<7;++i) lim_.ddqmax(i)=ddqmax[i];
  this->get_parameter("v_lin_max", lim_.v_lin_max);
  this->get_parameter("a_lin_max", lim_.a_lin_max);
  this->get_parameter("w_ang_max", lim_.w_ang_max);
  this->get_parameter("alpha_ang_max", lim_.alpha_ang_max);
  this->get_parameter("joint_names", joint_names_);

  // Calculate control timestep
  qp_.h = 1.0 / std::max(1, control_hz_);
  qp_.lambda_omega = lambda_omega_;
  qp_.lambda_v = lambda_v_;

  // Initialize KDL kinematics from URDF
  if (!initKDLfromURDF_()) {
    RCLCPP_FATAL(get_logger(), "KDL init failed.");
    throw std::runtime_error("KDL init failed");
  }

  // ===== Setup ROS2 Communication =====
  // Publisher for joint velocity commands
  pub_cmd_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/forward_velocity_controller/commands", rclcpp::SystemDefaultsQoS());

  // Publisher for Cartesian velocity (for monitoring/debugging)
  pub_cartesian_vel_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/cartesian_controller/cartesian_velocity", rclcpp::SystemDefaultsQoS());

  // Subscriber for current joint states
  sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&CartesianController::onJointState, this, std::placeholders::_1));

  // Subscribers for target commands (point or full pose)
  sub_point_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/cartesian_controller/target_point", rclcpp::SystemDefaultsQoS(),
      std::bind(&CartesianController::onTargetPoint, this, std::placeholders::_1));

  sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/cartesian_controller/target_pose", rclcpp::SystemDefaultsQoS(),
      std::bind(&CartesianController::onTargetPose, this, std::placeholders::_1));

  // Main control loop timer
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(int(1000.0 / std::max(1, control_hz_))),
      std::bind(&CartesianController::onTimer, this));

  RCLCPP_INFO(get_logger(), "QP-based Cartesian controller initialized. root=%s tip=%s, %d Hz",
              root_link_.c_str(), tip_link_.c_str(), control_hz_);
}

bool CartesianController::initKDLfromURDF_() {
  // Get robot description from robot_state_publisher
  auto param_client = std::make_shared<rclcpp::SyncParametersClient>(
    this, "/robot_state_publisher");

  param_client->wait_for_service(std::chrono::seconds(5));

  auto params = param_client->get_parameters({"robot_description"});
  std::string urdf_xml = params[0].as_string();

  // Build KDL chain for kinematics
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(urdf_xml, tree)) return false;
  if (!tree.getChain(root_link_, tip_link_, chain_)) return false;
  
  // Create forward kinematics and Jacobian solvers
  fk_solver_  = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
  jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
  return true;
}

// Maps joint names from joint_states message to indices
bool CartesianController::mapJointOrder_(const sensor_msgs::msg::JointState& js) {
  name_to_idx_.clear();
  for (size_t i=0;i<js.name.size();++i) name_to_idx_[js.name[i]] = i;

  if (joint_names_.empty()) {
    joint_names_.assign(CONTROLLED_JOINTS.begin(), CONTROLLED_JOINTS.end());
  }

  for (auto& n: joint_names_) {
    if (!name_to_idx_.count(n)) {
      RCLCPP_ERROR(get_logger(), "Joint '%s' not in joint_states", n.c_str());
      return false;
    }
  }
  return true;
}

// Extracts joint positions in correct order from joint_states message
bool CartesianController::extractQ_(const sensor_msgs::msg::JointState& js, Eigen::Matrix<double,7,1>& qout) {
  for (int i=0;i<7;++i) {
    auto it = name_to_idx_.find(joint_names_[i]);
    if (it==name_to_idx_.end() || it->second>=js.position.size()) return false;
    qout(i) = js.position[it->second];
  }
  return true;
}

// Forward kinematics: joint positions -> end-effector frame
KDL::Frame CartesianController::fk_(const Eigen::Matrix<double,7,1>& q) const {
  KDL::JntArray qk(chain_.getNrOfJoints());
  for (unsigned i=0;i<chain_.getNrOfJoints();++i) qk(i)=q(i);
  KDL::Frame F;
  fk_solver_->JntToCart(qk, F);
  return F;
}

// Computes Jacobian matrix (maps joint velocities to Cartesian velocities)
Eigen::Matrix<double,6,7> CartesianController::jac_(const Eigen::Matrix<double,7,1>& q) const {
  KDL::JntArray qk(chain_.getNrOfJoints());
  for (unsigned i=0;i<chain_.getNrOfJoints();++i) qk(i)=q(i);
  KDL::Jacobian J(chain_.getNrOfJoints());
  jac_solver_->JntToJac(qk, J);
  
  // Reorder from KDL format [angular; linear] to our format [linear; angular]
  Eigen::Matrix<double,6,7> Je;
  for (int c=0;c<7;++c) {
    Je(0,c) = J(3,c); Je(1,c)=J(4,c); Je(2,c)=J(5,c);  // Linear velocity
    Je(3,c) = J(0,c); Je(4,c)=J(1,c); Je(5,c)=J(2,c);  // Angular velocity
  }
  return Je;
}

// Callback for joint state updates - keeps track of current robot configuration
void CartesianController::onJointState(const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (!have_js_) {
    if (!mapJointOrder_(*msg)) return;
    have_js_=true;
  }
  if (!extractQ_(*msg, q_)) return;
  
  // Also extract joint velocities for acceleration limits
  for (int i=0;i<7;++i) {
    auto it = name_to_idx_.find(joint_names_[i]);
    if (it!=name_to_idx_.end() && it->second < msg->velocity.size())
      dq_meas_(i) = msg->velocity[it->second];
    else
      dq_meas_(i)=0.0;
  }
}

// ===== SETUP FUNCTION (from notes) =====
// This is called once when a new target is received
// It generates the complete geometric path and timing profile
void CartesianController::startPathFromCurrent_(const Eigen::Isometry3d& T_now, const Eigen::Isometry3d& T_star) {
  // Store start and goal poses
  p0_W_    = T_now.translation();
  R0_W_    = T_now.linear();
  pstar_W_ = T_star.translation();
  Rstar_W_ = T_star.linear();
  
  // Define geometric path:
  // - Position: straight line from p0 to pstar
  // - Orientation: direct interpolation in SO(3) using log/exp maps
  dp_      = pstar_W_ - p0_W_;  // Translation direction
  phi_     = so3Log(R0_W_.transpose() * Rstar_W_);  // Rotation axis-angle

  // Create motion profile (timing along the path)
  // Calculate max speed/acceleration based on Cartesian limits
  double L = dp_.norm();      // Total translation distance
  double theta = phi_.norm();  // Total rotation angle
  
  // Scale limits by path length to get limits for path parameter s
  double sdot_lin = (L>1e-12) ? (lim_.v_lin_max / L) : 1e9;
  double sddot_lin= (L>1e-12) ? (lim_.a_lin_max / L) : 1e9;
  double sdot_ang = (theta>1e-12)? (lim_.w_ang_max / theta) : 1e9;
  double sddot_ang= (theta>1e-12)? (lim_.alpha_ang_max / theta) : 1e9;
  
  // Use most restrictive limits
  double sdot_max = std::min(sdot_lin, sdot_ang);
  double sddot_max= std::min(sddot_lin, sddot_ang);
  
  // Configure trapezoidal profile with these limits
  prof_.configure(sdot_max, sddot_max);
  prof_.start(this->now());
  active_goal_ = true;
}

// Checks if robot has reached the target within tolerance
bool CartesianController::convergenceCheck_(const Eigen::Isometry3d& T_now, double s) const {
  // Compute desired pose at current path parameter s
  Eigen::Isometry3d Tdes = Eigen::Isometry3d::Identity();
  Tdes.linear() = R0_W_ * so3Exp(s * phi_);
  Tdes.translation() = p0_W_ + s * dp_;

  // Check position and orientation errors
  Eigen::Vector3d perr = Tdes.translation() - T_now.translation();
  Eigen::Vector3d wlog = so3Log(T_now.linear().transpose() * Tdes.linear());

  double pos_ok = perr.cwiseAbs().maxCoeff();
  double ang_ok = wlog.cwiseAbs().maxCoeff();
  return (pos_ok <= 1e-3) && (ang_ok <= 1e-2);  // Within 1mm and ~0.5 degrees
}

// Handles new point target (position only, keeps current orientation)
void CartesianController::onTargetPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  if (msg->header.frame_id != root_link_) {
    RCLCPP_WARN(get_logger(), "frame_id='%s' != root_link='%s'",
                msg->header.frame_id.c_str(), root_link_.c_str());
  }
  if (!have_js_) {
    RCLCPP_WARN(get_logger(), "No joint state yet.");
    return;
  }

  // Get current end-effector pose
  KDL::Frame Fk = fk_(q_);
  Eigen::Isometry3d Tnow = Eigen::Isometry3d::Identity();
  Tnow.translation() = Eigen::Vector3d(Fk.p.x(), Fk.p.y(), Fk.p.z());
  Eigen::Matrix3d R;
  R << Fk.M(0,0),Fk.M(0,1),Fk.M(0,2),
       Fk.M(1,0),Fk.M(1,1),Fk.M(1,2),
       Fk.M(2,0),Fk.M(2,1),Fk.M(2,2);
  Tnow.linear() = R;

  // Create target with new position but same orientation
  Eigen::Isometry3d Tstar = Eigen::Isometry3d::Identity();
  Tstar.linear() = R;  // Keep current orientation
  Tstar.translation() = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z);

  startPathFromCurrent_(Tnow, Tstar);
  RCLCPP_INFO(get_logger(), "New point target received.");
}

// Handles new pose target (position and orientation)
void CartesianController::onTargetPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (msg->header.frame_id != root_link_) {
    RCLCPP_WARN(get_logger(), "frame_id='%s' != root_link='%s'",
                msg->header.frame_id.c_str(), root_link_.c_str());
  }
  if (!have_js_) {
    RCLCPP_WARN(get_logger(), "No joint state yet.");
    return;
  }

  // Get current end-effector pose
  KDL::Frame Fk = fk_(q_);
  Eigen::Isometry3d Tnow = Eigen::Isometry3d::Identity();
  Tnow.translation() = Eigen::Vector3d(Fk.p.x(), Fk.p.y(), Fk.p.z());
  Eigen::Matrix3d R;
  R << Fk.M(0,0),Fk.M(0,1),Fk.M(0,2),
       Fk.M(1,0),Fk.M(1,1),Fk.M(1,2),
       Fk.M(2,0),Fk.M(2,1),Fk.M(2,2);
  Tnow.linear() = R;

  // Create target with specified position and orientation
  Eigen::Isometry3d Tstar = Eigen::Isometry3d::Identity();
  Tstar.translation() = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x,
                       msg->pose.orientation.y, msg->pose.orientation.z);
  Tstar.linear() = q.normalized().toRotationMatrix();

  startPathFromCurrent_(Tnow, Tstar);
  RCLCPP_INFO(get_logger(), "New pose target received.");
}

// ===== LOOP FUNCTION (from notes) =====
// Main control loop - runs at control_hz frequency
void CartesianController::onTimer() {
  if (!active_goal_ || !have_js_) return;

  // STEP 1: Read current robot state
  KDL::Frame Fk = fk_(q_);
  Eigen::Isometry3d Tnow = Eigen::Isometry3d::Identity();
  Tnow.translation() = Eigen::Vector3d(Fk.p.x(), Fk.p.y(), Fk.p.z());
  Eigen::Matrix3d R;
  R << Fk.M(0,0),Fk.M(0,1),Fk.M(0,2),
       Fk.M(1,0),Fk.M(1,1),Fk.M(1,2),
       Fk.M(2,0),Fk.M(2,1),Fk.M(2,2);
  Tnow.linear() = R;

  // STEP 2: Determine desired direction
  // Get current position and velocity along path from trapezoidal profile
  double s=0.0, sdot=0.0;
  prof_.eval(this->now(), s, sdot);

  // Calculate desired Cartesian velocity along the path
  Eigen::Matrix<double,6,1> Vpath;
  Vpath.head<3>() = sdot * phi_;  // Angular velocity
  Vpath.tail<3>() = sdot * dp_;   // Linear velocity

  // Weight and normalize to get unit direction vector
  // This represents the instantaneous direction the end-effector should move
  Eigen::Matrix<double,6,1> Wtask;
  Wtask << qp_.lambda_omega*Vpath.head<3>(), qp_.lambda_v*Vpath.tail<3>();
  double denom = Wtask.norm() + qp_.eps_u;  // eps_u prevents division by zero
  Eigen::Matrix<double,6,1> u;
  if (denom > 0.0) {
    u = Wtask / denom;
  } else {
    u = Eigen::Matrix<double,6,1>::Zero();
  }

  // STEP 3: Formulate and solve the QP
  // Get current Jacobian
  Eigen::Matrix<double,6,7> J = jac_(q_);

  // Solve for optimal joint velocities
  DiffIK_QP solver(qp_);
  QPResult res = solver.solve(q_, dq_meas_, J, u, lim_);
  
  if (!res.ok) {
    // QP failed - stop motion for safety
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.assign(7,0.0);
    pub_cmd_->publish(cmd);
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000, "QP failed; holding position.");
    return;
  }

  // STEP 4: Send command to robot
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data.resize(7);
  for (int i=0;i<7;++i) cmd.data[i] = res.dq(i);
  pub_cmd_->publish(cmd);

  // Also publish Cartesian velocity for monitoring
  Eigen::Matrix<double,6,1> V_cartesian = J * res.dq;
  geometry_msgs::msg::TwistStamped twist_msg;
  twist_msg.header.stamp = this->now();
  twist_msg.header.frame_id = root_link_;
  // Note: KDL Jacobian format is [angular; linear]
  twist_msg.twist.angular.x = V_cartesian(0);
  twist_msg.twist.angular.y = V_cartesian(1);
  twist_msg.twist.angular.z = V_cartesian(2);
  twist_msg.twist.linear.x = V_cartesian(3);
  twist_msg.twist.linear.y = V_cartesian(4);
  twist_msg.twist.linear.z = V_cartesian(5);
  pub_cartesian_vel_->publish(twist_msg);

  // STEP 5: Check if we've reached the goal
  if (convergenceCheck_(Tnow, s) && prof_.finished(this->now())) {
    active_goal_ = false;
    // Send zero velocities to stop
    std_msgs::msg::Float64MultiArray stop;
    stop.data.assign(7,0.0);
    pub_cmd_->publish(stop);
    RCLCPP_INFO(get_logger(), "Target reached successfully!");
  }
}

} // namespace cartesian_controller

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cartesian_controller::CartesianController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
