/* File Info 
 * Author:      BigDavid 
 * CreateTime:  2024/8/2 00:06:12 
 * LastEditor:  BigDavid 
 * ModifyTime:  2024/8/2 00:06:22 
 * Description: mpc_controller 定义
*/ 
//#include <osqp/osqp.h>
#include <casadi/casadi.hpp>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "eigen3/unsupported/Eigen/KroneckerProduct"
#include "eigen3/unsupported/Eigen/MatrixFunctions"

#include "nav2_mpc_controller/nav2_mpc_controller.hpp"

using nav2_util::declare_parameter_if_not_declared;
using namespace casadi;

namespace nav2_mpc_controller 
{
void MpcController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
                              std::shared_ptr<tf2_ros::Buffer> tf,
                              std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    auto node = parent.lock();
    node_ = parent;
    if (!node)
    {
        throw nav2_core::PlannerException("Unable to lock node!");
    }

    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    tf_ = tf;
    plugin_name_ = name;
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    // base parameters
    declare_parameter_if_not_declared(node, plugin_name_ + ".goal_dist_tolerance", rclcpp::ParameterValue(0.2));
    declare_parameter_if_not_declared(node, plugin_name_ + ".rotate_tolerance", rclcpp::ParameterValue(0.5));
    declare_parameter_if_not_declared(node, plugin_name_ + ".base_frame", rclcpp::ParameterValue(base_frame_));
    declare_parameter_if_not_declared(node, plugin_name_ + ".map_frame", rclcpp::ParameterValue(map_frame_));
    node->get_parameter(plugin_name_ + ".goal_dist_tolerance", goal_dist_tol_);
    node->get_parameter(plugin_name_ + ".rotate_tolerance", rotate_tol_);
    node->get_parameter(plugin_name_ + ".base_frame", base_frame_);
    node->get_parameter(plugin_name_ + ".map_frame", map_frame_);

    // lookahead parameters
    declare_parameter_if_not_declared(node, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.5));
    declare_parameter_if_not_declared(node, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.3));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(0.9));
    node->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
    node->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
    node->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);

    // linear velocity parameters
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_v", rclcpp::ParameterValue(0.5));
    declare_parameter_if_not_declared(node, plugin_name_ + ".min_v", rclcpp::ParameterValue(0.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_v_inc", rclcpp::ParameterValue(0.5));
    node->get_parameter(plugin_name_ + ".max_v", max_v_);
    node->get_parameter(plugin_name_ + ".min_v", min_v_);
    node->get_parameter(plugin_name_ + ".max_v_inc", max_v_inc_);

    // angular velocity parameters
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_w", rclcpp::ParameterValue(1.57));
    declare_parameter_if_not_declared(node, plugin_name_ + ".min_w", rclcpp::ParameterValue(0.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_w_inc", rclcpp::ParameterValue(1.57));
    node->get_parameter(plugin_name_ + ".max_w", max_w_);
    node->get_parameter(plugin_name_ + ".min_w", min_w_);
    node->get_parameter(plugin_name_ + ".max_w_inc", max_w_inc_);

    // iteration for Ricatti solution parameters
    declare_parameter_if_not_declared(node, plugin_name_ + ".predicting_time_domain", rclcpp::ParameterValue(4));
    declare_parameter_if_not_declared(node, plugin_name_ + ".control_time_domain", rclcpp::ParameterValue(4));
    node->get_parameter(plugin_name_ + ".predicting_time_domain", p_);
    node->get_parameter(plugin_name_ + ".control_time_domain", m_);

    // Weight matrix for penalizing state error while tracking [x, y, theta]
    std::vector<double> diag_vec;
    Q_ = Eigen::Matrix3d::Zero();
    node->get_parameter("Q_matrix_diag", diag_vec);
    for (size_t i = 0; i < diag_vec.size(); ++i) {
        Q_(i, i) = diag_vec[i];
    }

    // Weight matrix for penalizing input error while tracking [v, w]
    node->get_parameter("R_matrix_diag", diag_vec);
    R_ = Eigen::Matrix2d::Zero();
    for (size_t i = 0; i < diag_vec.size(); ++i) {
        R_(i, i) = diag_vec[i];
    }

    double controller_frequency;
    node->get_parameter("controller_frequency", controller_frequency);
    d_t_ = 1.0 / controller_frequency;

    target_pt_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("/target_point", 10);
    current_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("/current_pose", 10);

    RCLCPP_INFO(node->get_logger(), "MPC planner initialized!");

    // Initialize collision checker and set costmap
    collision_checker_ = std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*>>(costmap_);
    collision_checker_->setCostmap(costmap_);
    initialized_ = true;
}

void MpcController::cleanup()
{
  RCLCPP_INFO(logger_,
              "Cleaning up controller: %s of type"
              "nav2_mpc_controller::MpcController",
              plugin_name_.c_str());
  target_pt_pub_.reset();
  current_pose_pub_.reset();
}

void MpcController::activate()
{
  RCLCPP_INFO(logger_,
              "Activating controller: %s of type "
              "nav2_mpc_controller::MpcController",
              plugin_name_.c_str());
  target_pt_pub_->on_activate();
  current_pose_pub_->on_activate();
  // Add callback for dynamic parameters
  auto node = node_.lock();
}

void MpcController::deactivate()
{
  RCLCPP_INFO(logger_,
              "Deactivating controller: %s of type "
              "nav2_mpc_controller::MpcController",
              plugin_name_.c_str());
  target_pt_pub_->on_deactivate();
  current_pose_pub_->on_deactivate();
}

void MpcController::setPlan(const nav_msgs::msg::Path& path)
{
    if (!initialized_)
    {
        RCLCPP_ERROR(logger_, "This planner has not been initialized, please call initialize() before using this planner");
    }

    RCLCPP_INFO(logger_, "Got new plan");

    // Set new plan
    global_plan_ = path;

    // Reset plan parameters
    if (goal_x_ != global_plan_.poses.back().pose.position.x || goal_y_ != global_plan_.poses.back().pose.position.y)
    {
        goal_x_ = global_plan_.poses.back().pose.position.x;
        goal_y_ = global_plan_.poses.back().pose.position.y;
        goal_rpy_ = getEulerAngles(global_plan_.poses.back());
        goal_reached_ = false;
    }
}

void MpcController::setSpeedLimit(const double& speed_limit, const bool& percentage)
{
    RCLCPP_INFO(logger_, "empty");
}

geometry_msgs::msg::TwistStamped MpcController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                                                                        const geometry_msgs::msg::Twist& speed,
                                                                        nav2_core::GoalChecker* goal_checker)
{
    std::lock_guard<std::mutex> lock_reinit(mutex_);
    nav2_costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

    // Update for the current goal checker's state
    geometry_msgs::msg::Pose pose_tolerance;
    geometry_msgs::msg::Twist vel_tolerance;
    if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance))
    {
        RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
    }
    else
    {
        goal_dist_tol_ = pose_tolerance.position.x;
    }

    // get robot position in map
    geometry_msgs::msg::PoseStamped current_ps_map, target_ps_map;
    transformPose(map_frame_, pose, current_ps_map);

    // prune the global plan
    std::vector<geometry_msgs::msg::PoseStamped> prune_plan = prune(current_ps_map);

    // calculate look-ahead distance
    double vt = std::hypot(speed.linear.x, speed.linear.y);
    double wt = speed.angular.z;
    double L = getLookAheadDistance(vt);

    // get the particular point on the path at the lookahead distance
    geometry_msgs::msg::PointStamped lookahead_pt;
    double theta_trj, kappa;
    getLookAheadPoint(L, current_ps_map, prune_plan, lookahead_pt, theta_trj, kappa);

    target_ps_map.pose.position.x = lookahead_pt.point.x;
    target_ps_map.pose.position.y = lookahead_pt.point.y;
    
    // current angle
    double theta = tf2::getYaw(current_ps_map.pose.orientation);  // [-pi, pi]

    // position reached
    geometry_msgs::msg::TwistStamped cmd_vel;
    if (shouldRotateToGoal(current_ps_map, global_plan_.poses.back()))
    {
        du_p_ = Eigen::Vector2d(0, 0);
        double e_theta = regularizeAngle(goal_rpy_.z() - theta);

        // orientation reached
        if (!shouldRotateToPath(std::fabs(e_theta)))
        {
            cmd_vel.twist.linear.x = 0.0;
            cmd_vel.twist.angular.z = 0.0;
            goal_reached_ = true;
        }
        // orientation not reached
        else
        {
            cmd_vel.twist.linear.x = 0.0;
            cmd_vel.twist.angular.z = angularRegularization(speed.angular.z, e_theta / d_t_);
        }
    }
    // posistion not reached
    else
    {
        Eigen::Vector3d s(current_ps_map.pose.position.x, current_ps_map.pose.position.y, theta);    // current state
        Eigen::Vector3d s_d(target_ps_map.pose.position.x, target_ps_map.pose.position.y, theta_trj);  // desired state
        Eigen::Vector2d u_r(vt, regularizeAngle(vt * kappa));                                                                 // refered input
        Eigen::Vector2d u = mpcControl(s, s_d, u_r, du_p_);

        du_p_ = Eigen::Vector2d(linearRegularization(std::hypot(speed.linear.x, speed.linear.y), u[0]) - u_r[0], regularizeAngle(angularRegularization(speed.angular.z, u[1]) - u_r[1]));

        cmd_vel.twist.linear.x = linearRegularization(std::hypot(speed.linear.x, speed.linear.y), u[0]);
        cmd_vel.twist.angular.z = angularRegularization(speed.angular.z, u[1]);
    }

    // publish next target_ps_map pose
    target_ps_map.header.frame_id = "map";
    target_ps_map.header.stamp = current_ps_map.header.stamp;
    target_pt_pub_->publish(target_ps_map);

    // publish robot pose
    current_ps_map.header.frame_id = "map";
    current_ps_map.header.stamp = current_ps_map.header.stamp;
    current_pose_pub_->publish(current_ps_map);

    // populate and return message
    cmd_vel.header = pose.header;

    return cmd_vel;
}

Eigen::Vector2d MpcController::mpcControl(Eigen::Vector3d s, Eigen::Vector3d s_d, Eigen::Vector2d u_r, Eigen::Vector2d du_p) {
    int dim_u = 2;
    int dim_x = 3;

    // state vector (5 x 1)
    Eigen::VectorXd x = Eigen::VectorXd(dim_x + dim_u);
    x.topLeftCorner(dim_x, 1) = s - s_d;
    x[2] = regularizeAngle(x[2]);
    x.bottomLeftCorner(dim_u, 1) = du_p;

    // original state matrix
    Eigen::Matrix3d A_o = Eigen::Matrix3d::Identity();
    A_o(0, 2) = -u_r[0] * sin(s_d[2]) * d_t_;
    A_o(1, 2) = u_r[0] * cos(s_d[2]) * d_t_;

    // original control matrix
    Eigen::MatrixXd B_o = Eigen::MatrixXd::Zero(dim_x, dim_u);
    B_o(0, 0) = cos(s_d[2]) * d_t_;
    B_o(1, 0) = sin(s_d[2]) * d_t_;
    B_o(2, 1) = d_t_;

    // state matrix (5 x 5)
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(dim_x + dim_u, dim_x + dim_u);
    A.topLeftCorner(dim_x, dim_x) = A_o;
    A.topRightCorner(dim_x, dim_u) = B_o;
    A.bottomLeftCorner(dim_u, dim_x) = Eigen::MatrixXd::Zero(dim_u, dim_x);
    A.bottomRightCorner(dim_u, dim_u) = Eigen::Matrix2d::Identity();

    // control matrix (5 x 2)
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(dim_x + dim_u, dim_u);
    B.topLeftCorner(dim_x, dim_u) = B_o;
    B.bottomLeftCorner(dim_u, dim_u) = Eigen::Matrix2d::Identity();

    // output matrix(3 x 5)
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_x, dim_x + dim_u);
    C.topLeftCorner(dim_x, dim_x) = Eigen::Matrix3d::Identity();
    C.topRightCorner(dim_x, dim_u) = Eigen::MatrixXd::Zero(dim_x, dim_u);

    // mpc state matrix(3p x 5)
    Eigen::MatrixPower<Eigen::MatrixXd> A_pow(A);
    Eigen::MatrixXd S_x = Eigen::MatrixXd::Zero(dim_x * p_, dim_x + dim_u);
    for (int i = 0; i < p_; i++)
        S_x.middleRows(dim_x * i, dim_x) = C * A_pow(i + 1);
    
    // mpc control matrix(3p x 2m)
    Eigen::MatrixXd S_u = Eigen::MatrixXd::Zero(dim_x * p_, dim_u * m_);
    for (int i = 0; i < p_; i++)
    {
        for (int j = 0; j < m_; j++)
        {
        if (j <= i)
            S_u.block(dim_x * i, dim_u * j, dim_x, dim_u) = C * A_pow(i - j) * B;
        else
            S_u.block(dim_x * i, dim_u * j, dim_x, dim_u) = Eigen::MatrixXd::Zero(dim_x, dim_u);
        }
    }

    // optimization
    // min 1/2 * x.T * P * x + q.T * x
    // s.t. l <= Ax <= u
    Eigen::VectorXd Yr = Eigen::VectorXd::Zero(dim_x * p_);                              // (3p x 1)
    Eigen::MatrixXd Q = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(p_, p_), Q_);  // (3p x 3p)
    Eigen::MatrixXd R = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(m_, m_), R_);  // (2m x 2m)
    Eigen::MatrixXd P = S_u.transpose() * Q * S_u + R;                                   // (2m x 2m)
    Eigen::VectorXd q = S_u.transpose() * Q * (S_x * x - Yr);                            // (2m x 1)

    // boundary
    Eigen::Vector2d u_min(min_v_, -max_w_);
    Eigen::Vector2d u_max(max_v_, max_w_);
    Eigen::Vector2d u_k_1(du_p[0], du_p[1]);
    Eigen::Vector2d du_min(-0.2, -0.2);
    Eigen::Vector2d du_max(0.2, M_PI);
    Eigen::VectorXd U_min = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), u_min);    // (2m x 1)
    Eigen::VectorXd U_max = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), u_max);    // (2m x 1)
    Eigen::VectorXd U_r = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), u_r);        // (2m x 1)
    Eigen::VectorXd U_k_1 = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), u_k_1);    // (2m x 1)
    Eigen::VectorXd dU_min = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), du_min);  // (2m x 1)
    Eigen::VectorXd dU_max = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), du_max);  // (2m x 1)

    // constriants
    Eigen::VectorXd lower = Eigen::VectorXd::Zero(2 * dim_u * m_);  // (4m x 1)
    Eigen::VectorXd upper = Eigen::VectorXd::Zero(2 * dim_u * m_);  // (4m x 1)
    lower.topRows(dim_u * m_) = U_min - U_k_1 - U_r;
    lower.bottomRows(dim_u * m_) = dU_min;
    upper.topRows(dim_u * m_) = U_max - U_k_1 - U_r;
    upper.bottomRows(dim_u * m_) = dU_max;

    // Calculate kernel
    std::vector<c_float> P_data;
    std::vector<c_int> P_indices;
    std::vector<c_int> P_indptr;
    int ind_P = 0;
    for (int col = 0; col < dim_u * m_; ++col)
    {
        P_indptr.push_back(ind_P);
        for (int row = 0; row <= col; ++row)
        {
            P_data.push_back(P(row, col));
            // P_data.push_back(P(row, col) * 2.0);
            P_indices.push_back(row);
            ind_P++;
        }
    }
    P_indptr.push_back(ind_P);

    // Calculate affine constraints (4m x 2m)
    std::vector<c_float> A_data;
    std::vector<c_int> A_indices;
    std::vector<c_int> A_indptr;
    int ind_A = 0;
    A_indptr.push_back(ind_A);
    for (int j = 0; j < m_; ++j)
    {
        for (int n = 0; n < dim_u; ++n)
        {
            for (int row = dim_u * j + n; row < dim_u * m_; row += dim_u)
            {
                A_data.push_back(1.0);
                A_indices.push_back(row);
                ++ind_A;
            }
            A_data.push_back(1.0);
            A_indices.push_back(dim_u * m_ + dim_u * j + n);
            ++ind_A;
            A_indptr.push_back(ind_A);
        }
    }

    // Calculate offset
    std::vector<c_float> q_data;
    for (int row = 0; row < dim_u * m_; ++row)
    {
        q_data.push_back(q(row, 0));
    }

    // Calculate constraints
    std::vector<c_float> lower_bounds;
    std::vector<c_float> upper_bounds;
    for (int row = 0; row < 2 * dim_u * m_; row++)
    {
        lower_bounds.push_back(lower(row, 0));
        upper_bounds.push_back(upper(row, 0));
    }

    // solve
    OSQPWorkspace* work = nullptr;
    OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
    OSQPSettings* settings = reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
    if (settings) osqp_set_default_settings(settings);

    settings->verbose = false;
    settings->warm_start = true;

    data->n = dim_u * m_;
    data->m = 2 * dim_u * m_;
    data->P = csc_matrix(data->n, data->n, P_data.size(), P_data.data(), P_indices.data(), P_indptr.data());
    data->q = q.data();
    data->A = csc_matrix(data->m, data->n, A_data.size(), A_data.data(), A_indices.data(), A_indptr.data());
    data->l = lower_bounds.data();
    data->u = upper_bounds.data();

    //osqp_setup(&work, data, settings);
    osqp_setup(&work, data, settings);
    osqp_solve(work);
    auto status = work->info->status_val;

    if (status < 0)
    {
        std::cout << "failed optimization status:\t" << work->info->status;
        return Eigen::Vector2d::Zero();
    }

    if (status != 1 && status != 2)
    {
        std::cout << "failed optimization status:\t" << work->info->status;
        return Eigen::Vector2d::Zero();
    }

    Eigen::Vector2d u(work->solution->x[0] + du_p[0] + u_r[0], regularizeAngle(work->solution->x[1] + du_p[1] + u_r[1]));

    // Cleanup
    osqp_cleanup(work);
    c_free(data->A);
    c_free(data->P);
    c_free(data);
    c_free(settings);

    return u;
}

} // namespace nav2_mpc_controller


// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(nav2_mpc_controller::MpcController, nav2_core::Controller)