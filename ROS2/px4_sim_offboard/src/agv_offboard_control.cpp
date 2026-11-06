#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <px4_ros_com/frame_transforms.h>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Geometry>


#include <std_msgs/msg/float64.hpp>
#include <unordered_map>

#include <eliko_messages/msg/distances.hpp>
#include <eliko_messages/msg/distances_list.hpp>

#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <sstream>
#include <map>
#include <vector>
#include <array>
#include <mutex>
#include <cmath>
#include <limits>
#include <algorithm>
#include <cctype>
#include <stdexcept>

#include <random> 

#include "ament_index_cpp/get_package_share_directory.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;


static inline double wrapPi(double a) {
    while (a >  M_PI) a -= 2.0*M_PI;
    while (a < -M_PI) a += 2.0*M_PI;
    return a;
}

static std::vector<std::string> sensorKeysFromIds(
    const std::vector<std::string> &ids,
    const std::string &prefix)
{
    std::vector<std::string> keys;
    keys.reserve(ids.size());

    for (size_t i = 0; i < ids.size(); ++i) {
        const auto &raw_id = ids[i];
        if (raw_id.empty()) {
            continue;
        }

        const bool is_numeric = std::all_of(
            raw_id.begin(),
            raw_id.end(),
            [](unsigned char c) { return std::isdigit(c) != 0; });

        if (is_numeric) {
            keys.push_back(prefix + raw_id);
            continue;
        }

        if (raw_id.size() > prefix.size() &&
            raw_id.rfind(prefix, 0) == 0 &&
            std::all_of(raw_id.begin() + prefix.size(), raw_id.end(),
                        [](unsigned char c) { return std::isdigit(c) != 0; })) {
            keys.push_back(raw_id);
            continue;
        }

        keys.push_back(prefix + std::to_string(i + 1));
    }

    return keys;
}

static std::vector<std::string> directSensorKeysFromPrefixes(
    const std::vector<std::string> &prefixes,
    const std::string &ns_prefix,
    const std::string &sensor_prefix)
{
    std::vector<std::string> keys;
    const std::string prefix = ns_prefix + ".";
    for (const auto &entry : prefixes) {
        if (entry.rfind(prefix, 0) != 0) {
            continue;
        }
        const std::string sensor_id = entry.substr(prefix.size());
        const bool is_numeric = std::all_of(
            sensor_id.begin(),
            sensor_id.end(),
            [](unsigned char c) { return std::isdigit(c) != 0; });
        if (!is_numeric) {
            continue;
        }
        keys.push_back(sensor_prefix + sensor_id);
    }

    std::sort(keys.begin(), keys.end());
    keys.erase(std::unique(keys.begin(), keys.end()), keys.end());
    return keys;
}

class AGVOffboardControl : public rclcpp::Node
{
public:
    AGVOffboardControl();

private:

	static inline double yaw_from_quat_enu(const Eigen::Quaterniond &q)
	{
		Eigen::Vector3d eul = q.toRotationMatrix().eulerAngles(2, 1, 0);
		return eul[0];
	}

    void timer_callback();
    void load_trajectory(const std::string &filename);
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void publish_distances();
    bool loadSensorIdsFromArrayParam(const std::string &param_name,
                                     const std::string &prefix,
                                     std::unordered_map<std::string, std::string> &out_map,
                                     std::vector<std::string> &out_keys);
    bool parseAnchorTagKey(const std::string &key, std::string &anchor_key, std::string &tag_key) const;

    // Retrieve parameter values
    std::string offboard_control_mode_topic_;
    std::string trajectory_setpoint_topic_;
    std::string vehicle_command_topic_;
    std::string vehicle_odometry_topic_;
    std::string vehicle_local_position_topic_;
    std::string ros_odometry_topic_, ros_gt_topic_;
    std::string traj_file_;
    std::string frame_prefix_{"agv"};
    std::string marker_array_topic_{"/agv/gt_marker_array"};
    std::string distances_list_topic_{"eliko/Distances"};
    std::string world_frame_id_{"map"};
    uint8_t vehicle_target_system_ = 3;
    
    double lookahead_distance_, cruise_speed_;
    double odom_error_position_ = 0.0; // %,  meters for every 100 meters traveled
	double odom_error_angle_ = 0.0; // % degrees for every 100
    size_t current_index_ = 0;
    size_t offboard_counter_ = 0;

    // --- Odometry drift state (add to private:) ---
    bool have_prev_odom_{false};
    Eigen::Vector3d last_pos_true_enu_{0.0, 0.0, 0.0};
	Eigen::Quaterniond q_last_true_enu_{1,0,0,0}; // add as a private member
    double last_yaw_true_{0.0};     // ENU yaw

    Eigen::Vector3d pos_noisy_enu_{0.0, 0.0, 0.0};

    double yaw_noisy_{0.0};

    // scale biases (percent -> unitless scale)
    double pos_scale_err_{0.0};  // = odom_error_position_ / 100.0
    double yaw_scale_err_{0.0};  // = odom_error_angle_   / 100.0

    // Add near other private members:
    std::mt19937 rng_;
    std::normal_distribution<double> odom_noise_{0.0, 1.0};

    std::vector<std::array<double, 4>> trajectory_;
    std::array<double, 4> current_pose_ = {0.0, 0.0, 0.0, 0.0}; // x, y, z, yaw
    std::mutex pose_mutex_;

    Eigen::Vector3d origin_pos_enu_{0.0, 0.0, 0.0};
	Eigen::Quaterniond origin_q_enu_{1.0, 0.0, 0.0, 0.0}; // w,x,y,z

    // Replace/augment your index state with these:
    size_t closest_idx_ = 0;
    size_t current_target_idx_ = 0;

    int last_xy_reset_counter_{-1};
    int last_z_reset_counter_{-1};
    Eigen::Vector2d xy_reset_accum_{0.0, 0.0};
    double z_reset_accum_{0.0};

    rclcpp::TimerBase::SharedPtr timer_, distances_timer_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ros_odometry_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ros_gt_publisher_;

    rclcpp::Publisher<eliko_messages::msg::DistancesList>::SharedPtr distances_list_pub_;

	std::vector<geometry_msgs::msg::Pose> gt_pose_history_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;

    rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_subscriber_;
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_subscriber_;


    std::unordered_map<std::string, double> uwb_distances_;
    std::mutex uwb_mutex_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> uwb_subscribers_;
    std::vector<std::string> uwb_topic_names_;

    std::unordered_map<std::string, std::string> anchor_ids_;
    std::unordered_map<std::string, std::string> tag_ids_;
    std::vector<std::string> anchor_keys_, tag_keys_;

    rclcpp::Time start_time_;
    bool started_ = false;

    // In class AGVOffboardControl private:
    int    last_heading_reset_counter_{-1};
    double heading_reset_accum_{0.0};   // accumulated delta_heading across resets
    int    last_odom_reset_counter_{-1};

    // Odometry gating/zeroing
    bool map_aligned_{false};

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

};

AGVOffboardControl::AGVOffboardControl()
    : Node(
        "agv_offboard_control",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
{
    // Load parameters
    this->get_parameter("offboard_control_mode_topic", offboard_control_mode_topic_);
    this->get_parameter("trajectory_setpoint_topic", trajectory_setpoint_topic_);
    this->get_parameter("vehicle_command_topic", vehicle_command_topic_);
    this->get_parameter("vehicle_odometry_topic", vehicle_odometry_topic_);
    this->get_parameter("vehicle_local_position_topic", vehicle_local_position_topic_);
    this->get_parameter("trajectory_csv_file", traj_file_);
    this->get_parameter("ros_odometry_topic", ros_odometry_topic_);
    this->get_parameter("ros_gt_topic", ros_gt_topic_);
    int vehicle_target_system = 3;
    this->get_parameter("vehicle_target_system", vehicle_target_system);
    vehicle_target_system_ = static_cast<uint8_t>(vehicle_target_system);
    this->get_parameter("frame_prefix", frame_prefix_);
    this->get_parameter("marker_array_topic", marker_array_topic_);
    this->get_parameter("distances_list_topic", distances_list_topic_);
    this->get_parameter("world_frame_id", world_frame_id_);

    this->get_parameter("odom_error_position", odom_error_position_);
    this->get_parameter("odom_error_angle", odom_error_angle_);

    pos_scale_err_ = odom_error_position_ / 100.0;
    yaw_scale_err_ = odom_error_angle_   / 100.0;

    // Seed RNG for Gaussian noise
    rng_ = std::mt19937(std::random_device{}());

    this->get_parameter("lookahead_distance", lookahead_distance_);
    this->get_parameter("cruise_speed", cruise_speed_);


    std::vector<double> agv_origin_vec;
		this->get_parameter("agv_origin", agv_origin_vec);
		if (agv_origin_vec.size() != 6) {
			RCLCPP_WARN(this->get_logger(),
						"agv_origin must have 6 elements [x,y,z,roll,pitch,yaw]; using zeros");
		} else {
			origin_pos_enu_ = Eigen::Vector3d(agv_origin_vec[0], agv_origin_vec[1], agv_origin_vec[2]);

			// RPY are given in ENU. Build quaternion qz(yaw)*qy(pitch)*qx(roll).
			const double roll  = agv_origin_vec[3];
			const double pitch = agv_origin_vec[4];
			const double yaw   = agv_origin_vec[5];

			origin_q_enu_ =
				Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
				Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
				Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX());
			}


    const bool loaded_dynamic_tags = loadSensorIdsFromArrayParam("tags.ids", "t", tag_ids_, tag_keys_);

    const auto listed = this->list_parameters({"anchors"}, 10);
    anchor_keys_ = directSensorKeysFromPrefixes(listed.prefixes, "anchors", "a");
    anchor_ids_.clear();
    for (const auto &anchor_key : anchor_keys_) {
        anchor_ids_[anchor_key] = anchor_key.size() > 1 ? anchor_key.substr(1) : anchor_key;
    }

    // tags.ids lists every tag in the fleet. This UGV's own anchors may be none.
    if (!loaded_dynamic_tags) {
        RCLCPP_FATAL(this->get_logger(), "Invalid UWB config. Required param: tags.ids.");
        throw std::runtime_error("Missing required tag ID parameters.");
    }

    uwb_topic_names_.clear();
    for (const auto &anchor_key : anchor_keys_) {
        for (const auto &tag_key : tag_keys_) {
            uwb_topic_names_.push_back("/uwb_gz_simulator/distances/" + anchor_key + tag_key);
        }
    }

    RCLCPP_INFO(this->get_logger(),
                "Configured %zu anchors and %zu tags (%zu UWB topics).",
                anchor_keys_.size(), tag_keys_.size(), uwb_topic_names_.size());

    std::string package_path = ament_index_cpp::get_package_share_directory("px4_sim_offboard");
    std::string full_csv_path = package_path + "/trajectories/" + traj_file_;
    load_trajectory(full_csv_path);
    if (trajectory_.empty()) {
        RCLCPP_FATAL(this->get_logger(), "No trajectory points were loaded from %s", full_csv_path.c_str());
        throw std::runtime_error("Missing AGV trajectory data.");
    }

    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(offboard_control_mode_topic_, 10);
    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(trajectory_setpoint_topic_, 10);
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>(vehicle_command_topic_, 10);

    ros_odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(ros_odometry_topic_, 10);
    ros_gt_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(ros_gt_topic_, 10);
    distances_list_pub_ = this->create_publisher<eliko_messages::msg::DistancesList>(distances_list_topic_, 10);

    marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_array_topic_, 10);

    start_time_ = this->get_clock()->now();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
   
    // Publish static transforms for this robot's tags and anchors (relative to
    // agv/base_link). It may carry either, or both.
    for (const auto &[ns, key_prefix] : {std::pair<std::string, std::string>{"tags", "t"}, {"anchors", "a"}}) {
        const auto sensors = this->list_parameters({ns}, 10);
        for (const auto &key : directSensorKeysFromPrefixes(sensors.prefixes, ns, key_prefix)) {
            std::vector<double> sensor_pos;
            const std::string compact_param = ns + "." + key.substr(1) + ".position";
            if (this->get_parameter(compact_param, sensor_pos) && sensor_pos.size() == 3) {
                geometry_msgs::msg::TransformStamped tf;
                tf.header.stamp = this->get_clock()->now();
                tf.header.frame_id = frame_prefix_ + "/base_link";
                tf.child_frame_id = frame_prefix_ + "/" + key;
                tf.transform.translation.x = sensor_pos[0];
                tf.transform.translation.y = sensor_pos[1];
                tf.transform.translation.z = sensor_pos[2];
                tf.transform.rotation.x = 0.0;
                tf.transform.rotation.y = 0.0;
                tf.transform.rotation.z = 0.0;
                tf.transform.rotation.w = 1.0;
                static_tf_broadcaster_->sendTransform(tf);
            }
        }
    }
   
    vehicle_odometry_subscriber_ = this->create_subscription<VehicleOdometry>(
    vehicle_odometry_topic_, rclcpp::SensorDataQoS(),
    [this](VehicleOdometry::SharedPtr msg) {

        if (msg->pose_frame != VehicleOdometry::POSE_FRAME_NED) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Unsupported AGV pose_frame %u. Expected POSE_FRAME_NED.",
                msg->pose_frame);
            return;
        }

        // PX4 reset deltas shift the estimator frame. Subtract the accumulated
        // shift so this callback sees a continuous trajectory in startup coordinates.
        Eigen::Vector3d pos_ned(
            msg->position[0] - xy_reset_accum_.x(),
            msg->position[1] - xy_reset_accum_.y(),
            msg->position[2] - z_reset_accum_);
        Eigen::Vector3d pos_enu = px4_ros_com::frame_transforms::ned_to_enu_local_frame(pos_ned);

        Eigen::Quaterniond q_ned_raw(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
        const Eigen::Vector3d rpy_ned_raw = q_ned_raw.toRotationMatrix().eulerAngles(2, 1, 0);
        const double yaw_ned_cont = wrapPi(rpy_ned_raw[0] - heading_reset_accum_);
        Eigen::Quaterniond q_ned =
            Eigen::AngleAxisd(yaw_ned_cont, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rpy_ned_raw[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rpy_ned_raw[2], Eigen::Vector3d::UnitX());
        Eigen::Quaterniond q_enu = px4_ros_com::frame_transforms::px4_to_ros_orientation(q_ned);

        // True ENU yaw from quaternion
        const Eigen::Vector3d rpy_true = q_enu.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX
        const double yaw_true = rpy_true[0];
        const double pitch_true = rpy_true[1];
        const double roll_true  = rpy_true[2];

        if (last_odom_reset_counter_ < 0) {
            last_odom_reset_counter_ = msg->reset_counter;
        } else if (msg->reset_counter != last_odom_reset_counter_) {
            last_odom_reset_counter_ = msg->reset_counter;
            last_pos_true_enu_ = pos_enu;
            q_last_true_enu_ = q_enu;
            last_yaw_true_ = yaw_true;
            return;
        }

        if (!have_prev_odom_) {

            last_pos_true_enu_ = pos_enu;
            q_last_true_enu_   = q_enu;
            last_yaw_true_     = yaw_true;

            // Zero-base odom from the first valid sample.
            pos_noisy_enu_.setZero();
            yaw_noisy_ = 0.0;

            have_prev_odom_ = true;
        } else {
            // 1) True world increment
            Eigen::Vector3d dpos_true_enu = pos_enu - last_pos_true_enu_;

            // 2) Express it in last *true* body frame
            Eigen::Matrix3d R_last_true_enu = q_last_true_enu_.toRotationMatrix();
            Eigen::Vector3d dpos_true_body = R_last_true_enu.transpose() * dpos_true_enu;

            // 3) Add body-frame noise / scale (your per-axis % is fine here)
            Eigen::Vector3d sigma_body(
                pos_scale_err_ * std::abs(dpos_true_body.x()),
                pos_scale_err_ * std::abs(dpos_true_body.y()),
                pos_scale_err_ * std::abs(dpos_true_body.z()));
            Eigen::Vector3d dpos_noisy_body = dpos_true_body + Eigen::Vector3d(
                sigma_body.x() * odom_noise_(rng_),
                sigma_body.y() * odom_noise_(rng_),
                sigma_body.z() * odom_noise_(rng_));

            // Yaw increment (still Z-only if you want): 
            double dyaw_true = wrapPi(yaw_true - last_yaw_true_);
            double dyaw_noisy = dyaw_true + (yaw_scale_err_ * std::abs(dyaw_true)) * odom_noise_(rng_);

            // 4) Update the *estimated* orientation first
            yaw_noisy_ = wrapPi(yaw_noisy_ + dyaw_noisy);

            // If you want yaw-only attitude, build R_est from yaw_noisy_ and (optionally) keep true roll/pitch:
            Eigen::Matrix3d R_est_enu =
                (Eigen::AngleAxisd(yaw_noisy_, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(pitch_true, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(roll_true,  Eigen::Vector3d::UnitX())).toRotationMatrix();

            // 5) Rotate the **noisy local** increment into world and integrate position
            pos_noisy_enu_ += R_est_enu * dpos_noisy_body;
            
            // Save current true for next step
            last_pos_true_enu_ = pos_enu;
            q_last_true_enu_   = q_enu;
            last_yaw_true_ = yaw_true;
        }

        Eigen::Quaterniond q_noisy_enu =
            Eigen::AngleAxisd(yaw_noisy_, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch_true, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll_true,  Eigen::Vector3d::UnitX());

        // --- Publish Odometry with DRIFTED pose ---
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->get_clock()->now();

        odom_msg.header.frame_id = frame_prefix_ + "/odom";
        odom_msg.child_frame_id  = frame_prefix_ + "/base_link";

        // DRIFTED position
        odom_msg.pose.pose.position.x = pos_noisy_enu_.x();
        odom_msg.pose.pose.position.y = pos_noisy_enu_.y();
        odom_msg.pose.pose.position.z = pos_noisy_enu_.z();

        // DRIFTED orientation (yaw only)
        odom_msg.pose.pose.orientation.x = q_noisy_enu.x();
        odom_msg.pose.pose.orientation.y = q_noisy_enu.y();
        odom_msg.pose.pose.orientation.z = q_noisy_enu.z();
        odom_msg.pose.pose.orientation.w = q_noisy_enu.w();

        // Twists should be in child (base_link, FLU).
        Eigen::Vector3d v_child_flu = Eigen::Vector3d::Zero();
        if (msg->velocity_frame == VehicleOdometry::VELOCITY_FRAME_NED) {
            Eigen::Vector3d vel_ned(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
            Eigen::Vector3d vel_enu = px4_ros_com::frame_transforms::ned_to_enu_local_frame(vel_ned);
            v_child_flu = q_noisy_enu.inverse() * vel_enu;
        } else if (msg->velocity_frame == VehicleOdometry::VELOCITY_FRAME_BODY_FRD) {
            v_child_flu = Eigen::Vector3d(msg->velocity[0], -msg->velocity[1], -msg->velocity[2]);
        } else {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Unsupported AGV velocity_frame %u. Publishing zero linear twist.",
                msg->velocity_frame);
        }
        odom_msg.twist.twist.linear.x = v_child_flu.x();
        odom_msg.twist.twist.linear.y = v_child_flu.y();
        odom_msg.twist.twist.linear.z = v_child_flu.z();

        // PX4 angular velocity is BODY_FRD, convert to ROS child-frame FLU.
        odom_msg.twist.twist.angular.x = msg->angular_velocity[0];
        odom_msg.twist.twist.angular.y = -msg->angular_velocity[1];
        odom_msg.twist.twist.angular.z = -msg->angular_velocity[2];

        // Covariances: start from PX4 and (optionally) inflate to reflect the synthetic drift
        odom_msg.pose.covariance = {
            msg->position_variance[0], 0, 0, 0, 0, 0,
            0, msg->position_variance[1], 0, 0, 0, 0,
            0, 0, msg->position_variance[2], 0, 0, 0,
            0, 0, 0, msg->orientation_variance[0], 0, 0,
            0, 0, 0, 0, msg->orientation_variance[1], 0,
            0, 0, 0, 0, 0, msg->orientation_variance[2]
        };

        odom_msg.twist.covariance = {
            msg->velocity_variance[0], 0, 0, 0, 0, 0,
            0, msg->velocity_variance[1], 0, 0, 0, 0,
            0, 0, msg->velocity_variance[2], 0, 0, 0,
            0, 0, 0, 0.0, 0, 0,
            0, 0, 0, 0, 0.0, 0,
            0, 0, 0, 0, 0, 0.0
        };

        ros_odometry_publisher_->publish(odom_msg);
    });

    vehicle_local_position_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        vehicle_local_position_topic_, rclcpp::SensorDataQoS(),
        [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {


            // Handle heading reset accumulation
            if (last_heading_reset_counter_ < 0) {
                last_heading_reset_counter_ = msg->heading_reset_counter;
            }
            if (msg->heading_reset_counter != last_heading_reset_counter_) {
                heading_reset_accum_ += msg->delta_heading;
                last_heading_reset_counter_ = msg->heading_reset_counter;
            }

            if (last_xy_reset_counter_ < 0) last_xy_reset_counter_ = msg->xy_reset_counter;
            if (last_z_reset_counter_  < 0) last_z_reset_counter_  = msg->z_reset_counter;

            if (msg->xy_reset_counter != last_xy_reset_counter_) {
                xy_reset_accum_.x() += msg->delta_xy[0];
                xy_reset_accum_.y() += msg->delta_xy[1];
                last_xy_reset_counter_ = msg->xy_reset_counter;
            }
            if (msg->z_reset_counter != last_z_reset_counter_) {
                z_reset_accum_ += msg->delta_z;
                last_z_reset_counter_ = msg->z_reset_counter;
            }

            // Keep ground-truth pose continuous by undoing estimator frame resets.
            Eigen::Vector3d pos_ned(
						msg->x - xy_reset_accum_.x(),
						msg->y - xy_reset_accum_.y(),
						msg->z - z_reset_accum_
            );

            Eigen::Vector3d pos_enu_local =
                px4_ros_com::frame_transforms::ned_to_enu_local_frame(pos_ned);

            // Now compose world/map pose
            Eigen::Vector3d pos_enu_world = origin_q_enu_ * pos_enu_local + origin_pos_enu_;

            // Compensate heading resets in NED before converting to ENU.
            const double heading_ned = wrapPi(msg->heading - heading_reset_accum_);
            Eigen::Quaterniond q_heading_ned(Eigen::AngleAxisd(heading_ned, Eigen::Vector3d::UnitZ()));
            Eigen::Quaterniond q_heading_enu =
                px4_ros_com::frame_transforms::px4_to_ros_orientation(q_heading_ned);
            Eigen::Quaterniond q_world_enu = origin_q_enu_ * q_heading_enu;

            // Extract world ENU yaw
            double yaw_world_enu = wrapPi(yaw_from_quat_enu(q_world_enu));

             // Store current_pose_ (for your controller)
            {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                current_pose_[0] = pos_enu_world.x();
                current_pose_[1] = pos_enu_world.y();
                current_pose_[2] = pos_enu_world.z();
                current_pose_[3] = yaw_world_enu;
            }

            // Publish PoseStamped (gt)
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = this->get_clock()->now();
            pose_msg.header.frame_id = world_frame_id_;
            pose_msg.pose.position.x = pos_enu_world.x();
            pose_msg.pose.position.y = pos_enu_world.y();
            pose_msg.pose.position.z = pos_enu_world.z();
            pose_msg.pose.orientation.x = q_world_enu.x();
            pose_msg.pose.orientation.y = q_world_enu.y();
            pose_msg.pose.orientation.z = q_world_enu.z();
            pose_msg.pose.orientation.w = q_world_enu.w();
            ros_gt_publisher_->publish(pose_msg);

            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = pose_msg.header.stamp;
            tf_msg.header.frame_id = world_frame_id_;
            tf_msg.child_frame_id = frame_prefix_ + "/base_link";
            tf_msg.transform.translation.x = pos_enu_world.x();
            tf_msg.transform.translation.y = pos_enu_world.y();
            tf_msg.transform.translation.z = pos_enu_world.z();
            tf_msg.transform.rotation = pose_msg.pose.orientation;
            tf_broadcaster_->sendTransform(tf_msg);

            // Publish marker array
            gt_pose_history_.push_back(pose_msg.pose);
            if (gt_pose_history_.size() > 500.0) {
                gt_pose_history_.erase(gt_pose_history_.begin());
            }
            visualization_msgs::msg::MarkerArray marker_array;
            for (size_t i = 0; i < gt_pose_history_.size(); ++i) {
                visualization_msgs::msg::Marker marker;
                marker.header = pose_msg.header;
                marker.ns = frame_prefix_ + "_gt";
                marker.id = i;
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose = gt_pose_history_[i];
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0;
                marker.lifetime = rclcpp::Duration(0,0); // forever
                marker_array.markers.push_back(marker);
            }
            marker_array_pub_->publish(marker_array);

        });

    timer_ = this->create_wall_timer(100ms, std::bind(&AGVOffboardControl::timer_callback, this));

    distances_timer_ = this->create_wall_timer(100ms, std::bind(&AGVOffboardControl::publish_distances, this));

    for (const auto& topic : uwb_topic_names_) {
        auto sub = this->create_subscription<std_msgs::msg::Float64>(
            topic, 10,
            [this, topic = topic](const std_msgs::msg::Float64::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(this->uwb_mutex_);
                this->uwb_distances_[topic] = msg->data;
            });
        uwb_subscribers_.push_back(sub);
    }

}

bool AGVOffboardControl::loadSensorIdsFromArrayParam(
    const std::string &param_name,
    const std::string &prefix,
    std::unordered_map<std::string, std::string> &out_map,
    std::vector<std::string> &out_keys)
{
    std::vector<std::string> ids;
    this->get_parameter(param_name, ids);
    if (ids.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Parameter '%s' must be a non-empty string array.", param_name.c_str());
        return false;
    }

    out_map.clear();
    out_keys.clear();
    const auto keys = sensorKeysFromIds(ids, prefix);
    size_t key_index = 0;
    for (size_t i = 0; i < ids.size(); ++i) {
        if (ids[i].empty()) {
            RCLCPP_WARN(this->get_logger(), "Ignoring empty ID in '%s' at index %zu.", param_name.c_str(), i);
            continue;
        }
        if (key_index >= keys.size()) {
            continue;
        }
        const std::string &key = keys[key_index++];
        out_map[key] = ids[i];
        out_keys.push_back(key);
    }

    if (out_keys.empty()) {
        RCLCPP_WARN(this->get_logger(), "No valid IDs found in '%s'.", param_name.c_str());
        return false;
    }

    return true;
}

bool AGVOffboardControl::parseAnchorTagKey(
    const std::string &key,
    std::string &anchor_key,
    std::string &tag_key) const
{
    // Expected format: a<digits>t<digits> (e.g., a1t2, a12t3)
    if (key.size() < 4 || key[0] != 'a') {
        return false;
    }

    const size_t t_pos = key.find('t', 1);
    if (t_pos == std::string::npos || t_pos <= 1 || t_pos >= key.size() - 1) {
        return false;
    }

    for (size_t i = 1; i < t_pos; ++i) {
        if (!std::isdigit(static_cast<unsigned char>(key[i]))) {
            return false;
        }
    }
    for (size_t i = t_pos + 1; i < key.size(); ++i) {
        if (!std::isdigit(static_cast<unsigned char>(key[i]))) {
            return false;
        }
    }

    anchor_key = key.substr(0, t_pos);  // "a12"
    tag_key = key.substr(t_pos);        // "t3"
    return true;
}

void AGVOffboardControl::timer_callback()
{

    if (++offboard_counter_ == 10) {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1);
        
        start_time_ = this->get_clock()->now();
    }

    if (!started_ && offboard_counter_ > 10){
        //Wait a bit for the drone to take off
        auto now = this->get_clock()->now();
        if ((now - start_time_).seconds() >= 5.0) {
            started_ = true;
        }
    }


    publish_offboard_control_mode();

    //Start publishing setpoints
    publish_trajectory_setpoint();
}

void AGVOffboardControl::load_trajectory(const std::string &filename)
{
	std::ifstream file(filename);
	if (!file.is_open()) {
		RCLCPP_ERROR(this->get_logger(), "Failed to open trajectory file: %s", filename.c_str());
		return;
	}

	std::string line;
	size_t line_num = 0;

	// Skip the header row
	std::getline(file, line);  // <- Skip line 1

	while (std::getline(file, line)) {
		line_num++;
		std::stringstream ss(line);
		std::string val;
		std::array<double, 4> pose;
		int i = 0;
		while (std::getline(ss, val, ',')) {
			try {
				pose[i++] = std::stod(val);
			} catch (const std::exception &e) {
				RCLCPP_WARN(this->get_logger(), "Skipping invalid line %zu: %s", line_num + 1, line.c_str());
				i = -1;
				break;
			}
		}
		if (i == 4) {
			// Trajectories are authored in the vehicle-local ENU frame.
			// Convert them once into world ENU so they align with the
			// world-frame pose tracked by the controller.
			Eigen::Vector3d local_pos_enu(pose[0], pose[1], pose[2]);
			Eigen::Vector3d world_pos_enu = origin_q_enu_ * local_pos_enu + origin_pos_enu_;

			Eigen::Quaterniond q_local_enu(
				Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitZ()));
			Eigen::Quaterniond q_world_enu = origin_q_enu_ * q_local_enu;
			const double yaw_world_enu = wrapPi(yaw_from_quat_enu(q_world_enu));

			trajectory_.push_back({
				world_pos_enu.x(),
				world_pos_enu.y(),
				world_pos_enu.z(),
				yaw_world_enu});
		}
	}
	file.close();
	RCLCPP_INFO(this->get_logger(), "Loaded %zu trajectory points.", trajectory_.size());
}

void AGVOffboardControl::publish_offboard_control_mode()
{
    OffboardControlMode msg{};
    msg.velocity = true;
    msg.position = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

void AGVOffboardControl::publish_trajectory_setpoint()
{
    std::array<double, 4> pose;
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        pose = current_pose_;
    }

    if (trajectory_.empty()) return;

    if(!started_) return;

    // ---- Closest-point search in a forward window (like UAV) ----
    const size_t window_size = 50;
    const size_t search_start = closest_idx_;
    const size_t search_end   = std::min(closest_idx_ + window_size, trajectory_.size() - 1);

    double min_dist = std::numeric_limits<double>::max();
    size_t new_closest_idx = closest_idx_;

    for (size_t i = search_start; i <= search_end; ++i) {
        const double dx = trajectory_[i][0] - pose[0];
        const double dy = trajectory_[i][1] - pose[1];
        const double dist = std::sqrt(dx*dx + dy*dy);  // planar distance for AGV
        if (dist < min_dist) {
            min_dist = dist;
            new_closest_idx = i;
        }
    }

    // ---- Arc-length lookahead from the closest index ----
    size_t new_target_idx = new_closest_idx;
    double accum = 0.0;
    for (size_t i = new_closest_idx + 1; i < trajectory_.size(); ++i) {
        const double dx = trajectory_[i][0] - trajectory_[i-1][0];
        const double dy = trajectory_[i][1] - trajectory_[i-1][1];
        accum += std::sqrt(dx*dx + dy*dy);
        if (accum >= lookahead_distance_) {
            new_target_idx = i;
            break;
        }
    }

    // ensure target advances at least one sample (prevents zero-dist target)
    if (new_target_idx == new_closest_idx) {
        new_target_idx = std::min(new_closest_idx + 1, trajectory_.size() - 1);
    }

    closest_idx_ = new_closest_idx;
    current_target_idx_ = new_target_idx;

    const auto &target = trajectory_[current_target_idx_];

    // ---- Compute control to the lookahead target ----
    double dx = target[0] - pose[0];
    double dy = target[1] - pose[1];
    double dist = std::sqrt(dx*dx + dy*dy);

    // End-of-path stop (optional): if near the last waypoint, stop.
    // If we're very near the end of the path, trigger LAND (once)
	if ((closest_idx_ >= 0.8*trajectory_.size()) && dist < 0.4) {
			return;
	}

    // ----- SAFE direction: use ex,ey if valid; else use path tangent -----
    double ux = 0.0, uy = 0.0;
    if (dist > 1e-3) {
        ux = dx / dist;
        uy = dy / dist;
    }

    const double yaw_world_enu = std::atan2(dy, dx);

    // // Velocity in WORLD ENU
    const double vx_world_enu = cruise_speed_ * ux;
    const double vy_world_enu = cruise_speed_ * uy;
    const double vz_world_enu = 0.0;

    // --- Convert command to PX4 LOCAL NED ---
    // 1) Velocity: world ENU -> local ENU -> local NED
    Eigen::Vector3d v_world_enu(vx_world_enu, vy_world_enu, vz_world_enu);
    Eigen::Vector3d v_local_enu = origin_q_enu_.inverse() * v_world_enu;
    Eigen::Vector3d v_local_ned = px4_ros_com::frame_transforms::enu_to_ned_local_frame(v_local_enu);

    // 2) Yaw: world ENU -> local ENU -> PX4(NED) heading
    Eigen::Quaterniond q_world_enu(Eigen::AngleAxisd(yaw_world_enu, Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond q_local_enu = origin_q_enu_.inverse() * q_world_enu;
    Eigen::Quaterniond q_ned = px4_ros_com::frame_transforms::ros_to_px4_orientation(q_local_enu);
    const double yaw_ned =
        px4_ros_com::frame_transforms::utils::quaternion::quaternion_get_yaw(q_ned);

    // --- Build TrajectorySetpoint in PX4 local NED (velocity mode) ---
    TrajectorySetpoint msg{};
    const auto nan = std::numeric_limits<float>::quiet_NaN();
    msg.position = {nan, nan, nan};  // velocity mode
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    // Velocity mode
    msg.position = {nan, nan, nan};
    msg.velocity = {
        static_cast<float>(v_local_ned.x()),
        static_cast<float>(v_local_ned.y()),
        static_cast<float>(v_local_ned.z())
    };

    msg.yaw = static_cast<float>(wrapPi(yaw_ned));

    trajectory_setpoint_publisher_->publish(msg);
}

void AGVOffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = vehicle_target_system_;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}


void AGVOffboardControl::publish_distances()
{
    eliko_messages::msg::DistancesList list_msg;
    list_msg.header.stamp = this->get_clock()->now();
    list_msg.header.frame_id = frame_prefix_ + "/eliko";

    {
        std::lock_guard<std::mutex> lock(uwb_mutex_);
        for (const auto& [topic, distance] : uwb_distances_) {
            // Parse topic like "/uwb_gz_simulator/distances/a1t2"
            std::string key = topic.substr(topic.find_last_of('/') + 1); // "a1t2"
            std::string anchor_key, tag_key;
            if (!parseAnchorTagKey(key, anchor_key, tag_key)) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "Ignoring malformed UWB key '%s' from topic '%s'.",
                                     key.c_str(), topic.c_str());
                continue;
            }

            if (anchor_ids_.count(anchor_key) && tag_ids_.count(tag_key)) {
                eliko_messages::msg::Distances d;
                d.anchor_sn = anchor_ids_[anchor_key];
                d.tag_sn = tag_ids_[tag_key];
                d.distance = static_cast<float>(distance);
                list_msg.anchor_distances.push_back(d);
            }
        }

        uwb_distances_.clear();
    }

    if (!list_msg.anchor_distances.empty()) {
        distances_list_pub_->publish(list_msg);
    }

}

int main(int argc, char *argv[])
{
    std::cout << "Starting AGV offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AGVOffboardControl>();
    node->set_parameter(rclcpp::Parameter("use_sim_time", true));

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
