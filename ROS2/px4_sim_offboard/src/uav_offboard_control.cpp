/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <px4_ros_com/frame_transforms.h>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <algorithm>
#include <cctype>

#include <rclcpp/qos.hpp>

#include <fstream>
#include <sstream>
#include <map>
#include <vector>
#include <random>
#include <stdexcept>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;


static inline double wrapPi(double a) {
    while (a >  M_PI) a -= 2.0*M_PI;
    while (a < -M_PI) a += 2.0*M_PI;
    return a;
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

class UAVOffboardControl : public rclcpp::Node
{
public:
	UAVOffboardControl()
	: Node(
		"uav_offboard_control",
		rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
	{
		// Retrieve parameter values
		std::string offboard_control_mode_topic_;
		std::string trajectory_setpoint_topic_;
		std::string vehicle_command_topic_;
		std::string vehicle_odometry_topic_;
		std::string vehicle_local_position_topic_;
		std::string ros_odometry_topic_, ros_gt_topic_;
		std::string traj_file_;

		this->get_parameter("offboard_control_mode_topic", offboard_control_mode_topic_);
		this->get_parameter("trajectory_setpoint_topic", trajectory_setpoint_topic_);
		this->get_parameter("vehicle_command_topic", vehicle_command_topic_);
		this->get_parameter("vehicle_odometry_topic", vehicle_odometry_topic_);
		this->get_parameter("vehicle_local_position_topic", vehicle_local_position_topic_);
		// Older generated configs have no vehicle_status_topic: derive it from the command topic.
		std::string vehicle_status_topic_ = vehicle_command_topic_;
		const std::string cmd_suffix = "/in/vehicle_command";
		if (vehicle_status_topic_.size() >= cmd_suffix.size() &&
		    vehicle_status_topic_.compare(vehicle_status_topic_.size() - cmd_suffix.size(), cmd_suffix.size(), cmd_suffix) == 0) {
			vehicle_status_topic_.replace(vehicle_status_topic_.size() - cmd_suffix.size(), cmd_suffix.size(), "/out/vehicle_status_v1");
		}
		this->get_parameter("vehicle_status_topic", vehicle_status_topic_);
		this->get_parameter("trajectory_csv_file", traj_file_);
		this->get_parameter("lookahead_distance", lookahead_distance_);
		this->get_parameter("ros_odometry_topic", ros_odometry_topic_);
		this->get_parameter("ros_gt_topic", ros_gt_topic_);

		this->get_parameter("odom_error_position", odom_error_position_);
		this->get_parameter("odom_error_angle", odom_error_angle_);
		int vehicle_target_system = 2;
		this->get_parameter("vehicle_target_system", vehicle_target_system);
		vehicle_target_system_ = static_cast<uint8_t>(vehicle_target_system);
		this->get_parameter("frame_prefix", frame_prefix_);
		this->get_parameter("marker_array_topic", marker_array_topic_);
		this->get_parameter("world_frame_id", world_frame_id_);

		this->get_parameter("cruise_speed", cruise_speed_);

		const auto listed = this->list_parameters({"tags"}, 10);
		tag_keys_ = directSensorKeysFromPrefixes(listed.prefixes, "tags", "t");
		if (tag_keys_.empty()) {
			throw std::runtime_error("Missing tag configuration under 'tags'.");
		}
		for (const auto &tag_key : tag_keys_) {
			const std::string tag_numeric = tag_key.size() > 1 ? tag_key.substr(1) : tag_key;
			const std::string compact_param = "tags." + tag_numeric + ".position";
			if (!this->has_parameter(compact_param)) {
				throw std::runtime_error("Missing tag position parameter: " + compact_param);
			}
		}

		pos_scale_err_ = odom_error_position_ / 100.0;
		yaw_scale_err_ = odom_error_angle_   / 100.0;

		std::vector<double> uav_origin_vec;
		this->get_parameter("uav_origin", uav_origin_vec);
		if (uav_origin_vec.size() != 6) {
			RCLCPP_WARN(this->get_logger(),
						"uav_origin must have 6 elements [x,y,z,roll,pitch,yaw]; using zeros");
		} else {
			origin_pos_enu_ = Eigen::Vector3d(uav_origin_vec[0], uav_origin_vec[1], uav_origin_vec[2]);

			// RPY are given in ENU. Build quaternion qz(yaw)*qy(pitch)*qx(roll).
			const double roll  = uav_origin_vec[3];
			const double pitch = uav_origin_vec[4];
			const double yaw   = uav_origin_vec[5];

			origin_q_enu_ =
				Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
				Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
				Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX());
		}
		
		std::string package_path = ament_index_cpp::get_package_share_directory("px4_sim_offboard");
		std::string full_csv_path = package_path + "/trajectories/" + traj_file_;
		load_trajectory(full_csv_path);
		if (trajectory_.empty()) {
			RCLCPP_FATAL(this->get_logger(), "No trajectory points were loaded from %s", full_csv_path.c_str());
			throw std::runtime_error("Missing UAV trajectory data.");
		}
		initial_hover_target_ = trajectory_.front(); 

		// Create publishers with the topics from params
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(offboard_control_mode_topic_, 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(trajectory_setpoint_topic_, 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>(vehicle_command_topic_, 10);

		ros_odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(ros_odometry_topic_, 10);
		ros_gt_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(ros_gt_topic_, 10);

		marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_array_topic_, 10);

		start_time_ = this->get_clock()->now();

		tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    	static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

		// Set QoS to match PX4 publisher
		rclcpp::QoS qos_profile = rclcpp::SensorDataQoS();

		for (const auto& tag : tag_keys_) {
			std::vector<double> tag_pos;
			const std::string tag_numeric = tag.size() > 1 ? tag.substr(1) : tag;
			const std::string compact_param = "tags." + tag_numeric + ".position";
			const bool loaded = this->get_parameter(compact_param, tag_pos) && tag_pos.size() == 3;
			if (loaded) {
				geometry_msgs::msg::TransformStamped tf;
				tf.header.stamp = this->get_clock()->now();
				tf.header.frame_id = frame_prefix_ + "/base_link";
				tf.child_frame_id = frame_prefix_ + "/" + tag;
				tf.transform.translation.x = tag_pos[0];
				tf.transform.translation.y = tag_pos[1];
				tf.transform.translation.z = tag_pos[2];
				tf.transform.rotation.x = 0.0;
				tf.transform.rotation.y = 0.0;
				tf.transform.rotation.z = 0.0;
				tf.transform.rotation.w = 1.0;
				static_tf_broadcaster_->sendTransform(tf);
			}
		}

		vehicle_odometry_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
			vehicle_odometry_topic_, qos_profile,
			[this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {

				if (msg->pose_frame != VehicleOdometry::POSE_FRAME_NED) {
					RCLCPP_WARN_THROTTLE(
						this->get_logger(),
						*this->get_clock(),
						2000,
						"Unsupported UAV pose_frame %u. Expected POSE_FRAME_NED.",
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
					const double roll_true = rpy_true[2];

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

					Eigen::Matrix3d R_last_true_enu = q_last_true_enu_.toRotationMatrix();
					Eigen::Vector3d dpos_true_body = R_last_true_enu.transpose() * dpos_true_enu;

					// 3) Add body-frame noise / scale
					Eigen::Vector3d sigma_body(
						pos_scale_err_ * std::abs(dpos_true_body.x()),
						pos_scale_err_ * std::abs(dpos_true_body.y()),
						pos_scale_err_ * std::abs(dpos_true_body.z()));
					Eigen::Vector3d dpos_noisy_body = dpos_true_body + Eigen::Vector3d(
						sigma_body.x() * odom_noise_(rng_),
						sigma_body.y() * odom_noise_(rng_),
						sigma_body.z() * odom_noise_(rng_));

					// Yaw increment
					double dyaw_true = wrapPi(yaw_true - last_yaw_true_);
					double dyaw_noisy = dyaw_true + (yaw_scale_err_ * std::abs(dyaw_true)) * odom_noise_(rng_);

					// 4) Update the *estimated* orientation first
					yaw_noisy_ = wrapPi(yaw_noisy_ + dyaw_noisy);

					Eigen::Matrix3d R_est_enu =
						(Eigen::AngleAxisd(yaw_noisy_, Eigen::Vector3d::UnitZ()) *
						Eigen::AngleAxisd(pitch_true, Eigen::Vector3d::UnitY()) *
						Eigen::AngleAxisd(roll_true,  Eigen::Vector3d::UnitX())).toRotationMatrix();

					pos_noisy_enu_ += R_est_enu * dpos_noisy_body;
					
					// Save true for next step
					last_pos_true_enu_ = pos_enu;
					q_last_true_enu_   = q_enu;
					last_yaw_true_ = yaw_true;
				}

				Eigen::Quaterniond q_noisy_enu =
	            Eigen::AngleAxisd(yaw_noisy_, Eigen::Vector3d::UnitZ()) *
	            Eigen::AngleAxisd(pitch_true, Eigen::Vector3d::UnitY()) *
	            Eigen::AngleAxisd(roll_true,  Eigen::Vector3d::UnitX());

			// --- Fill and publish Odometry (NOISY pose) ---
			nav_msgs::msg::Odometry odom_msg;
			odom_msg.header.stamp = this->get_clock()->now();
				odom_msg.header.frame_id = frame_prefix_ + "/odom";
				odom_msg.child_frame_id = frame_prefix_ + "/base_link";

			// NOISY position
			odom_msg.pose.pose.position.x = pos_noisy_enu_.x();
			odom_msg.pose.pose.position.y = pos_noisy_enu_.y();
			odom_msg.pose.pose.position.z = pos_noisy_enu_.z();

			// NOISY orientation (yaw only perturbed)
			odom_msg.pose.pose.orientation.x = q_noisy_enu.x();
			odom_msg.pose.pose.orientation.y = q_noisy_enu.y();
			odom_msg.pose.pose.orientation.z = q_noisy_enu.z();
			odom_msg.pose.pose.orientation.w = q_noisy_enu.w();

			Eigen::Vector3d vel_child_flu = Eigen::Vector3d::Zero();
			if (msg->velocity_frame == VehicleOdometry::VELOCITY_FRAME_NED) {
				Eigen::Vector3d vel_ned(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
				Eigen::Vector3d vel_enu = px4_ros_com::frame_transforms::ned_to_enu_local_frame(vel_ned);
				vel_child_flu = q_noisy_enu.inverse() * vel_enu;
			} else if (msg->velocity_frame == VehicleOdometry::VELOCITY_FRAME_BODY_FRD) {
				vel_child_flu = Eigen::Vector3d(msg->velocity[0], -msg->velocity[1], -msg->velocity[2]);
			} else {
				RCLCPP_WARN_THROTTLE(
					this->get_logger(),
					*this->get_clock(),
					2000,
					"Unsupported UAV velocity_frame %u. Publishing zero linear twist.",
					msg->velocity_frame);
			}

			odom_msg.twist.twist.linear.x = vel_child_flu.x();
			odom_msg.twist.twist.linear.y = vel_child_flu.y();
			odom_msg.twist.twist.linear.z = vel_child_flu.z();

			// PX4 angular velocity is BODY_FRD, convert to ROS child-frame FLU.
			odom_msg.twist.twist.angular.x = msg->angular_velocity[0];
			odom_msg.twist.twist.angular.y = -msg->angular_velocity[1];
			odom_msg.twist.twist.angular.z = -msg->angular_velocity[2];


			// Covariances: you can keep PX4 variances or inflate slightly since we add noise.
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

		vehicle_status_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
			vehicle_status_topic_, qos_profile,
			[this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
				have_status_ = true;
				if (!engaged_ &&
				    msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED &&
				    msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
					engaged_ = true;
					RCLCPP_INFO(this->get_logger(), "Armed in offboard mode.");
				}
			});

		vehicle_local_position_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
			vehicle_local_position_topic_, qos_profile,
			[this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {

				// Initialize/reset counter tracking
				if (last_heading_reset_counter_ < 0) {
					last_heading_reset_counter_ = msg->heading_reset_counter;
				}

				// If EKF has applied a reset, accumulate delta_heading (NED frame)
				if (msg->heading_reset_counter != last_heading_reset_counter_) {
					heading_reset_accum_ += msg->delta_heading;  // NED
					last_heading_reset_counter_ = msg->heading_reset_counter;
				}

				if (last_xy_reset_counter_ < 0) last_xy_reset_counter_ = msg->xy_reset_counter;
				if (last_z_reset_counter_  < 0) last_z_reset_counter_  = msg->z_reset_counter;

				if (msg->xy_reset_counter != last_xy_reset_counter_) {
					xy_reset_accum_.x() += msg->delta_xy[0];  // meters, NED x
					xy_reset_accum_.y() += msg->delta_xy[1];  // meters, NED y
					last_xy_reset_counter_ = msg->xy_reset_counter;
				}
				if (msg->z_reset_counter != last_z_reset_counter_) {
					z_reset_accum_ += msg->delta_z;           // meters, NED z
					last_z_reset_counter_ = msg->z_reset_counter;
				}

					// Keep ground-truth pose continuous by undoing estimator frame resets.
					Eigen::Vector3d pos_ned(
						msg->x - xy_reset_accum_.x(),
						msg->y - xy_reset_accum_.y(),
						msg->z - z_reset_accum_
					);

				// --- Convert to local ENU, then compose with world origin ---
				Eigen::Vector3d pos_enu_local  = px4_ros_com::frame_transforms::ned_to_enu_local_frame(pos_ned);
				Eigen::Vector3d pos_enu_world  = origin_q_enu_ * pos_enu_local + origin_pos_enu_;

					// Compensate heading resets in NED before converting to ENU.
					const double heading_ned = wrapPi(msg->heading - heading_reset_accum_);
					Eigen::Quaterniond q_heading_ned(Eigen::AngleAxisd(heading_ned, Eigen::Vector3d::UnitZ()));
				Eigen::Quaterniond q_heading_enu =
					px4_ros_com::frame_transforms::px4_to_ros_orientation(q_heading_ned);

				// compose with world origin orientation
				Eigen::Quaterniond q_world_enu = origin_q_enu_ * q_heading_enu;

				// Extract world ENU yaw (for your controller state)
				double yaw_world_enu = wrapPi(yaw_from_quat_enu(q_world_enu));

				// ---- Update current pose in WORLD ENU ----
				{
					std::lock_guard<std::mutex> lock(position_mutex_);
					current_pose_[0] = pos_enu_world.x();
					current_pose_[1] = pos_enu_world.y();
					current_pose_[2] = pos_enu_world.z();
					current_pose_[3] = yaw_world_enu;
				}


				geometry_msgs::msg::PoseStamped pose_msg;
				// Header
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

				// Store pose history
				gt_pose_history_.push_back(pose_msg.pose);
				if (gt_pose_history_.size() > 500.0) {
                gt_pose_history_.erase(gt_pose_history_.begin());
            	}
				// Publish marker array
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
					marker.color.r = 0.0;
					marker.color.g = 0.0;
					marker.color.b = 1.0;
					marker.color.a = 1.0;
					marker.lifetime = rclcpp::Duration(0,0); // forever
					marker_array.markers.push_back(marker);
				}
				marker_array_pub_->publish(marker_array);

			});


		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {

			// Change to Offboard mode and arm after 10 setpoints. Commands sent
			// before the DDS link to PX4 has matched are dropped, so while
			// vehicle_status doesn't confirm armed + offboard, resend every second
			// (never after LAND). Without vehicle_status, send once as before.
			const auto now = this->get_clock()->now();
			const bool retry = have_status_ && !engaged_ && !land_sent_ &&
			                   offboard_setpoint_counter_ >= 10 && (now - start_time_).seconds() >= 1.0;
			if (offboard_setpoint_counter_ == 10 || retry) {
				if (retry) {
					RCLCPP_WARN(this->get_logger(), "Arm/offboard not confirmed by PX4 yet, resending.");
				}
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
				start_time_ = now;

			}

			if (!started_ && offboard_setpoint_counter_ >= 10 && (engaged_ || !have_status_)){
				//Wait a bit for the drone to take off
				if ((now - start_time_).seconds() >= 2.0) {
					started_ = true;
				}
			}

			// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();
			publish_trajectory_setpoint();

			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
		
	}

	void arm();
	void disarm();
	void land();

private:

	static inline double yaw_from_quat_enu(const Eigen::Quaterniond &q)
	{
		Eigen::Vector3d eul = q.toRotationMatrix().eulerAngles(2, 1, 0);
		return eul[0];
	}

	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ros_odometry_publisher_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ros_gt_publisher_;
	std::vector<geometry_msgs::msg::Pose> gt_pose_history_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
	rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_subscriber_;
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_subscriber_;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscriber_;
	bool have_status_ = false;  // a vehicle_status message has been received
	bool engaged_ = false;      // PX4 has reported armed + offboard (latched)

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	std::array<double, 4> current_pose_{0.0, 0.0, 0.0, 0.0};	//[x,y,z,yaw]
	std::mutex position_mutex_;  // to make access thread-safe

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void load_trajectory(const std::string &filename);

	std::vector<std::array<double, 4>> trajectory_;
	size_t current_target_idx_ = 0;
	size_t closest_idx_ = 0;
	double lookahead_distance_ = 2.0;  // meters
	double cruise_speed_ = 0.5;         // proportional gain
	double odom_error_position_ = 0.0; // %,  meters for every 100 meters traveled
	double odom_error_angle_ = 0.0; // % degrees for every 100

	bool hover_reached_ = false;
	bool land_sent_ = false;  // flag to ensure LAND command is sent only once
	std::array<double, 4> initial_hover_target_{0.0, 0.0, -2.0, 0.0};
	double hover_tolerance_ = 0.3;  // meters

	Eigen::Vector3d origin_pos_enu_{0.0, 0.0, 0.0};
	Eigen::Quaterniond origin_q_enu_{1.0, 0.0, 0.0, 0.0}; // w,x,y,z

		// --- Odometry noise state ---
		bool have_prev_odom_{false};
		Eigen::Vector3d last_pos_true_enu_{0.0, 0.0, 0.0};
		Eigen::Quaterniond q_last_true_enu_{1,0,0,0}; 
		int last_odom_reset_counter_{-1};

	double last_yaw_true_{0.0};     // ENU yaw

	Eigen::Vector3d pos_noisy_enu_{0.0, 0.0, 0.0}; // integrated noisy position

	double yaw_noisy_{0.0};                        // integrated noisy yaw (ENU)

	// scale biases (as percentages). 1.0 == 1% scale error.
	double pos_scale_err_{0.0};                    // = odom_error_position_ / 100.0
	double yaw_scale_err_{0.0};                    // = odom_error_angle_   / 100.0

	// RNG for Gaussian noise
	std::mt19937 rng_{std::random_device{}()};
	std::normal_distribution<double> odom_noise_{0.0, 1.0};

	int    last_heading_reset_counter_{-1};
	double heading_reset_accum_{0.0};   // sum of delta_heading since start

	int last_xy_reset_counter_{-1};
	int last_z_reset_counter_{-1};
	Eigen::Vector2d xy_reset_accum_{0.0, 0.0};
	double z_reset_accum_{0.0};

    rclcpp::Time start_time_;
	bool started_ = false;
	uint8_t vehicle_target_system_ = 2;
	std::string frame_prefix_{"uav"};
	std::string world_frame_id_{"map"};
	std::string marker_array_topic_{"/uav/gt_marker_array"};
	std::vector<std::string> tag_keys_;

	std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

};

void UAVOffboardControl::load_trajectory(const std::string &filename)
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
			// Convert them once into world ENU so they can be compared
			// directly against the world-frame ground-truth pose.
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


/**
 * @brief Send a command to Arm the vehicle
 */
void UAVOffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void UAVOffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void UAVOffboardControl::land()
{
    // PX4: NAV_LAND starts an auto land at current position for MC
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
	land_sent_ = true;  // Set flag to prevent multiple LAND commands
    RCLCPP_INFO(this->get_logger(), "LAND command sent");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void UAVOffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	if (!started_ || !hover_reached_) {
		msg.position = true;
		msg.velocity = false;
	} else {
		msg.position = false;
		msg.velocity = true;
	}
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 */
void UAVOffboardControl::publish_trajectory_setpoint()
{
	std::array<double, 4> pose;
	{
		std::lock_guard<std::mutex> lock(position_mutex_);
		pose = current_pose_;
	}

	if (!started_ || !hover_reached_) {
		// Check distance to hover point
		double dx = pose[0] - initial_hover_target_[0];
		double dy = pose[1] - initial_hover_target_[1];
		double dz = pose[2] - initial_hover_target_[2];
		double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

		// Convert hover target (world ENU) to local NED for PX4
		Eigen::Vector3d p_world_enu(initial_hover_target_[0], initial_hover_target_[1], initial_hover_target_[2]);
		Eigen::Vector3d p_local_enu = origin_q_enu_.inverse() * (p_world_enu - origin_pos_enu_);
		Eigen::Vector3d p_local_ned = px4_ros_com::frame_transforms::enu_to_ned_local_frame(p_local_enu);

		// Yaw: build world ENU quaternion, move to local ENU, then to PX4(NED)
		Eigen::Quaterniond q_hover_world_enu(
			Eigen::AngleAxisd(initial_hover_target_[3], Eigen::Vector3d::UnitZ()));
		Eigen::Quaterniond q_hover_local_enu = origin_q_enu_.inverse() * q_hover_world_enu;
		Eigen::Quaterniond q_hover_ned = px4_ros_com::frame_transforms::ros_to_px4_orientation(q_hover_local_enu);
		double yaw_ned = px4_ros_com::frame_transforms::utils::quaternion::quaternion_get_yaw(q_hover_ned);

		TrajectorySetpoint msg{};
		const auto nan = std::numeric_limits<float>::quiet_NaN();
		msg.position = {static_cast<float>(p_local_ned.x()),
						static_cast<float>(p_local_ned.y()),
						static_cast<float>(p_local_ned.z())};
		msg.velocity = {nan, nan, nan};
		msg.acceleration = {nan, nan, nan};
		msg.jerk = {nan, nan, nan};
		msg.yaw = static_cast<float>(wrapPi(yaw_ned));
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		trajectory_setpoint_publisher_->publish(msg);

		if (dist < hover_tolerance_) {
			if (!hover_reached_) {
				RCLCPP_INFO(this->get_logger(), "Initial hover position reached.");
			}
			hover_reached_ = true;
		}

		if (!started_ || !hover_reached_) {
			return;
		}
	}

	// ---- Begin normal trajectory tracking ----

	if (trajectory_.empty()) {
		RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Trajectory is empty");
		return;
	}

	if(land_sent_){
		RCLCPP_INFO(this->get_logger(), "Landing already sent, not sending further setpoints.");
		return;
	}

	// Step 1: Find the closest point in a forward-looking window
	size_t window_size = 50;
	size_t search_start = closest_idx_;
	size_t search_end = std::min(closest_idx_ + window_size, trajectory_.size() - 1);

	double min_dist = std::numeric_limits<double>::max();
	size_t new_closest_idx = closest_idx_;  // start from last known

	for (size_t i = search_start; i < search_end; ++i) {
		double dx = trajectory_[i][0] - pose[0];
		double dy = trajectory_[i][1] - pose[1];
		double dz = trajectory_[i][2] - pose[2];
		double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
		if (dist < min_dist) {
			min_dist = dist;
			new_closest_idx = i;
		}
	}

	size_t start = new_closest_idx;
	double accum_dist = 0.0;
	size_t new_target_idx = start;

	for (size_t i = start + 1; i < trajectory_.size(); ++i) {
		double dx = trajectory_[i][0] - trajectory_[i-1][0];
		double dy = trajectory_[i][1] - trajectory_[i-1][1];
		double dz = trajectory_[i][2] - trajectory_[i-1][2];
		accum_dist += std::sqrt(dx*dx + dy*dy + dz*dz);
		if (accum_dist >= lookahead_distance_) {
			new_target_idx = i;
			break;
		}
	}


	// Update indices
	closest_idx_ = new_closest_idx;
	current_target_idx_ = new_target_idx;

	const auto &target = trajectory_[current_target_idx_];

	// Compute position error
	double ex = target[0] - pose[0];
	double ey = target[1] - pose[1];
	double ez = target[2] - pose[2];
	double dist = std::sqrt(ex*ex + ey*ey + ez*ez);

	// If we're very near the end of the path, trigger LAND (once)
	if ((closest_idx_ >= 0.8*trajectory_.size()) && dist < 0.4) {
		land();
		return;
	}

	double vx = 0.0, vy = 0.0, vz = 0.0;
	if (dist > 1e-3) {
		const double vmag = std::min(cruise_speed_, dist * 0.8); // taper near goal
		double ux = ex / dist, uy = ey / dist, uz = ez / dist;
		vx = vmag * ux; vy = vmag * uy; vz = vmag * uz;
	}

	// v_world_enu from your controller (vx, vy, vz in world ENU)
	Eigen::Vector3d v_world_enu(vx, vy, vz);
	// Transform velocity to local ENU then to local NED
	Eigen::Vector3d v_local_enu = origin_q_enu_.inverse() * v_world_enu;
	Eigen::Vector3d v_local_ned = px4_ros_com::frame_transforms::enu_to_ned_local_frame(v_local_enu);

	// Yaw to face the target in world ENU
	double yaw_world_enu = std::atan2(ey, ex);

	// Convert that yaw to PX4 NED heading
	Eigen::Quaterniond q_world_enu(Eigen::AngleAxisd(yaw_world_enu, Eigen::Vector3d::UnitZ()));
	Eigen::Quaterniond q_local_enu = origin_q_enu_.inverse() * q_world_enu;
	Eigen::Quaterniond q_ned = px4_ros_com::frame_transforms::ros_to_px4_orientation(q_local_enu);
	double yaw_ned = px4_ros_com::frame_transforms::utils::quaternion::quaternion_get_yaw(q_ned);

	// Build setpoint
	TrajectorySetpoint msg{};
	auto nan = std::numeric_limits<float>::quiet_NaN();
    
	// msg.position = {static_cast<float>(p_pred_local_ned.x()),
    //                 static_cast<float>(p_pred_local_ned.y()),
    //                 static_cast<float>(p_pred_local_ned.z())};
	msg.position = {nan, nan, nan}; // PX4 ignores position if velocity is set
	msg.velocity = {static_cast<float>(v_local_ned.x()),
					static_cast<float>(v_local_ned.y()),
					static_cast<float>(v_local_ned.z())};
	msg.yaw = static_cast<float>(wrapPi(yaw_ned));
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void UAVOffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
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

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	auto node = std::make_shared<UAVOffboardControl>();
	node->set_parameter(rclcpp::Parameter("use_sim_time", true));
	rclcpp::spin(node);

	rclcpp::shutdown();
	return 0;
}
