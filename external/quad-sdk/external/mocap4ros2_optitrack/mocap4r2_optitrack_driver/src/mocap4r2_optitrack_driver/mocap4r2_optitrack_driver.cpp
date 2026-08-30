// Copyright 2021 Institute for Robotics and Intelligent Machines,
//                Georgia Institute of Technology
// Copyright 2024 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Christian Llanes <christian.llanes@gatech.edu>
// Author: David Vargas Frutos <david.vargas@urjc.es>
// Author: Francisco Martín <fmrico@urjc.es>

#include <string>
#include <vector>
#include <memory>

#include "mocap4r2_msgs/msg/marker.hpp"
#include "mocap4r2_msgs/msg/markers.hpp"

#include "mocap4r2_optitrack_driver/mocap4r2_optitrack_driver.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace mocap4r2_optitrack_driver
{

using std::placeholders::_1;
using std::placeholders::_2;

OptitrackDriverNode::OptitrackDriverNode()
: ControlledLifecycleNode("mocap4r2_optitrack_driver_node")
{
  declare_parameter<std::string>("connection_type", "Unicast");
  declare_parameter<std::string>("server_address", "000.000.000.000");
  declare_parameter<std::string>("local_address", "000.000.000.000");
  declare_parameter<std::string>("multicast_address", "000.000.000.000");
  declare_parameter<uint16_t>("server_command_port", 0);
  declare_parameter<uint16_t>("server_data_port", 0);
  declare_parameter<std::vector<int64_t>>("rigid_body_ids", std::vector<int64_t>{});

  client = new NatNetClient();
  client->SetFrameReceivedCallback(process_frame_callback, this);
}

OptitrackDriverNode::~OptitrackDriverNode()
{
}

void OptitrackDriverNode::set_settings_optitrack()
{
  if (connection_type_ == "Multicast") {
    client_params.connectionType = ConnectionType::ConnectionType_Multicast;
    client_params.multicastAddress = multicast_address_.c_str();
  } else if (connection_type_ == "Unicast") {
    client_params.connectionType = ConnectionType::ConnectionType_Unicast;
  } else {
    RCLCPP_FATAL(get_logger(), "Unknown connection type -- options are Multicast, Unicast");
    rclcpp::shutdown();
  }

  client_params.serverAddress = server_address_.c_str();
  client_params.localAddress = local_address_.c_str();
  client_params.serverCommandPort = server_command_port_;
  client_params.serverDataPort = server_data_port_;
}

bool OptitrackDriverNode::stop_optitrack()
{
  RCLCPP_INFO(get_logger(), "Disconnecting from optitrack DataStream SDK");

  return true;
}

void
OptitrackDriverNode::control_start(const mocap4r2_control_msgs::msg::Control::SharedPtr msg)
{
  (void)msg;
}

void
OptitrackDriverNode::control_stop(const mocap4r2_control_msgs::msg::Control::SharedPtr msg)
{
  (void)msg;
}

void NATNET_CALLCONV process_frame_callback(sFrameOfMocapData * data, void * pUserData)
{
  static_cast<OptitrackDriverNode *>(pUserData)->process_frame(data);
}

std::chrono::nanoseconds OptitrackDriverNode::get_optitrack_system_latency(sFrameOfMocapData * data)
{
  const bool bSystemLatencyAvailable = data->CameraMidExposureTimestamp != 0;

  if (bSystemLatencyAvailable) {
    const double clientLatencySec =
      client->SecondsSinceHostTimestamp(data->CameraMidExposureTimestamp);
    const double clientLatencyMillisec = clientLatencySec * 1000.0;
    const double transitLatencyMillisec =
      client->SecondsSinceHostTimestamp(data->TransmitTimestamp) * 1000.0;

    const double largeLatencyThreshold = 100.0;
    if (clientLatencyMillisec >= largeLatencyThreshold) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *this->get_clock(), 500,
        "Optitrack system latency >%.0f ms: [Transmission: %.0fms, Total: %.0fms]",
        largeLatencyThreshold, transitLatencyMillisec, clientLatencyMillisec);
    }

    return round<std::chrono::nanoseconds>(std::chrono::duration<float>{clientLatencySec});
  } else {
    RCLCPP_WARN_ONCE(get_logger(), "Optitrack's system latency not available");
    return std::chrono::nanoseconds::zero();
  }
}

void
OptitrackDriverNode::process_frame(sFrameOfMocapData * data)
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  frame_number_++;
  // rclcpp::Duration frame_delay = rclcpp::Duration(get_optitrack_system_latency(data));
  rclcpp::Duration frame_delay = rclcpp::Duration(0, 0);  // Use now() only, matching ROS1 behavior



  std::map<int, std::vector<mocap4r2_msgs::msg::Marker>> marker2rb;

  // Markers
  if (mocap4r2_markers_pub_->get_subscription_count() > 0) {
    mocap4r2_msgs::msg::Markers msg;
    msg.header.stamp = now() - frame_delay;
    msg.header.frame_id = "map";
    msg.frame_number = frame_number_;

    for (int i = 0; i < data->nLabeledMarkers; i++) {
      bool Unlabeled = ((data->LabeledMarkers[i].params & 0x10) != 0);
      bool ActiveMarker = ((data->LabeledMarkers[i].params & 0x20) != 0);
      sMarker & marker_data = data->LabeledMarkers[i];
      int modelID, markerID;
      NatNet_DecodeID(marker_data.ID, &modelID, &markerID);

      mocap4r2_msgs::msg::Marker marker;
      marker.id_type = mocap4r2_msgs::msg::Marker::USE_INDEX;
      marker.marker_index = i;
      marker.translation.x = marker_data.x;
      marker.translation.y = marker_data.y;
      marker.translation.z = marker_data.z;
      if (ActiveMarker || Unlabeled) {
        msg.markers.push_back(marker);
      } else {
        marker2rb[modelID].push_back(marker);
      }
    }
    mocap4r2_markers_pub_->publish(msg);
  }

  if (mocap4r2_rigid_body_pub_->get_subscription_count() > 0) {
    mocap4r2_msgs::msg::RigidBodies msg_rb;
    msg_rb.header.stamp = now() - frame_delay;
    msg_rb.header.frame_id = "map";
    msg_rb.frame_number = frame_number_;

    for (int i = 0; i < data->nRigidBodies; i++) {
      mocap4r2_msgs::msg::RigidBody rb;

      rb.rigid_body_name = std::to_string(data->RigidBodies[i].ID);
      rb.pose.position.x = data->RigidBodies[i].x;
      rb.pose.position.y = data->RigidBodies[i].y;
      rb.pose.position.z = data->RigidBodies[i].z;
      rb.pose.orientation.x = data->RigidBodies[i].qx;
      rb.pose.orientation.y = data->RigidBodies[i].qy;
      rb.pose.orientation.z = data->RigidBodies[i].qz;
      rb.pose.orientation.w = data->RigidBodies[i].qw;
      rb.markers = marker2rb[data->RigidBodies[i].ID];

      msg_rb.rigidbodies.push_back(rb);
    }

    mocap4r2_rigid_body_pub_->publish(msg_rb);
  }

  // Per-body publishing: PoseStamped, Pose2D, and TF with OptiTrack Y-Z coordinate swap
  rclcpp::Time stamp = now() - frame_delay;
  for (int i = 0; i < data->nRigidBodies; i++) {
    int body_id = data->RigidBodies[i].ID;
    auto cfg_it = rigid_body_configs_.find(body_id);
    if (cfg_it == rigid_body_configs_.end()) {
      continue;
    }
    const RigidBodyConfig & cfg = cfg_it->second;

    // Apply OptiTrack -> ROS coordinate transform.
    // OptiTrack frame: +x=left, +y=up, +z=forward
    // ROS frame:       +x=forward, +y=left, +z=up
    double px = data->RigidBodies[i].z;
    double py = data->RigidBodies[i].x;
    double pz = data->RigidBodies[i].y;
    double ox = data->RigidBodies[i].qz;
    double oy = data->RigidBodies[i].qx;
    double oz = data->RigidBodies[i].qy;
    double ow = data->RigidBodies[i].qw;

    if (cfg.publish_pose && pose_pubs_.count(body_id)) {
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.stamp = stamp;
      pose_msg.header.frame_id = cfg.parent_frame_id;
      pose_msg.pose.position.x = px;
      pose_msg.pose.position.y = py;
      pose_msg.pose.position.z = pz;
      pose_msg.pose.orientation.x = ox;
      pose_msg.pose.orientation.y = oy;
      pose_msg.pose.orientation.z = oz;
      pose_msg.pose.orientation.w = ow;
      pose_pubs_[body_id]->publish(pose_msg);
    }

    if (cfg.publish_pose2d && pose2d_pubs_.count(body_id)) {
      // Yaw from quaternion: atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
      double yaw = std::atan2(2.0 * (ow * oz + ox * oy), 1.0 - 2.0 * (oy * oy + oz * oz));
      geometry_msgs::msg::Pose2D pose2d_msg;
      pose2d_msg.x = px;
      pose2d_msg.y = py;
      pose2d_msg.theta = yaw;
      pose2d_pubs_[body_id]->publish(pose2d_msg);
    }

    if (cfg.publish_tf && tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = stamp;
      tf_msg.header.frame_id = cfg.parent_frame_id;
      tf_msg.child_frame_id = cfg.child_frame_id;
      tf_msg.transform.translation.x = px;
      tf_msg.transform.translation.y = py;
      tf_msg.transform.translation.z = pz;
      tf_msg.transform.rotation.x = ox;
      tf_msg.transform.rotation.y = oy;
      tf_msg.transform.rotation.z = oz;
      tf_msg.transform.rotation.w = ow;
      tf_broadcaster_->sendTransform(tf_msg);
    }
  }
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


// The next Callbacks are used to manage behavior in the different states of the lifecycle node.
CallbackReturnT
OptitrackDriverNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  initParameters();

  mocap4r2_markers_pub_ = create_publisher<mocap4r2_msgs::msg::Markers>(
    "markers", rclcpp::QoS(1000));
  mocap4r2_rigid_body_pub_ = create_publisher<mocap4r2_msgs::msg::RigidBodies>(
    "rigid_bodies", rclcpp::QoS(1000));

  loadRigidBodyConfig();
  for (auto const & [id, config] : rigid_body_configs_) {
    if (config.publish_pose) {
      pose_pubs_[id] = create_publisher<geometry_msgs::msg::PoseStamped>(
        config.pose_topic, rclcpp::QoS(1000));
    }
    if (config.publish_pose2d) {
      pose2d_pubs_[id] = create_publisher<geometry_msgs::msg::Pose2D>(
        config.pose2d_topic, rclcpp::QoS(1000));
    }
  }
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  connect_optitrack();

  RCLCPP_INFO(get_logger(), "Configured!\n");

  return ControlledLifecycleNode::on_configure(state);
}

CallbackReturnT
OptitrackDriverNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  mocap4r2_markers_pub_->on_activate();
  mocap4r2_rigid_body_pub_->on_activate();
  for (auto & [id, pub] : pose_pubs_) {pub->on_activate();}
  for (auto & [id, pub] : pose2d_pubs_) {pub->on_activate();}
  RCLCPP_INFO(get_logger(), "Activated!\n");

  return ControlledLifecycleNode::on_activate(state);
}

CallbackReturnT
OptitrackDriverNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  mocap4r2_markers_pub_->on_deactivate();
  mocap4r2_rigid_body_pub_->on_deactivate();
  for (auto & [id, pub] : pose_pubs_) {pub->on_deactivate();}
  for (auto & [id, pub] : pose2d_pubs_) {pub->on_deactivate();}
  RCLCPP_INFO(get_logger(), "Deactivated!\n");

  return ControlledLifecycleNode::on_deactivate(state);
}

CallbackReturnT
OptitrackDriverNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Cleaned up!\n");

  if (disconnect_optitrack()) {
    return ControlledLifecycleNode::on_cleanup(state);
  } else {
    return CallbackReturnT::FAILURE;
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
OptitrackDriverNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Shutted down!\n");

  if (disconnect_optitrack()) {
    return ControlledLifecycleNode::on_shutdown(state);
  } else {
    return CallbackReturnT::FAILURE;
  }
}

CallbackReturnT
OptitrackDriverNode::on_error(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());

  disconnect_optitrack();

  return ControlledLifecycleNode::on_error(state);
}

bool
OptitrackDriverNode::connect_optitrack()
{
  RCLCPP_INFO(
    get_logger(),
    "Trying to connect to Optitrack NatNET SDK at %s ...", server_address_.c_str());

  client->Disconnect();
  set_settings_optitrack();

  if (client->Connect(client_params) == ErrorCode::ErrorCode_OK) {
    RCLCPP_INFO(get_logger(), "... connected!");

    memset(&server_description, 0, sizeof(server_description));
    client->GetServerDescription(&server_description);
    if (!server_description.HostPresent) {
      RCLCPP_DEBUG(get_logger(), "Unable to connect to server. Host not present.");
      return false;
    }

    if (client->GetDataDescriptionList(&data_descriptions) != ErrorCode_OK || !data_descriptions) {
      RCLCPP_DEBUG(get_logger(), "[Client] Unable to retrieve Data Descriptions.\n");
    }

    RCLCPP_INFO(get_logger(), "\n[Client] Server application info:\n");
    RCLCPP_INFO(
      get_logger(), "Application: %s (ver. %d.%d.%d.%d)\n",
      server_description.szHostApp, server_description.HostAppVersion[0],
      server_description.HostAppVersion[1], server_description.HostAppVersion[2],
      server_description.HostAppVersion[3]);
    RCLCPP_INFO(
      get_logger(), "NatNet Version: %d.%d.%d.%d\n", server_description.NatNetVersion[0],
      server_description.NatNetVersion[1],
      server_description.NatNetVersion[2], server_description.NatNetVersion[3]);
    RCLCPP_INFO(get_logger(), "Client IP:%s\n", client_params.localAddress);
    RCLCPP_INFO(get_logger(), "Server IP:%s\n", client_params.serverAddress);
    RCLCPP_INFO(get_logger(), "Server Name:%s\n", server_description.szHostComputerName);

    void * pResult;
    int nBytes = 0;

    if (client->SendMessageAndWait("FrameRate", &pResult, &nBytes) == ErrorCode_OK) {
      float fRate = *(static_cast<float *>(pResult));
      RCLCPP_INFO(get_logger(), "Mocap Framerate : %3.2f\n", fRate);
    } else {
      RCLCPP_DEBUG(get_logger(), "Error getting frame rate.\n");
    }

  } else {
    RCLCPP_INFO(get_logger(), "... not connected :( ");
    return false;
  }

  return true;
}

bool
OptitrackDriverNode::disconnect_optitrack()
{
  void * response;
  int nBytes;
  if (client->SendMessageAndWait("Disconnect", &response, &nBytes) == ErrorCode_OK) {
    client->Disconnect();
    RCLCPP_INFO(get_logger(), "[Client] Disconnected");
    return true;
  } else {
    RCLCPP_ERROR(get_logger(), "[Client] Disconnect not successful..");
    return false;
  }
}

void
OptitrackDriverNode::initParameters()
{
  get_parameter<std::string>("connection_type", connection_type_);
  get_parameter<std::string>("server_address", server_address_);
  get_parameter<std::string>("local_address", local_address_);
  get_parameter<std::string>("multicast_address", multicast_address_);
  get_parameter<uint16_t>("server_command_port", server_command_port_);
  get_parameter<uint16_t>("server_data_port", server_data_port_);
  get_parameter<std::vector<int64_t>>("rigid_body_ids", rigid_body_ids_);
}

void
OptitrackDriverNode::loadRigidBodyConfig()
{
  rigid_body_configs_.clear();
  if (rigid_body_ids_.empty()) {
    RCLCPP_WARN(get_logger(), "rigid_body_ids not set — per-body PoseStamped/Pose2D/TF publishing disabled");
    return;
  }

  for (int64_t id : rigid_body_ids_) {
    std::string prefix = "body_" + std::to_string(id) + "_";
    RigidBodyConfig cfg;

    std::string pose_topic, pose2d_topic, child_frame, parent_frame;

    // Declare and read per-body parameters
    declare_parameter<std::string>(prefix + "pose_topic", "");
    declare_parameter<std::string>(prefix + "pose2d_topic", "");
    declare_parameter<std::string>(prefix + "child_frame_id", "");
    declare_parameter<std::string>(prefix + "parent_frame_id", "");

    get_parameter(prefix + "pose_topic", pose_topic);
    get_parameter(prefix + "pose2d_topic", pose2d_topic);
    get_parameter(prefix + "child_frame_id", child_frame);
    get_parameter(prefix + "parent_frame_id", parent_frame);

    if (!pose_topic.empty()) {
      cfg.pose_topic = pose_topic;
      cfg.publish_pose = true;
    }
    if (!pose2d_topic.empty()) {
      cfg.pose2d_topic = pose2d_topic;
      cfg.publish_pose2d = true;
    }
    if (!child_frame.empty() && !parent_frame.empty()) {
      cfg.child_frame_id = child_frame;
      cfg.parent_frame_id = parent_frame;
      cfg.publish_tf = true;
    }

    rigid_body_configs_[static_cast<int>(id)] = cfg;
    RCLCPP_INFO(
      get_logger(),
      "Rigid body %ld: pose=%s, pose2d=%s, tf=%s->%s",
      id,
      cfg.publish_pose ? cfg.pose_topic.c_str() : "disabled",
      cfg.publish_pose2d ? cfg.pose2d_topic.c_str() : "disabled",
      cfg.publish_tf ? cfg.parent_frame_id.c_str() : "disabled",
      cfg.publish_tf ? cfg.child_frame_id.c_str() : "disabled");
  }
}

}  // namespace mocap4r2_optitrack_driver
