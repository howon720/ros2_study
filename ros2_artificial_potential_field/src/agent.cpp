#include "agent.h"

using namespace std::chrono_literals;
namespace apf {
ApfAgent::ApfAgent() : rclcpp::Node("agent") {
  // Agent id
  this->declare_parameter("agent_id", 0);
  agent_id = this->get_parameter("agent_id").as_int();

  // Mission file name
  this->declare_parameter("mission_file_name", "~/ros2_ws/src/ros2_artificial_potential_field/mission/mission_single_agent.yaml");
  std::string mission_file_name = this->get_parameter("mission_file_name").as_string();

  // Mission
  YAML::Node mission = YAML::LoadFile(mission_file_name);
  auto agents_yaml = mission["agents"];
  number_of_agents = agents_yaml.size();
  agent_positions.resize(number_of_agents);
  for(size_t id = 0; id < number_of_agents; id++) {
    agent_positions[id] = Vector3d(agents_yaml[id]["start"][0].as<double>(),
                                   agents_yaml[id]["start"][1].as<double>(),
                                   agents_yaml[id]["start"][2].as<double>());
  }
  start = agent_positions[agent_id];
  goal = Vector3d(agents_yaml[agent_id]["goal"][0].as<double>(),
                  agents_yaml[agent_id]["goal"][1].as<double>(),
                  agents_yaml[agent_id]["goal"][2].as<double>());

  auto obstacles_yaml = mission["obstacles"];
  number_of_obstacles = obstacles_yaml.size();
  obstacles.resize(number_of_obstacles);
  for(size_t obs_id = 0; obs_id < number_of_obstacles; obs_id++) {
    obstacles[obs_id].position = Vector3d(obstacles_yaml[obs_id]["position"][0].as<double>(),
                                          obstacles_yaml[obs_id]["position"][1].as<double>(),
                                          obstacles_yaml[obs_id]["position"][2].as<double>());
    obstacles[obs_id].radius = obstacles_yaml[obs_id]["radius"].as<double>();
  }

  // Initialize agent's state
  state.position = start;
  state.velocity = Vector3d(0, 0, 0);

  // TF2_ROS
  //TODO: initialize tf buffer, listener, broadcaster
  tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  // ROS timer
  int timer_period_ms = static_cast<int>(dt * 1000);
  timer_tf = this->create_wall_timer(std::chrono::milliseconds(timer_period_ms),
                                  std::bind(&ApfAgent::timer_tf_callback, this));
  timer_pub = this->create_wall_timer(40ms,
                                  std::bind(&ApfAgent::timer_pub_callback, this));

  // ROS publisher
  pub_pose = this->create_publisher<visualization_msgs::msg::MarkerArray>("robot/pose", 10);

  std::cout << "[ApfAgent] Agent" << agent_id << " is ready." << std::endl;
}

void ApfAgent::timer_tf_callback() {
  listen_tf();
  update_state();
  broadcast_tf();
}

void ApfAgent::timer_pub_callback() {
  publish_marker_pose();
}

void ApfAgent::listen_tf() {
  //TODO: listen tf and update agent_positions 
  for (size_t id = 0; id < number_of_agents; id++) {
    try {
      auto tf = tf_buffer->lookupTransform(frame_id, "agent_" + std::to_string(id), tf2::TimePointZero);
      agent_positions[id] = Vector3d(
        tf.transform.translation.x,
        tf.transform.translation.y,
        tf.transform.translation.z
      );
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed for agent_%zu: %s", id, ex.what());
    }
  }
  
  // Collision check
  double min_dist = 1000000;
  for(size_t id = 0; id < number_of_agents; id++) {
    double dist = (agent_positions[id] - state.position).norm();
    if(id != agent_id and dist < min_dist) {
      min_dist = dist;
    }
  }
  if(min_dist < 2 * radius){
    std::cout<< "Collision! Minimum distance between agents: " + std::to_string(min_dist) << std::endl;
  }

  for(size_t obs_id = 0; obs_id < number_of_obstacles; obs_id++) {
    double dist = (obstacles[obs_id].position - state.position).norm();
    if(dist < radius + obstacles[obs_id].radius) {
      std::cout<< "Collision! Minimum distance between agent and obstacle: " + std::to_string(dist) << std::endl;
    }
  }
}

void ApfAgent::update_state() {
  //TODO: Update the agent's state using the double integrator model
  Vector3d u = apf_controller();
  state.velocity += u * dt;
  state.position += state.velocity * dt;
}

void ApfAgent::broadcast_tf() {
  //TODO: Broadcast the agent's current position
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = this->get_clock()->now();
  tf_msg.header.frame_id = frame_id;
  tf_msg.child_frame_id = "agent_" + std::to_string(agent_id);
  tf_msg.transform.translation.x = state.position.x();
  tf_msg.transform.translation.y = state.position.y();
  tf_msg.transform.translation.z = state.position.z();
  tf_msg.transform.rotation.w = 1.0;
  tf_msg.transform.rotation.x = 0.0;
  tf_msg.transform.rotation.y = 0.0;
  tf_msg.transform.rotation.z = 0.0;
  tf_broadcaster->sendTransform(tf_msg);
}

Vector3d ApfAgent::apf_controller() {
  Vector3d u;

  // Attraction force
  Vector3d u_goal = 0.5 * (goal - state.position);

  // Repulsion force
  Vector3d u_obs = Vector3d::Zero();
  for (size_t i = 0; i < number_of_agents; i++) {
    if (i == agent_id) continue;
    Vector3d diff = state.position - agent_positions[i];
    double dist = diff.norm();
    if (dist < 1.5 && dist > 0.001) {
      u_obs += 2.0 * (1.0 / dist - 1.0 / 1.5) * (1.0 / (dist * dist)) * diff.normalized();
    }
  }

  for (const auto& obs : obstacles) {
    Vector3d diff = state.position - obs.position;
    double dist = diff.norm();
    double safe_dist = radius + obs.radius + 1.2;
    if (dist < safe_dist && dist > 0.001) {
      u_obs += 3.0 * (1.0 / dist - 1.0 / safe_dist) * (1.0 / (dist * dist)) * diff.normalized();
    }
  }

  // Damping force
  Vector3d u_damp = -1.7 * state.velocity;

  // Net force
  u = u_goal + u_obs + u_damp;

  // Clamping for maximum acceleration constraint
  for(int i = 0; i < 3; i++) {
    if(u(i) > max_acc) {u(i) = max_acc;} 
    else if(u(i) < -max_acc) {u(i) = -max_acc;}
  }
  return u;
}


void ApfAgent::publish_marker_pose() {
  // Only agent_id
  if(agent_id != 0) {
    return;
  }

  visualization_msgs::msg::MarkerArray msg;
  for(size_t id = 0; id < number_of_agents; id++) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "agent";
    marker.id = (int)id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = agent_positions[id].x();
    marker.pose.position.y = agent_positions[id].y();
    marker.pose.position.z = agent_positions[id].z();
    marker.pose.orientation.w = 1;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.scale.x = 2 * radius;
    marker.scale.y = 2 * radius;
    marker.scale.z = 2 * radius;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1;
    marker.color.a = 0.3;
    msg.markers.emplace_back(marker);
  }

  for(size_t obs_id = 0; obs_id < number_of_obstacles; obs_id++) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "obstacle";
    marker.id = (int)obs_id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = obstacles[obs_id].position.x();
    marker.pose.position.y = obstacles[obs_id].position.y();
    marker.pose.position.z = obstacles[obs_id].position.z();
    marker.pose.orientation.w = 1;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.scale.x = 2 * obstacles[obs_id].radius;
    marker.scale.y = 2 * obstacles[obs_id].radius;
    marker.scale.z = 2 * obstacles[obs_id].radius;
    marker.color.a = 1;
    msg.markers.emplace_back(marker);
  }

  pub_pose->publish(msg);
}
} // namespace apf