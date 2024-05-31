#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node{
  public:
    MinimalPublisher(): Node("px4_pose_node"){
      //rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
      //auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

      // Publisher for pose estimator
      estimator_publisher = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 10);

      // Create subscription to the position of the UAV (posestamped) 
      vicon_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(
				"/vicon/QuadDani/QuadDani", 10, std::bind(&MinimalPublisher::vicon_callback, this, std::placeholders::_1));
      // Create timer to publish the pose of the UAV
      timer = this->create_wall_timer(10ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
   
    void timer_callback(){
      estimator_publisher->publish(pose);
    }

    void vicon_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
      px4_msgs::msg::VehicleOdometry pose;
      pose.timestamp = std::chrono::system_clock::now().time_since_epoch().count();
      pose.q[0] = msg->pose.orientation.x;
      pose.q[1] = msg->pose.orientation.y;
      pose.q[2] = msg->pose.orientation.z;
      pose.q[3] = msg->pose.orientation.w;
      pose.position[0] = msg->pose.position.x;
      pose.position[1] = -1*msg->pose.position.y;
      pose.position[2] = -1*msg->pose.position.z;
      pose.pose_frame = 2;
      // NaN values for the velocities
      pose.velocity[0] = std::numeric_limits<float>::quiet_NaN();
      pose.velocity[1] = std::numeric_limits<float>::quiet_NaN();
      pose.velocity[2] = std::numeric_limits<float>::quiet_NaN();

      this->pose = pose;
    }

    px4_msgs::msg::VehicleOdometry pose;

    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr estimator_publisher;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vicon_subscriber;

    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
