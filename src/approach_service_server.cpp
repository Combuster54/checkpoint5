#include "attach_shelf/srv/detail/go_to_loading__struct.hpp"
#include "attach_shelf/srv/go_to_loading.hpp"

#include "rclcpp/executors.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "rmw/types.h"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "attach_shelf/srv/go_to_loading.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include <geometry_msgs/msg/transform_stamped.h>
#include <unistd.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <functional>
#include <future>
#include <inttypes.h>
#include <iostream>
#include <memory>
#include <sstream>
#include <vector>

using GoToLoadingMsg = attach_shelf::srv::GoToLoading;

using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

class FramePublisher : public rclcpp::Node {
public:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  bool initialized_ = false;
  float dist;
  float x_frame;
  float y_frame;
  bool on_ = false;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  int count = 0;
  std::vector<float> x_list;
  std::vector<float> y_list;
  float prom_x = 0;
  float prom_y = 0;
  geometry_msgs::msg::Point position;

  FramePublisher()
      : Node("FramePublisher_Node"), tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {

    count = 0;
    x_list.clear();
    y_list.clear();
    static_transform_.header.frame_id = "robot_odom";
    static_transform_.child_frame_id = "cart_frame";

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&FramePublisher::odom_callback, this, _1));

    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&FramePublisher::publish_frame, this));
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    if (on_  && count<= 200) {

      RCLCPP_INFO_ONCE(this->get_logger(), "Go odometry!");

      x_list.push_back(msg->pose.pose.position.x);
      y_list.push_back(msg->pose.pose.position.y);
      RCLCPP_INFO(this->get_logger(), "x_list[%i] :  %f", count, x_list[count]);
      RCLCPP_INFO(this->get_logger(), "y_list[%i] :  %f", count, y_list[count]);
      count += 1;
    }
  }

  void mean_positions() {

    sleep(5);
    if (!x_list.empty()) {
      float sum_x = std::accumulate(x_list.begin(), x_list.end(), 0.0);
      prom_x = sum_x / x_list.size();
      RCLCPP_INFO_ONCE(this->get_logger(), " [mean_pos] sum_x : %f", sum_x);
      RCLCPP_INFO_ONCE(this->get_logger(), " [mean_pos] prom_x : %f", prom_x);

    } else {

      RCLCPP_INFO_ONCE(this->get_logger(), " x_list is EMPTY");
    }
    if (!y_list.empty()) {
      float sum_y = std::accumulate(y_list.begin(), y_list.end(), 0.0);
      prom_y = sum_y / y_list.size();
      RCLCPP_INFO_ONCE(this->get_logger(), " [mean_pos] sum_x : %f", sum_y);
      RCLCPP_INFO_ONCE(this->get_logger(), " [mean_pos] prom_x : %f", prom_y);

    } else {

      RCLCPP_INFO_ONCE(this->get_logger(), " y_list is EMPTY");
    }
  }

  void publish_frame() {

    if (on_) {

      mean_positions();

      if (!initialized_) {

        // Obtener la transformación entre "robot_base_link" y "robot_odom"
        auto transform = tf_buffer_.lookupTransform(
            "robot_odom", "robot_front_laser_link", tf2::TimePoint(),
            tf2::Duration(static_cast<long int>(0.1 * 1e9)));

        // Crear la transformación estática entre "cart_frame" y
        // "robot_base_link"
        // RCLCPP_INFO_ONCE(this->get_logger(), "X : %f",
        //                  transform.transform.translation.x);
        // RCLCPP_INFO_ONCE(this->get_logger(), "Y : %f",
        //                  transform.transform.translation.y);

        // RCLCPP_INFO_ONCE(this->get_logger(), "frame_x : %f", x_frame);
        // RCLCPP_INFO_ONCE(this->get_logger(), "frame_y : %f", y_frame);

        float pos_x = prom_x - y_frame;
        float pos_y = prom_y - 0.210 - x_frame;

        // RCLCPP_INFO_ONCE(this->get_logger(), "POS X  : %f", pos_x);
        // RCLCPP_INFO_ONCE(this->get_logger(), "POS Y  : %f", pos_y);

        // position.x = robot_base_link.x + cart_frame.x
        position.x = pos_x;
        // position.x = robot_base_link.y -
        // dif(robot_base_link-/-robot_front_laser_link) -
        // cart_frame.y
        position.y = pos_y;
        // X : -2.748567
        // Y : 4.947450
        static_transform_.header.stamp = this->now();
        // static_transform_.transform.translation.x =
        //     transform.transform.translation.x;
        // static_transform_.transform.translation.y =
        //     transform.transform.translation.y;
        static_transform_.transform.translation.x = position.x;
        static_transform_.transform.translation.y = position.y;
        static_transform_.transform.translation.z = 0.0;
        static_transform_.transform.rotation.x = 0.0;
        static_transform_.transform.rotation.y = 0.0;
        static_transform_.transform.rotation.z = 0.0;
        static_transform_.transform.rotation.w = 1.0;

        initialized_ = true;
      }

      // Publish the static transform
      broadcaster_->sendTransform(static_transform_);
    }
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::TransformStamped static_transform_;
};

class FrameFollower : public rclcpp::Node {

public:
  bool timer_on = false;
  int count = 0;
  FrameFollower()
      : Node("FrameFollower_Node"), tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {
    cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("robot/cmd_vel", 10);
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(200),
                                std::bind(&FrameFollower::sendCommand, this));
    // positioning_timer_ =
    //     this->create_wall_timer(std::chrono::milliseconds(200),
    //                             std::bind(&FrameFollower::position, this));
  }

  //   void position(){

  //   }

  void sendCommand() {
    // try {
    RCLCPP_INFO_ONCE(this->get_logger(), "[FRAMEFOLLOWER] sendCommand");

    if (timer_on) {
      sleep(7);
      RCLCPP_INFO_ONCE(this->get_logger(), "HELLO");

      // Obtener la transformación entre "rick/base_link" y "morty/base_link"
      auto transform = tf_buffer_.lookupTransform(
          "robot_base_link", "cart_frame", tf2::TimePoint(),
          tf2::Duration(static_cast<long int>(0.1 * 1e9)));

      // Calcular la distancia y el error angular entre los frames de referencia
      double error_distance = sqrt(pow(transform.transform.translation.x, 2) +
                                   pow(transform.transform.translation.y, 2));
      double error_yaw = atan2(transform.transform.translation.y,
                               transform.transform.translation.x);

      //   RCLCPP_INFO(this->get_logger(), "error_distance : %f",
      //   error_distance); RCLCPP_INFO(this->get_logger(), "error_yaw : %f",
      //   error_yaw);

      // Definir la velocidad angular y lineal
      double kp_yaw =
          0.5; // Valor fijo para el controlador de la velocidad angular
      double kp_distance =
          0.2; // Valor fijo para el controlador de la velocidad lineal
      double angular_vel = kp_yaw * error_yaw;
      double linear_vel = kp_distance * error_distance;

      //   RCLCPP_INFO(this->get_logger(), "kp_yaw : %f", kp_yaw);
      //   RCLCPP_INFO(this->get_logger(), "kp_distance : %f", kp_distance);
      RCLCPP_INFO(this->get_logger(), "angular_vel : %f", angular_vel);
      RCLCPP_INFO(this->get_logger(), "linear_vel : %f", linear_vel);

      // Menor a esto es que Rick ya alcanzo a Morty y ya colisionaron, asi que
      // no hay razon para seguir avanzando
      if (error_distance <= 0.1) {

        // count +=1;
        this->cmd_vel.angular.z = 0.0;
        this->cmd_vel.linear.x = 0.0;
        cmd_vel_publisher_->publish(this->cmd_vel);

        // this->cmd_vel.angular.z = 0.1;
        // cmd_vel_publisher_->publish(this->cmd_vel);
        // sleep(3);
        this->cmd_vel.angular.z = 0.0;
        this->cmd_vel.linear.x = 0.0;
        cmd_vel_publisher_->publish(this->cmd_vel);
        RCLCPP_INFO(this->get_logger(), "cart_frame reached");
        RCLCPP_INFO(this->get_logger(), "Positioning to load object...");
        this->cmd_vel.linear.x = 0.1;
        cmd_vel_publisher_->publish(this->cmd_vel);
        sleep(4);
        this->cmd_vel.angular.z = 0.0;
        this->cmd_vel.linear.x = 0.0;
        cmd_vel_publisher_->publish(this->cmd_vel);
        RCLCPP_INFO(this->get_logger(), "Ready to load!");
        timer_on = false;
        timer_->cancel();
      } else {

        // Alcanza cart_frame
        RCLCPP_INFO(this->get_logger(), "following cart_frame");

        this->cmd_vel.angular.z = angular_vel;
        this->cmd_vel.linear.x = linear_vel;
        cmd_vel_publisher_->publish(this->cmd_vel);
      }
      // }catch (tf2::TransformException &ex) {
      //   RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      // }
    }
  }

  geometry_msgs::msg::Twist cmd_vel;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2::TimePoint tf_time_zero = tf2::TimePoint(std::chrono::seconds(0));
  double kp_distance_;
  double kp_yaw_;
};

// START HERE
class AppService : public rclcpp::Node {
public:
  std::shared_ptr<FramePublisher> framePublisher;
  std::shared_ptr<FrameFollower> frameFollower;

  AppService() : Node("App_Node") {

    // Initialize FrameFollower and FramePublisher
    framePublisher = std::make_shared<FramePublisher>();
    frameFollower = std::make_shared<FrameFollower>();

    // Callback Group
    callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // LaserScan G1
    laser_callback_G1 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // LaserScan G1
    laser_callback_G2 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options1;
    options1.callback_group = laser_callback_G1;

    rclcpp::SubscriptionOptions options2;
    options2.callback_group = laser_callback_G2;

    // Subscribers
    // sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
    //     "odom", 10,
    //     std::bind(&AppService::odom_callback, this, std::placeholders::_1),
    //     options2);

    sub_laser = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&AppService::scan_callback, this, std::placeholders::_1),
        options1);

    // Create Server
    srv_ = this->create_service<GoToLoadingMsg>(
        "approach_shelf",
        std::bind(&AppService::service_callback, this, std::placeholders::_1,
                  std::placeholders::_2),
        ::rmw_qos_profile_default, callback_group_);

    RCLCPP_INFO(this->get_logger(), "GoToLoadingMsg Server is READY!");
  }

private:
  std::vector<float> intensities_above_threshold;
  bool start = true;
  int changes = 0;
  std::vector<int> index_intensities;
  std::vector<float> ranges;
  float angle_min;
  float angle_increment;
  bool read_intensity = false;
  rclcpp::Service<GoToLoadingMsg>::SharedPtr srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::CallbackGroup::SharedPtr laser_callback_G1;
  rclcpp::CallbackGroup::SharedPtr laser_callback_G2;

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    if (read_intensity) {

      RCLCPP_INFO_ONCE(this->get_logger(), "Reading...");

      //
      intensities_above_threshold = msg->intensities;
      ranges = msg->ranges;
      angle_min = msg->angle_min;
      angle_increment = msg->angle_increment;
    }
  }
  void
  service_callback(const std::shared_ptr<GoToLoadingMsg::Request> request,
                   const std::shared_ptr<GoToLoadingMsg::Response> response) {

    RCLCPP_INFO(this->get_logger(), "Server has been called!");

    // RCLCPP_INFO(this->get_logger(), "Read True");
    read_intensity = true;
    sleep(2);
    read_intensity = false;
    // RCLCPP_INFO(this->get_logger(), "Read False");
    framePublisher->on_ = true;

    for (size_t i = 0; i < intensities_above_threshold.size(); ++i) {

      if (intensities_above_threshold[i] !=
          intensities_above_threshold[i - 1]) {

        changes += 1;
        if (changes % 2 == 0) // par
        {
          index_intensities.push_back(i - 1);
        } else {
          index_intensities.push_back(i);
        }
      }
    }
    int positive = 0;

    for (size_t i = 0; i < intensities_above_threshold.size(); ++i) {

      if (intensities_above_threshold[i - 1] - intensities_above_threshold[i] >
          0) {

        positive += 1;

        if (changes % 2 == 0) // par
        {
          index_intensities.push_back(i - 1);
        } else {
          index_intensities.push_back(i);
        }
      }
    }

    if (changes <= 3) {

      RCLCPP_INFO(this->get_logger(), "One or none Leg!");
      response->complete = false;
    }
    if (changes == 4) {
      RCLCPP_INFO(this->get_logger(), "2 LEGS!");

      int index_d1 = (index_intensities[0] + index_intensities[1]) / 2;
      int index_d2 = ((index_intensities[2] + index_intensities[3]) / 2);

      //   int index_d1 = index_intensities[1];
      //   int index_d2 = index_intensities[4];

      float d1 = ranges[index_d1];
      float d2 = ranges[index_d2];

      // right 230-240
      // 235
      // subscribe robot and obtain these values
      //
      //   RCLCPP_INFO(this->get_logger(), "D1 : %f", d1);
      //   RCLCPP_INFO(this->get_logger(), "D2 : %f", d2);

      float distance = (d1 + d2) / 2;
      //   RCLCPP_INFO(this->get_logger(), "DISTANCE : %f", distance);

      //   float angle_1 = abs(index_d1 - 540) / 4;
      //   float angle_2 = abs(540 - index_d2) / 4;
      //   float x1 = x0 - D * cos(T_rad);
      //   float y1 = y0 - D * sin(T_rad);

      float angle1 = angle_min + index_d1 * angle_increment;
      float angle2 = angle_min + index_d2 * angle_increment;
      float theta = (angle1 + angle2) / 2;

      float x = distance * cos(theta);
      float y = distance * sin(theta);
      //   RCLCPP_INFO(this->get_logger(), "x_d : %f", x);
      //   RCLCPP_INFO(this->get_logger(), "y_d : %f", y);
      framePublisher->x_frame = x;
      framePublisher->y_frame = y;
      framePublisher->dist = distance;
      sleep(5);
      //   RCLCPP_INFO(this->get_logger(), "timer_on = %d",
      //               frameFollower->timer_on ? 1 : 0);
      RCLCPP_INFO(this->get_logger(), "attach_to_shelf = %d",
                  request->attach_to_shelf ? 1 : 0);

      if (request->attach_to_shelf) {
        frameFollower->timer_on = true;
        while (frameFollower->timer_on) {

          RCLCPP_INFO_ONCE(this->get_logger(),
                           "[Service - Server] Following frame");
        }

        //  Avanzar 30cm mas para terminar en el centro
      }
      RCLCPP_INFO(this->get_logger(), "response->complete = true");
      response->complete = true;

    } else {
      RCLCPP_INFO(this->get_logger(), "FFFFFFFF");

      RCLCPP_INFO(this->get_logger(), "Response = False ");

      response->complete = false;
    }
  }
};

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  auto app_server = std::make_shared<AppService>();
  auto staticPublisher = app_server->framePublisher;
  auto frameFollower = app_server->frameFollower;

  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(app_server);
  executor.add_node(staticPublisher);
  executor.add_node(frameFollower);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}