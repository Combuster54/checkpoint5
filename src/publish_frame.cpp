#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_broadcaster.h"

class ScanToTFNode : public rclcpp::Node {
public:
  ScanToTFNode() : Node("scan_to_tf") {
    // Crear un publicador para enviar el mensaje de TF
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    // Distancia entre las patas
    leg_separation_ = 0.2; // Suponiendo una separación de 20 cm entre las patas
    // leg_separation_ = abs(d2-d1);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
    // Calcular la posición del objeto
    // En este ejemplo, se asume que el objeto está a una distancia de 1 metro y
    // un ángulo de 45 grados con respecto al sensor.

    double object_distance = 1.0;
    // double object_distance = d;

    double object_angle = 45.0 * M_PI / 180.0;
    double object_x = object_distance * std::cos(object_angle);
    double object_y = object_distance * std::sin(object_angle);

    // Calcular la posición de las patas
    double left_leg_x = object_x - leg_separation_ / 2.0;
    double left_leg_y = object_y;
    double right_leg_x = object_x + leg_separation_ / 2.0;
    double right_leg_y = object_y;

    // Calcular la posición del objeto en el centro de las patas
    double object_center_x = (left_leg_x + right_leg_x) / 2.0;
    double object_center_y = (left_leg_y + right_leg_y) / 2.0;

    // Crear el mensaje de Transformación de Frames
    geometry_msgs::msg::TransformStamped transform_stamped_msg;
    transform_stamped_msg.header.stamp = this->now();
    transform_stamped_msg.header.frame_id =
        "base_link"; // El marco de referencia fijo
    transform_stamped_msg.child_frame_id =
        "object"; // El marco de referencia del objeto
    transform_stamped_msg.transform.translation.x = object_center_x;
    transform_stamped_msg.transform.translation.y = object_center_y;
    transform_stamped_msg.transform.translation.z = 0.0;
    transform_stamped_msg.transform.rotation.x = 0.0;
    transform_stamped_msg.transform.rotation.y = 0.0;
    transform_stamped_msg.transform.rotation.z = 0.0;
    transform_stamped_msg.transform.rotation.w = 1.0;

    // Crear el mensaje de Transformación de Frames para la pata izquierda
    geometry_msgs::msg::TransformStamped transform_stamped_left_leg_msg;
    transform_stamped_left_leg_msg.header.stamp = this->now();
    transform_stamped_left_leg_msg.header.frame_id = ""
  }
};