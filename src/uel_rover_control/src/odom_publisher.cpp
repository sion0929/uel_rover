#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <chrono>
#include <cmath>

// ────────────────────────────────────────────────────────────────
//  오도메트리 노드
//
//  OpenRB-150(micro-ROS)이 발행하는 좌/우 바퀴 RPM을 받아
//  속도 적분으로 위치 추정
//
//  subscribe: /wheel_velocity  (Float32MultiArray: [vL_rpm, vR_rpm])
//  publish:   /odom            (nav_msgs/Odometry)
//             /tf              (odom → base_link)
// ────────────────────────────────────────────────────────────────

class OdomPublisher : public rclcpp::Node {
public:
    OdomPublisher() : Node("odom_publisher") {
        wheel_base_   = declare_parameter("wheel_base",   0.30);
        wheel_radius_ = declare_parameter("wheel_radius", 0.065);
        // 펌웨어의 MAX_RPM은 50. 그 2배를 넘는 값은 물리적으로 나올 수 없으므로
        // 측정이 아니라 오염으로 간주한다. 여유를 크게 둬서 정상 주행은 절대
        // 걸리지 않게 한다.
        max_wheel_rpm_ = declare_parameter("max_wheel_rpm", 100.0);

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        tf_br_    = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // OpenRB-150으로부터 바퀴 RPM 수신
        vel_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "/wheel_velocity", 10,
            std::bind(&OdomPublisher::velCallback, this, std::placeholders::_1));

        last_time_ = now();
        RCLCPP_INFO(get_logger(), "odom_publisher started");
    }

private:
    void velCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) return;

        auto cur_time = now();
        double dt = (cur_time - last_time_).seconds();
        last_time_ = cur_time;

        if (dt <= 0.0 || dt > 1.0) return;  // 비정상 dt 무시

        // 비물리적 RPM 방어. 2026-07-22 실기에서 odom이 (-71 km, -257 km)와
        // (-6.7 km, -9.2 km)까지 발산한 사례가 있었다. 그 좌표에서 slam_toolbox가
        // 0.05 m 해상도 OccupancyGrid를 잡으려 하면 약 99 GB가 필요해 커널 OOM
        // Killer에 SIGKILL(exit -9) 당한다. 발생원은 아직 특정하지 못했으므로
        // (재현 실패) 근본 수정이 아니라 적분 단계의 가드다.
        const float rpm_l = msg->data[0];
        const float rpm_r = msg->data[1];
        if (!std::isfinite(rpm_l) || !std::isfinite(rpm_r) ||
            std::fabs(rpm_l) > max_wheel_rpm_ || std::fabs(rpm_r) > max_wheel_rpm_) {
            // 조용히 버리면 발생원 추적이 불가능해지므로 반드시 남긴다.
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "비물리적 바퀴 속도 무시: [%.3f, %.3f] rpm (허용 ±%.1f)",
                rpm_l, rpm_r, max_wheel_rpm_);
            return;
        }

        // RPM → m/s 변환
        double v_left  = rpmToMs(rpm_l);
        double v_right = rpmToMs(rpm_r);

        // 차동구동 기구학
        double v     = (v_left + v_right) / 2.0;
        double omega = (v_right - v_left) / wheel_base_;

        double dtheta = omega * dt;
        double dx     = v * std::cos(theta_ + dtheta / 2.0) * dt;
        double dy     = v * std::sin(theta_ + dtheta / 2.0) * dt;

        x_     += dx;
        y_     += dy;
        theta_ += dtheta;

        publishOdom(cur_time, v, omega);
    }

    double rpmToMs(double rpm) {
        return rpm * 2.0 * M_PI * wheel_radius_ / 60.0;
    }

    void publishOdom(const rclcpp::Time & stamp, double v, double omega) {
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp    = stamp;
        tf.header.frame_id = "odom";
        tf.child_frame_id  = "base_link";
        tf.transform.translation.x = x_;
        tf.transform.translation.y = y_;
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();
        tf_br_->sendTransform(tf);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id  = "base_link";
        odom.pose.pose.position.x    = x_;
        odom.pose.pose.position.y    = y_;
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        odom.twist.twist.linear.x  = v;
        odom.twist.twist.angular.z = omega;
        odom_pub_->publish(odom);
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br_;

    double wheel_base_, wheel_radius_, max_wheel_rpm_;
    double x_ = 0, y_ = 0, theta_ = 0;
    rclcpp::Time last_time_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPublisher>());
    rclcpp::shutdown();
}
