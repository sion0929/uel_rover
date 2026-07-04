#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

// ────────────────────────────────────────────────────────────────
//  모드 관리 노드
//
//  OpenRB-150이 RC 스위치(ch5)를 읽어 /drive_mode를 발행
//  AUTO 모드일 때만 Nav2의 /cmd_vel을 /cmd_vel_out으로 전달
//
//  subscribe: /drive_mode  (String: "RC" | "AUTO" | "STOP")  ← OpenRB-150
//             /cmd_vel     (Twist)                            ← Nav2 (미래)
//  publish:   /cmd_vel_out (Twist)                            → OpenRB-150
// ────────────────────────────────────────────────────────────────

class ModeManager : public rclcpp::Node {
public:
    ModeManager() : Node("mode_manager") {
        mode_sub_ = create_subscription<std_msgs::msg::String>(
            "/drive_mode", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                if (msg->data != current_mode_) {
                    current_mode_ = msg->data;
                    RCLCPP_INFO(get_logger(), "드라이브 모드: %s", current_mode_.c_str());
                    if (current_mode_ != "AUTO") {
                        // RC/STOP 모드 전환 시 정지 명령 발행
                        cmd_pub_->publish(geometry_msgs::msg::Twist());
                    }
                }
            });

        nav_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                // AUTO 모드일 때만 Nav2 명령을 OpenRB-150으로 전달
                if (current_mode_ == "AUTO") {
                    cmd_pub_->publish(*msg);
                }
            });

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_out", 10);

        RCLCPP_INFO(get_logger(), "mode_manager started");
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    std::string current_mode_ = "STOP";
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModeManager>());
    rclcpp::shutdown();
}
