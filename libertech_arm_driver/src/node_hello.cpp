#include "libertech_arm_driver/ArmDriver.hpp"
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class ArmSimpleSubscriber : public rclcpp::Node
{
public:
  ArmSimpleSubscriber()
  : Node("arm_subscriber")
  {
    armDriver_ = new ArmDriver;
    armDriver_->runSendReceiveThread();


    subscription_ = create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&ArmSimpleSubscriber::jointStateCallback, this, std::placeholders::_1));
  }

  ~ArmSimpleSubscriber() {
    armDriver_->pause();
  }

private:
    // 回调函数，处理接收到的关节角消息
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "Received JointState message");
        Vec14 q;
        // 打印关节角信息
        for (size_t i = 0; i < msg->name.size(); ++i) {
            RCLCPP_INFO(get_logger(), "Joint %s: Position %f, Velocity %f, Effort %f",
                        msg->name[i].c_str(), msg->position[i], msg->velocity[i], msg->effort[i]);
            q[i] = msg->position[i];

			armDriver_->setQ(q);
            usleep(1000);
        }

    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    ArmDriver* armDriver_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmSimpleSubscriber>());
  rclcpp::shutdown();
  return 0;
}