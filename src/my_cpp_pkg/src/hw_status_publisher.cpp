#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/hardware_status.hpp"


class HwStatusPublisher: public rclcpp::Node
{
    public:

        HwStatusPublisher(): Node("hw_status_publisher")
        {
            publisher_ = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>("hw_status",10);
            timer_ = this->create_wall_timer(std::chrono::seconds(1),std::bind(&HwStatusPublisher::callback_publish_hw_status,this));
            RCLCPP_INFO(this->get_logger(),"Hardware Status publisher node has been started.");
        }

    private:

        void callback_publish_hw_status()
        {
            auto msg = my_robot_interfaces::msg::HardwareStatus();
            msg.temperature  = 45;
            msg.are_motors_ready = true;
            msg.debug_message  = "Set to ttrue";
            publisher_->publish(msg);
        }

        rclcpp::Publisher<my_robot_interfaces::msg::HardwareStatus>::SharedPtr publisher_;

        rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<HwStatusPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}