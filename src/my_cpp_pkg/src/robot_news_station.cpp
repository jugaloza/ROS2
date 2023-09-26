#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

class RobotNewsStationNode: public rclcpp::Node
{
public:
    RobotNewsStationNode(): Node("robot_news_station"), m_robotName("R2D2")
    {
        m_publisher = this->create_publisher<example_interfaces::msg::String>("robot_news",10);


        m_timer = this->create_wall_timer(std::chrono::milliseconds(5000),std::bind(&RobotNewsStationNode
        ::callback_robotNews,this));

        RCLCPP_INFO(this->get_logger(),"Robot news Station has been started. ");  
    
    }

private:

    void callback_robotNews()
    {
        auto msg = example_interfaces::msg::String();
        msg.data = std::string("Hi this is ") + m_robotName + " from robot news station. ";
        m_publisher->publish(msg);
    }
    std::string m_robotName;
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr m_publisher;
    rclcpp::TimerBase::SharedPtr m_timer;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<RobotNewsStationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}