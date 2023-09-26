#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/msg/string.hpp"

class NumberPublisher: public rclcpp::Node
{
    public:

        NumberPublisher(): Node("number_publisher")
        {
            //this->declare_parameter("text",exa)
            this->declare_parameter("number_to_publish",2);
            this->declare_parameter("frequency_to_publish",1.0);

            m_counter = this->get_parameter("number_to_publish").as_int();
            double m_frequency_to_publish = this->get_parameter("frequency_to_publish").as_double();

            m_publisher = this->create_publisher<example_interfaces::msg::Int64>("number",10);
            RCLCPP_INFO(this->get_logger(),"Number publisher has been started.");

            m_timer = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0 / m_frequency_to_publish)),std::bind(&NumberPublisher::callback_publisher,this));;
        }

    private:


        void callback_publisher()
        {
            //m_counter += 1;
            auto msg = example_interfaces::msg::Int64();
            msg.data  = m_counter;

            m_publisher->publish(msg);
        }

        rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr m_publisher;
        rclcpp::TimerBase::SharedPtr m_timer;
        int m_counter;

};

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<NumberPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
