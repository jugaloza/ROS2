#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"
#include "my_robot_interfaces/msg/led_states.hpp"

using std::placeholders::_1;
using std::placeholders::_2;


class LEDPanelNode: public rclcpp::Node
{
    public:
        LEDPanelNode(): Node("led_panel")
        {
            this->declare_parameter("led_states",rclcpp::PARAMETER_INTEGER_ARRAY);
            m_ledStates = this->get_parameter("led_states").as_integer_array();
            m_service = this->create_service<my_robot_interfaces::srv::SetLed>("set_led",std::bind(&LEDPanelNode::callback_setLed,this,_1,_2));
            m_publish = this->create_publisher<my_robot_interfaces::msg::LedStates>("led_states",10);
            m_timer = this->create_wall_timer(std::chrono::seconds(1),std::bind(&LEDPanelNode::publish,this));
            RCLCPP_INFO(this->get_logger(),"LED Panel is started");
        }
    private:

        void publish()
        {
            auto msg = my_robot_interfaces::msg::LedStates();
            msg.led_state.at(0) = m_ledStates.at(0);

            msg.led_state.at(1) = m_ledStates.at(1);
            msg.led_state.at(2) = m_ledStates.at(2);
            m_publish->publish(msg);

        }

        void callback_setLed(const my_robot_interfaces::srv::SetLed::Request::SharedPtr request, const my_robot_interfaces::srv::SetLed::Response::SharedPtr response)
        {
            if(request->state)
            {
                auto led_number = request->led_number - 1;
                //auto msg = my_robot_interfaces::msg::LedStates();
                m_ledStates[led_number] = 1;
                //msg.led_state = m_ledStates;
                //msg.led_state.at(led_number) = 1;
                //m_publish->publish(msg);
                response->success = true;

            }
            else if (!request->state)
            {
                auto led_number = request->led_number - 1;

                //auto msg = my_robot_interfaces::msg::LedStates();
                m_ledStates[led_number]  = 0;
                //msg.led_state.at(led_number) = 0;
                //m_publish->publish(msg);
                response->success = true;
            }
        }
        rclcpp::Service<my_robot_interfaces::srv::SetLed>::SharedPtr m_service;
        rclcpp::Publisher<my_robot_interfaces::msg::LedStates>::SharedPtr m_publish;
        //std::array<int64_t,3> m_ledStates = {0,0,0};
        rclcpp::TimerBase::SharedPtr m_timer;
        std::vector<int64_t, std::allocator<int64_t>> m_ledStates;
        //int m_ledStates[3] = {0,0,0};
};

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<LEDPanelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}