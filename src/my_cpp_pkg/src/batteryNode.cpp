#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"


class BatteryNode : public rclcpp::Node
{
    public:
        BatteryNode():  Node("Battery"), battery_state(std::string("Full")), time_since_battery_last_charged(this->get__current_time__seconds())
        {
            m_battery_timer = this->create_wall_timer(std::chrono::microseconds(500),std::bind(&BatteryNode::check_battery_state,this));
        }

        double get__current_time__seconds()
        {
            m_clockPtr = this->get_clock();
            return m_clockPtr->now().seconds();
            
        }

        void check_battery_state()
        {
            auto current_time = get__current_time__seconds();

            if (battery_state == "Full")
            {
                if (current_time - time_since_battery_last_charged > 4)
                {
                    RCLCPP_INFO(this->get_logger(),"Battery empty");;
                    battery_state = "Empty";
                    call_set_led(2,true);
                }
            }
            else 
            {
                if (current_time - time_since_battery_last_charged > 6)
                {
                    RCLCPP_INFO(this->get_logger(),"Battery Full");
                    battery_state = "Full";
                    call_set_led(2,false);
                }
            }
        }

        void call_set_led(int led,bool isOn)
        {
            auto client = this->create_client<my_robot_interfaces::srv::SetLed>("set_led");

            while(!client->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_WARN(this->get_logger(),"SET LED server is unavailable");
            }

            auto request = std::make_shared<my_robot_interfaces::srv::SetLed::Request>();

            request->led_number =  led;
            request->state = isOn;

            auto future = client->async_send_request(request);

            try
            {
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(),"led no %d is now %d and it is %d",led,isOn,response->success);
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(this->get_logger(),"Error while getting response");
            }
            
        }
    private:

        //std::thread thread_;
        std::string battery_state;
        rclcpp::TimerBase::SharedPtr m_battery_timer;
        rclcpp::Clock::SharedPtr  m_clockPtr;
        double time_since_battery_last_charged;
};

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<BatteryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
