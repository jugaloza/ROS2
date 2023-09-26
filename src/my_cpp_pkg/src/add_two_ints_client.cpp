#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class AddTwoIntsClientNode: public rclcpp::Node
{
    public:


        AddTwoIntsClientNode(): Node("add_two_ints_client")
        {
            thread_ = std::thread(std::bind(&AddTwoIntsClientNode::call_add_two_ints,this,4,3));
        }

        void call_add_two_ints(int  a,int b)
        {
            auto client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

            while(!client->wait_for_service(std::chrono::seconds(1))){

                RCLCPP_WARN(this->get_logger(),"Waiting for server to connect.");
            }

            auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();;


            request->a = a;
            request->b = b;

            auto future = client->async_send_request(request);

            try
            {
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(),"%ld + %ld = %ld",request->a,request->b,response->sum);
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(this->get_logger(),"Unable to get response from server");
            }
            
        }

    private:
        std::thread thread_;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<AddTwoIntsClientNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}