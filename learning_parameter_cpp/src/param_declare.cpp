/***
@作者: 古月居(www.guyuehome.com)
@说明: ROS2参数示例-创建、读取、修改参数
***/

#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>    // ROS2 C++接口库

using namespace std::chrono_literals;

class ParameterNode : public rclcpp::Node
{
    public:
        ParameterNode()
        : Node("param_declare")                                             // ROS2节点父类初始化
        {
            this->declare_parameter("robot_name", "mbot");

            timer_ = this->create_wall_timer(                               
                1000ms, std::bind(&ParameterNode::timer_callback, this));   // 创建一个定时器，定时执行回调函数
        }
         
        // 创建定时器周期执行的回调函数 
        void timer_callback()     
        {
            std::string robot_name_param = this->get_parameter("robot_name").as_string();                // 从ROS2系统中读取参数的值
            RCLCPP_INFO(this->get_logger(), "Hello %s!", robot_name_param.c_str());                      // 输出日志信息，打印读取到的参数值
            std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("robot_name", "mbot")};  // 重新将参数值设置为指定值
            this->set_parameters(all_new_parameters);                                                    // 将重新创建的参数列表发送给ROS2系统
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;
};

// ROS2节点主入口main函数
int main(int argc, char ** argv)                         
{

    // ROS2 C++接口初始化
    rclcpp::init(argc, argv);       
    
    // 循环等待ROS2退出                   
    rclcpp::spin(std::make_shared<ParameterNode>());   
    
    // 关闭ROS2 C++接口
    rclcpp::shutdown();                                
      
    return 0;
}
