/***
@作者: 古月居(www.guyuehome.com)
@说明: ROS2服务示例-提供加法器的服务器处理功能
***/

#include "rclcpp/rclcpp.hpp"                         // ROS2 C++接口库
#include "learning_interface/srv/add_two_ints.hpp"   // 自定义的服务接口

#include <memory>

// 创建回调函数，执行收到请求后对数据的处理
void adderServer(const std::shared_ptr<learning_interface::srv::AddTwoInts::Request> request, 
          std::shared_ptr<learning_interface::srv::AddTwoInts::Response>      response)
{
    // 完成加法求和计算，将结果放到反馈的数据中
    response->sum = request->a + request->b;
    // 输出日志信息，提示已经完成加法求和计算                                                    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",             
                request->a, request->b);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

// ROS2节点主入口main函数
int main(int argc, char **argv)                                                                 
{
    // ROS2 C++接口初始化
    rclcpp::init(argc, argv);
    // 创建ROS2节点对象并进行初始化                                                                   
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("service_adder_server");   
    // 创建服务器对象（接口类型、服务名、服务器回调函数）  
    rclcpp::Service<learning_interface::srv::AddTwoInts>::SharedPtr service =
        node->create_service<learning_interface::srv::AddTwoInts>("add_two_ints", &adderServer);  
          
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");
    
    // 循环等待ROS2退出
    rclcpp::spin(node);    
    // 关闭ROS2 C++接口                                                                     
    rclcpp::shutdown();                                                                         
}
