/***
@作者: 古月居(www.guyuehome.com)
@说明: ROS2节点示例-发布“Hello World”日志信息, 使用面向对象的实现方式
***/

#include "rclcpp/rclcpp.hpp"


/***
创建一个HelloWorld节点, 初始化时输出“hello world”日志
***/
class HelloWorldNode : public rclcpp::Node
{
    public:
        HelloWorldNode()
        : Node("node_helloworld_class")                          // ROS2节点父类初始化
        {
            while(rclcpp::ok())                                  // ROS2系统是否正常运行
            {
                RCLCPP_INFO(this->get_logger(), "Hello World");  // ROS2日志输出
                sleep(1);                                        // 休眠控制循环时间
            }
        }
};

// ROS2节点主入口main函数
int main(int argc, char * argv[])                               
{

    // ROS2 C++接口初始化
    rclcpp::init(argc, argv);        
    
    // 创建ROS2节点对象并进行初始化                 
    rclcpp::spin(std::make_shared<HelloWorldNode>()); 
    
    // 关闭ROS2 C++接口
    rclcpp::shutdown();                               
    
    return 0;
}
