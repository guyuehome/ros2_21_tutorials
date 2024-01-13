/***
@作者: 古月居(www.guyuehome.com)
@说明: ROS2话题示例-发布图像话题
***/

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"          // ROS2 C++接口库
#include "std_msgs/msg/string.hpp"    // 字符串消息类型

using namespace std::chrono_literals;


class PublisherNode : public rclcpp::Node
{
    public:
        PublisherNode()
        : Node("topic_helloworld_pub") // ROS2节点父类初始化
        {
            // 创建发布者对象（消息类型、话题名、队列长度）
            publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10); 
            // 创建一个定时器,定时执行回调函数
            timer_ = this->create_wall_timer(
                500ms, std::bind(&PublisherNode::timer_callback, this));            
        }

    private:
        // 创建定时器周期执行的回调函数
        void timer_callback()                                                       
        {
          // 创建一个String类型的消息对象
          auto msg = std_msgs::msg::String();   
          // 填充消息对象中的消息数据                                    
          msg.data = "Hello World";
          // 发布话题消息                                                 
          RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str()); 
          // 输出日志信息，提示已经完成话题发布   
          publisher_->publish(msg);                                                
        }
        
        rclcpp::TimerBase::SharedPtr timer_;                             // 定时器指针
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;  // 发布者指针
};

// ROS2节点主入口main函数
int main(int argc, char * argv[])                      
{
    // ROS2 C++接口初始化
    rclcpp::init(argc, argv);                
    
    // 创建ROS2节点对象并进行初始化          
    rclcpp::spin(std::make_shared<PublisherNode>());   
    
    // 关闭ROS2 C++接口
    rclcpp::shutdown();                                

    return 0;
}
