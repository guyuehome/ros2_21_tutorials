/***
@作者: 古月居(www.guyuehome.com)
@说明: ROS2话题示例-订阅“Hello World”话题消息
***/

#include <memory>

#include "rclcpp/rclcpp.hpp"                  // ROS2 C++接口库
#include "std_msgs/msg/string.hpp"            // 字符串消息类型
using std::placeholders::_1;

class SubscriberNode : public rclcpp::Node
{
    public:
        SubscriberNode()
        : Node("topic_helloworld_sub")        // ROS2节点父类初始化
        {
            subscription_ = this->create_subscription<std_msgs::msg::String>(       
                "chatter", 10, std::bind(&SubscriberNode::topic_callback, this, _1));   // 创建订阅者对象（消息类型、话题名、订阅者回调函数、队列长度）
        }

    private:
        void topic_callback(const std_msgs::msg::String & msg) const                  // 创建回调函数，执行收到话题消息后对数据的处理
        {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());       // 输出日志信息，提示订阅收到的话题消息
        }
        
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;         // 订阅者指针
};

// ROS2节点主入口main函数
int main(int argc, char * argv[])                         
{
    // ROS2 C++接口初始化
    rclcpp::init(argc, argv);                 
    
    // 创建ROS2节点对象并进行初始化            
    rclcpp::spin(std::make_shared<SubscriberNode>());     
    
    // 关闭ROS2 C++接口
    rclcpp::shutdown();                                  
    
    return 0;
}
