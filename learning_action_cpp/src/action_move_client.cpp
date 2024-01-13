/***
@作者: 古月居(www.guyuehome.com)
@说明: ROS2动作示例-请求执行圆周运动动作的客户端
***/

#include <iostream>

#include "rclcpp/rclcpp.hpp"                          // ROS2 C++接口库
#include "rclcpp_action/rclcpp_action.hpp"            // ROS2 动作类
#include "learning_interface/action/move_circle.hpp"  // 自定义的圆周运动接口

using namespace std;

class MoveCircleActionClient : public rclcpp::Node
{
    public:
        // 定义一个自定义的动作接口类，便于后续使用
        using CustomAction = learning_interface::action::MoveCircle;
        // 定义一个处理动作请求、取消、执行的客户端类
        using GoalHandle = rclcpp_action::ClientGoalHandle<CustomAction>;

        explicit MoveCircleActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
        : Node("action_move_client", node_options)                            // ROS2节点父类初始化
        {
            this->client_ptr_ = rclcpp_action::create_client<CustomAction>(   // 创建动作客户端（接口类型、动作名）
                this->get_node_base_interface(),
                this->get_node_graph_interface(),
                this->get_node_logging_interface(),
                this->get_node_waitables_interface(),
                "move_circle");
        }

        // 创建一个发送动作目标的函数
        void send_goal(bool enable)
        {
            // 检查动作服务器是否可以使用
            if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) 
            {
                RCLCPP_ERROR(this->get_logger(), "Client: Action server not available after waiting");
                rclcpp::shutdown();
                return;
            }

            // 绑定动作请求、取消、执行的回调函数
            auto send_goal_options = rclcpp_action::Client<CustomAction>::SendGoalOptions();
            using namespace std::placeholders;
            send_goal_options.goal_response_callback =
                std::bind(&MoveCircleActionClient::goal_response_callback, this, _1);
            send_goal_options.feedback_callback =
                std::bind(&MoveCircleActionClient::feedback_callback, this, _1, _2);
            send_goal_options.result_callback =
                std::bind(&MoveCircleActionClient::result_callback, this, _1);

            // 创建一个动作目标的消息
            auto goal_msg = CustomAction::Goal();
            goal_msg.enable = enable;
            // 异步方式发送动作的目标
            RCLCPP_INFO(this->get_logger(), "Client: Sending goal");
                this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        }

    private:
        rclcpp_action::Client<CustomAction>::SharedPtr client_ptr_;

        // 创建一个服务器收到目标之后反馈时的回调函数
        void goal_response_callback(GoalHandle::SharedPtr goal_message)
        {
            if (!goal_message)
            {
                RCLCPP_ERROR(this->get_logger(), "Client: Goal was rejected by server");
                rclcpp::shutdown(); // Shut down client node
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Client: Goal accepted by server, waiting for result");
            }
        }

        // 创建处理周期反馈消息的回调函数
        void feedback_callback(
            GoalHandle::SharedPtr,
            const std::shared_ptr<const CustomAction::Feedback> feedback_message)
        {
            std::stringstream ss;
            ss << "Client: Received feedback: "<< feedback_message->state;
            RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
        }

        // 创建一个收到最终结果的回调函数
        void result_callback(const GoalHandle::WrappedResult & result_message)
        {
            switch (result_message.code)
            {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Client: Goal was aborted");
                    rclcpp::shutdown(); // 关闭客户端节点
                    return;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(this->get_logger(), "Client: Goal was canceled");
                    rclcpp::shutdown(); // 关闭客户端节点
                    return;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Client: Unknown result code");
                    rclcpp::shutdown(); // 关闭客户端节点
                    return;
            }
            RCLCPP_INFO(this->get_logger(), "Client: Result received: %s", (result_message.result->finish ? "true" : "false"));
            rclcpp::shutdown();         // 关闭客户端节点
        }
};

// ROS2节点主入口main函数
int main(int argc, char * argv[])                                    
{
    // ROS2 C++接口初始化
    rclcpp::init(argc, argv);   
    
    // 创建一个客户端指针                                     
    auto action_client = std::make_shared<MoveCircleActionClient>(); 
    
    // 发送动作目标
    action_client->send_goal(true);     
    
    // 创建ROS2节点对象并进行初始化                            
    rclcpp::spin(action_client);  
    
    // 关闭ROS2 C++接口                                   
    rclcpp::shutdown();                                              
    
    return 0;
}
