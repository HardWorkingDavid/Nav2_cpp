#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


class Nav_grab: public rclcpp::Node
{
private:
    using Nav2_client= nav2_msgs::action::NavigateToPose;
	using ClientGoalHandle = rclcpp_action::ClientGoalHandle<Nav2_client>;
    rclcpp_action::Client<Nav2_client>::SharedPtr goal_client;

public:

    Nav_grab(const std::string &node_name):Node(node_name)
    {
        goal_client =  rclcpp_action::create_client<Nav2_client>(this,"navigate_to_pose");
    }
    void send_goal(void)
    {
        if (!goal_client->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available!");
            rclcpp::shutdown();
        }

        auto goal = Nav2_client::Goal();
        goal.pose.header.stamp = this->now();
        goal.pose.header.frame_id = "map";
        goal.pose.pose.position.x = 1.0;
        goal.pose.pose.position.y = 1.0;
        goal.pose.pose.position.z = 0;
        goal.pose.pose.orientation.x = 0;
        goal.pose.pose.orientation.y = 0;
        goal.pose.pose.orientation.z = 0;
        goal.pose.pose.orientation.w = 1;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<Nav2_client>::SendGoalOptions();
        send_goal_options.goal_response_callback = 
            std::bind(&Nav_grab::goal_response_callback,this,std::placeholders::_1);
        send_goal_options.feedback_callback = 
            std::bind(&Nav_grab::feedback_callback,this,std::placeholders::_1,std::placeholders::_2);
        send_goal_options.result_callback = 
            std::bind(&Nav_grab::result_callback,this,std::placeholders::_1);

        goal_client->async_send_goal(goal,send_goal_options);
    }
    

    void goal_response_callback(const std::shared_ptr<ClientGoalHandle> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        }
        else RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }

/**************************************************************************
函数功能：动作反馈信息处理回调函数
返回  值：无
**************************************************************************/
    void feedback_callback(ClientGoalHandle::SharedPtr,const std::shared_ptr<const Nav2_client::Feedback> feedback)
    {
         RCLCPP_INFO(this->get_logger(),"Remaining Distance from Destination: %f",feedback->distance_remaining);
    }

/**************************************************************************
函数功能：动作结果回调函数
返回  值：无
**************************************************************************/
    void result_callback(const ClientGoalHandle::WrappedResult &result)
    {
        switch(result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                // 处理事件
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                //  处理事件
                return;
            case rclcpp_action::ResultCode::CANCELED:
                //  处理事件
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return;
            default:
                // 处理事件
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }
        RCLCPP_INFO(this->get_logger(), "Result received");
    }

};


int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node=std::make_shared<Nav_grab>("nav_grab_node");
    node->send_goal();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
