#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class sub_from_nav : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    sub_from_nav(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "大家好，我是%s.", name.c_str());

        this->declare_parameter<std::string>("topic_name", "communication_data");
        const auto topic_name = this->get_parameter("topic_name").as_string();

        sub_ = this->create_subscription<std_msgs::msg::String>(
            topic_name,
            10,
            std::bind(&sub_from_nav::msg_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "开始监听话题: %s", topic_name.c_str());
    }

private:
    void msg_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "收到数据: %s", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<sub_from_nav>("sub_from_nav");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
