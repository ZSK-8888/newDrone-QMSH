/**
 * @file offb_node.cpp
 * @brief Offboard 控制示例 C++ 节点，与 PX4 官方文档同步
 *
 * 这个节点演示了如何：
 * 1. 订阅飞控状态 (mavros/state)
 * 2. 订阅当前本地位置 (mavros/local_position/pose)
 * 3. 发布目标本地位置 (mavros/setpoint_position/local)
 * 4. 调用服务来解锁 (mavros/cmd/arming)
 * 5. 调用服务来设置模式 (mavros/set_mode)
 *
 * 遵循 PX4 的 Offboard 模式安全握手流程。
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// 用于存储从飞控获取的当前位置
geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}
int main(int argc, char **argv)
{
    setlocale(LC_CTYPE, "");
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // 订阅飞控状态
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
            
    // 订阅飞控的当前本地位置
    // 注意：PX4 SITL 启动时可能不会立即发布这个话题，直到它获得位置估计
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);

    // 发布目标本地位置
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    // MAVROS 服务客户端：用于解锁
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");

    // MAVROS 服务客户端：用于设置模式
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    // ROS 循环频率
    ros::Rate rate(20.0);

    // 等待 MAVROS 连接到飞控 (FCU)
    // `current_state` 会通过 state_cb 回调函数更新
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("MAVROS 已连接到飞控 (FCU)");

    // 目标姿态：飞到 (0, 0, 2)
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    // 在进入 Offboard 模式前，必须先发送一些设定点
    // PX4 需要这个来确保有一个外部控制器在运行
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("已发送初始设定点");

    // 准备 Offboard 模式请求
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // 准备解锁请求
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    // ========= 进入主控制循环 =========
    while(ros::ok()){
        // 1. 尝试切换到 Offboard 模式
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard 模式已启用");
            }
            last_request = ros::Time::now();
        } 
        // 2. 如果已进入 Offboard 模式，尝试解锁
        else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("无人机已解锁");
                }
                last_request = ros::Time::now();
            }
        }
        
        // 3. 在 Offboard 模式并解锁后，持续发布目标点
        // (即使已经解锁，也要持续发布)
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}