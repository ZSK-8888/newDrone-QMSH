/**
 * @file offb_node.cpp
 * @brief Offboard 控制示例 C++ 节点 (带状态机)
 *
 * 这个节点演示了如何使用一个简单的状态机来执行一个任务：
 * 1. 等待 Offboard 和 Arm
 * 2. 起飞到 2 米
 * 3. 飞到点 A
 * 4. 飞到点 B
 * 5. 返回原点上空
 * 6. 自动降落
 *
 * 遵循 PX4 的 Offboard 模式安全握手流程。
 */

#include <ros/ros.h>
#include <cmath> // 用于 pow 和 sqrt
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h> // 用于降落服务

// 定义飞行状态
enum class FlightState {
    INIT,
    TAKEOFF,
    MOVE_TO_A,
    MOVE_TO_B,
    RETURN_HOME,
    LAND
};

// --- 全局变量 ---
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;

// --- 回调函数 ---
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

// --- 辅助函数 ---
/**
 * @brief 计算两个姿态点之间的3D距离
 */
double get_distance(const geometry_msgs::PoseStamped& pos1, const geometry_msgs::PoseStamped& pos2) {
    double dx = pos1.pose.position.x - pos2.pose.position.x;
    double dy = pos1.pose.position.y - pos2.pose.position.y;
    double dz = pos1.pose.position.z - pos2.pose.position.z;
    return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2) + std::pow(dz, 2));
}

// --- 主函数 ---
int main(int argc, char **argv)
{
    setlocale(LC_CTYPE, "");
    ros::init(argc, argv, "offb_node_statemachine");
    ros::NodeHandle nh;

    // --- ROS 订阅者, 发布者, 服务客户端 ---
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/land");

    // 循环频率
    ros::Rate rate(20.0);

    // --- 1. 等待 MAVROS 连接和位置数据 ---
    while(ros::ok() && !current_state.connected){
        ROS_INFO_ONCE("等待 MAVROS 连接到飞控...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("MAVROS 已连接");

    while(ros::ok() && local_pose_sub.getNumPublishers() == 0) {
        ROS_INFO_ONCE("等待 /mavros/local_position/pose 话题发布...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("SITL 已连接并发布位置。");

    // --- 2. 定义任务航点 ---
    geometry_msgs::PoseStamped pose_takeoff;
    pose_takeoff.pose.position.x = 0;
    pose_takeoff.pose.position.y = 0;
    pose_takeoff.pose.position.z = 2.0;

    geometry_msgs::PoseStamped pose_A;
    pose_A.pose.position.x = 5.0;
    pose_A.pose.position.y = 2.0;
    pose_A.pose.position.z = 2.0;

    geometry_msgs::PoseStamped pose_B;
    pose_B.pose.position.x = 5.0;
    pose_B.pose.position.y = -2.0;
    pose_B.pose.position.z = 2.0;

    geometry_msgs::PoseStamped pose_Return;
    pose_Return.pose.position.x = 0.0;
    pose_Return.pose.position.y = 0.0;
    pose_Return.pose.position.z = 2.0;

    // 动态目标点
    geometry_msgs::PoseStamped target_pose = pose_takeoff; // 我们的第一个目标是起飞

    // --- 3. 发送初始设定点 (PX4 安全握手) ---
    // 在请求 Offboard 之前，必须先开始流式传输设定点
    // 我们将流式传输我们的第一个任务点：起飞点
    ROS_INFO("发送初始设定点 (起飞点)...");
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(target_pose);
        ros::spinOnce();
        rate.sleep();
    }

    // --- 4. 初始化状态机和服务请求 ---
    FlightState current_flight_state = FlightState::INIT;
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.min_pitch = 0.0;
    land_cmd.request.yaw = 0.0;
    land_cmd.request.latitude = 0.0; // 0,0,0 会触发在当前位置降落
    land_cmd.request.longitude = 0.0;
    land_cmd.request.altitude = 0.0;

    ros::Time last_request = ros::Time::now();
    double pos_tolerance = 0.25; // 25cm 位置容忍误差

    ROS_INFO("准备进入主控制循环...");

    // --- 5. 主控制循环 ---
    while(ros::ok()){
        
        // --- 状态机：INIT 状态 ---
        // 这个状态只负责解锁和切换到 Offboard 模式
        if (current_flight_state == FlightState::INIT) {
            if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                    ROS_INFO("Offboard 模式已启用");
                }
                last_request = ros::Time::now();
            } else {
                if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
                    if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                        ROS_INFO("无人机已解锁");
                    }
                    last_request = ros::Time::now();
                }
            }
            
            // 检查是否成功
            if (current_state.mode == "OFFBOARD" && current_state.armed) {
                ROS_INFO("系统准备就绪。切换到 TAKEOFF 状态。");
                current_flight_state = FlightState::TAKEOFF;
            }
        }
        
        // --- 状态机：飞行任务状态 ---
        else {
            
            double distance;
            switch (current_flight_state) {
                case FlightState::TAKEOFF:
                    target_pose = pose_takeoff; // 确保目标是起飞点
                    distance = get_distance(current_pose, target_pose);
                    if (distance < pos_tolerance) {
                        ROS_INFO("已到达起飞高度，前往点 A");
                        current_flight_state = FlightState::MOVE_TO_A;
                    }
                    break;
                
                case FlightState::MOVE_TO_A:
                    target_pose = pose_A; // 设置新目标
                    distance = get_distance(current_pose, target_pose);
                    if (distance < pos_tolerance) {
                        ROS_INFO("已到达点 A，前往点 B");
                        ROS_INFO("现在位置%.2f %.2f %.2f",current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z);
                        current_flight_state = FlightState::MOVE_TO_B;
                    }
                    break;
                    
                case FlightState::MOVE_TO_B:
                    target_pose = pose_B; // 设置新目标
                    distance = get_distance(current_pose, target_pose);
                    if (distance < pos_tolerance) {
                        ROS_INFO("已到达点 B，返回原点上方");
                        ROS_INFO("现在位置%.2f %.2f %.2f",current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z);
                        current_flight_state = FlightState::RETURN_HOME;
                    }
                    break;
                    
                case FlightState::RETURN_HOME:
                    target_pose = pose_Return; // 设置新目标
                    distance = get_distance(current_pose, target_pose);
                    if (distance < pos_tolerance) {
                        ROS_INFO("已返回原点上方，准备降落");
                        ROS_INFO("现在位置%.2f %.2f %.2f",current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z);
                        current_flight_state = FlightState::LAND;
                        if(land_client.call(land_cmd) && land_cmd.response.success){
                            ROS_INFO("正在降落...");
                        } else {
                            ROS_ERROR("降落服务调用失败。");
                            // 可以在此添加备用降落方案，例如设置 z=0
                        }
                    }
                    break;

                case FlightState::LAND:
                    // 飞机正在自动降落
                    // 我们可以通过检查是否已降落并自动锁定来结束
                    if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(10.0))) {
                         ROS_INFO("已降落并锁定。任务完成。");
                         ros::shutdown(); // 结束节点
                    }
                    break;

                default: // INIT 在循环开始时已处理
                    break;
            }
            
        } 

        // 无论在哪个状态，都必须持续发布当前的目标点
        local_pos_pub.publish(target_pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

