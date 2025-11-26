/*
 * @Author: nuc 1078456645@qq.com
 * @Date: 2025-11-06 14:08:28
 * @LastEditors: nuc 1078456645@qq.com
 * @LastEditTime: 2025-11-11 17:05:18
 * @FilePath: /livox_ws/src/livox_repub_ros2/src/livox_repub_node.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// Point type definition with timestamp and ring for LIO-SAM
struct PointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint16_t, ring, ring)
    (float, time, time)
)

typedef PointXYZIRT PointType;

class LivoxRepub : public rclcpp::Node {
public:
    LivoxRepub() : Node("livox_repub") {
        
        auto qos_pub = rclcpp::QoS(rclcpp::KeepLast(10));
        // qos_pub.reliable();
        //         // optimized_sensor_qos.durability_volatile();

        // auto qos = rclcpp::QoS(
        //     rclcpp::QoSInitialization(
        //         RMW_QOS_POLICY_HISTORY_KEEP_LAST,  // 历史策略：保留最后几条
        //         10  // 深度：保留最近10帧点云
        //     )
        // );
        
        // qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);  // 尽力传输，允许丢包
        // qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);       // 不持久化
        // qos.deadline(rclcpp::Duration(0, 100000000));            // 100ms截止时间
        // qos.lifespan(rclcpp::Duration(1, 0));                    // 消息寿命1秒
        // qos.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);     // 自动存活性检测
        // qos.liveliness_lease_duration(rclcpp::Duration(1, 0));   // 租约持续时间1秒
        

        // Initialize subscriber and publisher
        sub_livox_msg_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
                "/livox/lidar", qos_pub,
                std::bind(&LivoxRepub::LivoxMsgCbk, this, std::placeholders::_1));
                
        pub_pcl_out_ = create_publisher<sensor_msgs::msg::PointCloud2>(
                "/livox/points", qos_pub);
    }

private:
    void LivoxMsgCbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr livox_msg) {
        RCLCPP_INFO_ONCE(this->get_logger(), "Received first LiDAR message 1");


        pcl::PointCloud<PointType>::Ptr pcl_in(new pcl::PointCloud<PointType>());

        for (unsigned int i = 0; i < livox_msg->point_num; ++i) {
            PointType pt;
            pt.x = livox_msg->points[i].x;
            pt.y = livox_msg->points[i].y;
            pt.z = livox_msg->points[i].z;

            // Set intensity (reflectivity)
            pt.intensity = livox_msg->points[i].reflectivity;
            
            // Livox Mid360's line number corresponds to ring number
            pt.ring = static_cast<uint16_t>(livox_msg->points[i].line);


            // Set timestamp for deskewing (normalized to 0-1 range)
            pt.time  = livox_msg->points[i].offset_time/1e9; 

            pcl_in->push_back(pt);
        }

        // Convert PointCloud to ROS message
        sensor_msgs::msg::PointCloud2 pcl_ros_msg;
        pcl::toROSMsg(*pcl_in, pcl_ros_msg);

        pcl_ros_msg.header = livox_msg->header;

        // Publish the converted PointCloud
        pub_pcl_out_->publish(pcl_ros_msg);
    }

    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_livox_msg_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_out_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LivoxRepub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}