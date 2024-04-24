#ifndef POINT_CLOUD_MERGE_H
#define POINT_CLOUD_MERGE_H
#include <iostream>
#include <memory>
#include <chrono>
#include <vector>
#include <deque>
#include <string>
#include <QObject>
#include "scenario_datatype.h"

#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include <QDebug>
#include <QList>

#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#endif // POINT_CLOUD_MERGE_H

using namespace std::chrono_literals;
using point_cloud_msg_wrapper::PointCloud2Modifier;

class LidarSubscriber :public QObject, public rclcpp::Node
{
Q_OBJECT
public:
    explicit LidarSubscriber();
    explicit LidarSubscriber(const rclcpp::NodeOptions & node_options);
    LidarSubscriber(const rclcpp::NodeOptions & node_options, int queue_size);

public slots:
    void handleDataFromMain(const QString& data);
    void handleLidarInfo(QList<LidarInfo> lidarinfos);
signals:
    void sendValueToMain(QString value);

private:

    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    std::deque<geometry_msgs::msg::TwistStamped::ConstSharedPtr> twist_ptr_queue_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_output_;
    int maximum_queue_size_ = 4;
    double timeout_sec_ = 0.1;
    std::vector<double> input_offset_;
    std::set<std::string> not_subscribed_topic_names_;
    std::string output_frame_;

    rclcpp::TimerBase::SharedPtr timer_;
    diagnostic_updater::Updater updater_{this};

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_twist_;

    std::map<std::string, sensor_msgs::msg::PointCloud2::ConstSharedPtr> cloud_stdmap_;
    std::map<std::string, sensor_msgs::msg::PointCloud2::ConstSharedPtr> cloud_stdmap_tmp_;
    std::mutex mutex_;

    std::map<std::string, double> offset_map_;

    std::vector<std::string> input_topics;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subscription;
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST,1));

    bool isSaveBin=false;
    int fileName=0;
    QList<LidarInfo> li;
    std::vector<geometry_msgs::msg::TransformStamped> transform_main;
    std::vector<std::shared_ptr<tf2_ros::StaticTransformBroadcaster>> tf_publisher;

    tf2::Quaternion total_rotation_;
    bool startLabel=false;
    void transformPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in, sensor_msgs::msg::PointCloud2::SharedPtr & out);
    void combineClouds(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in1, const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in2,sensor_msgs::msg::PointCloud2::SharedPtr & out);
    void publish();
    void timer_callback();
    void test();
    void setPeriod(const int64_t new_period);
    void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input_ptr, const std::string & topic_name);
    bool timestamps_within_threshold(const std::map<std::string, std::shared_ptr<const sensor_msgs::msg::PointCloud2>>& cloud_map, const std::chrono::milliseconds& threshold);
    void save_to_bin(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const std::string& filename);
};
