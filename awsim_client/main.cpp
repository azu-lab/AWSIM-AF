#include "mainwindow.h"
#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include "point_cloud_merge.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    // ROS 2初始化
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarSubscriber>();

    // 在另一个线程中运行ROS 2节点，以避免阻塞Qt的事件循环
    std::thread ros_thread([node]() {
        rclcpp::spin(node);
        rclcpp::shutdown();
    });

    MainWindow w;

    QObject::connect(&w, &MainWindow::sendValueToROS, node.get(), &LidarSubscriber::handleDataFromMain);
    QObject::connect(node.get(), &LidarSubscriber::sendValueToMain, &w, &MainWindow::handleDataFromRos);
    QObject::connect(&w,&MainWindow::sendLidarValueToRos,node.get(),&LidarSubscriber::handleLidarInfo);

    w.show();  // 显示主窗口

    int result = a.exec();  // 开始Qt的事件循环

    if (ros_thread.joinable())
        ros_thread.join();  // 等待ROS线程结束

    return result;
}
