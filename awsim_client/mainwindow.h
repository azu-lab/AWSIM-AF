#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <yaml-cpp/yaml.h>
#include <chrono>

#include <lanelet2_io/Io.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_projection/UTM.h>

#include "mgrs_projector.hpp"
#include "scenario_datatype.h"
#include <boost/filesystem.hpp>
#include "point_cloud_widget.h"

#include <cstdio>
//#include "SendScenario.h"
//#include "ScenarioDataType.h"
#include <QPushButton>
#include <QTcpSocket>
#include <QDebug>
#include <QList>
#include <QByteArray>
#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonArray>
#include <QFileDialog>
#include <QStringListModel>
#include <QListView>
#include <QStandardItemModel>
#include <vector>
#include <QtWebEngineWidgets/QWebEngineView>
#include <QVector3D>
#include <QVector4D>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLBuffer>
#include <QTimer>
#include <QFile>
#include <QTextStream>
#include <QMessageBox>
#include <QThread>
#include <QList>

#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <GL/glu.h>


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    QTcpSocket *tcpClient;
    //SendScenarioThread sendScenario;
    Eigen::Vector2d convert_lanelet_to_world(lanelet::Id id,double s_offset,double t_offset);
    Eigen::Vector2d convert_world_to_awsim(Eigen::Vector2d point);
    lanelet::LaneletMapPtr map;
    void add_init_position();
    void add_other_position();
    bool isSaveBin=false;
    bool isSaveTxt=false;
    int fileName=0;

public: signals:
    //void requestDraw();
    void requestDraw(QVector<GLfloat>,QVector<GLuint>);
    void newPointCloudReceived(const QVector<QVector3D>& points);
    void savePointcloutToBin();
    void saveInforToTxt();

private slots:
    void on_sendScenarioButton_clicked();
    void on_socket_ready_read();
    void on_pushButton_clicked();

    void onNewPointCloudReceived(const QVector<QVector3D>& points);

    void on_load_yaml_pushButton_clicked();

    void on_vehicle_listView_clicked(const QModelIndex &index);

    void on_vehicle_detail_listView_clicked(const QModelIndex &index);

    void on_pushButton_8_clicked();

    void on_checkBox_stateChanged(int arg1);

    void on_pushButton_9_clicked(bool checked);

    void on_pushButton_10_clicked(bool checked);

    void on_pushButton_11_clicked(bool checked);

    void on_pushButton_12_clicked(bool checked);

    void on_pushButton_13_clicked(bool checked);

    void on_pushButton_14_clicked(bool checked);

public slots:
    void handleDataFromRos(const QString& data);

signals:
    void sendValueToROS(QString value);
    void sendLidarValueToRos(QList<LidarInfo> lidarinfos);

private:
    Ui::MainWindow *ui;
    QString scenarioFileName = "";
    YAML::Node scenarioYamlObject;
    QStringListModel *vehicleListModel;
    QStringList vehicleList;
    QWebEngineView *view;

    //QStringListModel *wayPointListModel;
    //QStringList wayPointList;
    QStandardItemModel *wayPointListModel;
    //WayPoint wayPoint[10];
    std::vector<WayPoint> wayPoints[10];
    int vehicleListRow;
    int wayPointListRow;
    QStandardItem *point[10];

    PointCloudOpenGLWidget *pointCloudWidget;
    //QVector<GLfloat> computeCubeVerticesOpenGL(const QVector3D& centerUnity, const QVector3D& sizeUnity, const QQuaternion& rotation);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

    void initRos2();
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    QList<Vehicle> parseJsonToVehicles(const QJsonDocument &jsonDoc);

    QList<Vehicle> VehicleInformation;

    void save_to_bin(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const std::string& filename);

    bool saveTextToFile(const QString &filePath, const QString &content);

    QString vehicleToString(const Vehicle &vehicle);

    QVector3D convert_unityxyz_to_local(const QVector3D& centerUnity);
    QVector3D convert_unityxyz_to_mid70_local(const QVector3D& centerUnity);

    QTimer *timer;

    void myMsleep(int msec);
    std::chrono::duration<double, std::milli> totalTime;

    QVector3D convert_mid70_unityxyz_to_local(const QVector3D& centerUnity);
    void saveVehicleInformation();
    int ab = 0;
    QList<LidarInfo> lidarinfos;
    QList<QCheckBox*> checkBoxList;
    //QMap<QCheckBox*, LidarInfo> lidarMap;

};


#endif // MAINWINDOW_H
