#include "mainwindow.h"
#include "./ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    //
    //auto condig = YAML::LoadFile("../scenario/all-in-one.yaml");

    //qDebug()<<QString::fromStdString(condig["OpenSCENARIO"]["FileHeader"]["revMajor"].as<std::string>());

    //sendScenario.start();
    tcpClient = new QTcpSocket(this);
    tcpClient->connectToHost("127.0.0.1",quint16(9999));

    connect(tcpClient,SIGNAL(readyRead()),this,SLOT(on_socket_ready_read()));
    //connect(socket, &QTcpSocket::readyRead, readDataFromServer);

    lanelet::ErrorMessages errors{};
    std::string lanelet2_filename = "/home/zzhhaa1/Desktop/awsim_client/awsim_client/lanelet2_map.osm";
    lanelet::projection::MGRSProjector projector{};
    map = lanelet::load(lanelet2_filename, projector, &errors);

    //lanelet::projection::UtmProjector projector(lanelet::Origin({35.68855194431519, 139.69142711058254}));
    //lanelet::LaneletMapPtr map = lanelet::load(lanelet2_filename, projector, &errors);



    ui->setupUi(this);
    //view = new QWebEngineView(ui->tableWidget);
    view = new QWebEngineView(ui->widget_web);
    view->resize(ui->widget_web->width(),ui->widget_web->height());
    //
    QUrl url = QUrl("http://0.0.0.0:8601/");
    view->load(url);

    //
    //


    initRos2();
    //view->show();
    pointCloudWidget = new PointCloudOpenGLWidget(this);
    //pointCloudWidget->loadBinFile("/home/zzhhaa1/Desktop/awsim_client/awsim_client/0.bin");
    ui->openGLWidget->setLayout(new QVBoxLayout(ui->openGLWidget));
    ui->openGLWidget->layout()->addWidget(pointCloudWidget);

    connect(this,&MainWindow::savePointcloutToBin,this,[this](){
        isSaveBin = true;
    });

    connect(this,&MainWindow::saveInforToTxt,this,[this](){
        isSaveTxt = true;
    });

    connect(this, &MainWindow::newPointCloudReceived, this, &MainWindow::onNewPointCloudReceived);
    bool connected = connect(this, SIGNAL(requestDraw(QVector<GLfloat>,QVector<GLuint>)), pointCloudWidget, SLOT(paintLine(QVector<GLfloat>,QVector<GLuint>)));



    wayPointListModel = new QStandardItemModel(ui->vehicle_detail_listView);
    ui->pushButton_9->setVisible(false);
    ui->pushButton_10->setVisible(false);
    //ui->pushButton_12->setVisible(false);
    ui->pushButton_13->setVisible(false);
}

MainWindow::~MainWindow()
{
    delete ui;
    tcpClient->close();
    delete tcpClient;
    rclcpp::shutdown();
}

void MainWindow::initRos2()
{
    //rclcpp::init(0, nullptr);
    node_ = rclcpp::Node::make_shared("point_cloud_viewer");

    //subscription_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>("/lidar3/pointcloud_ex", rclcpp::SensorDataQoS().keep_last(1),std::bind(&MainWindow::pointCloudCallback, this, std::placeholders::_1));
    subscription_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>("/output", rclcpp::SensorDataQoS().keep_last(1),std::bind(&MainWindow::pointCloudCallback, this, std::placeholders::_1));
    //subscription_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>("/output_cloud", rclcpp::SensorDataQoS().keep_last(1),std::bind(&MainWindow::pointCloudCallback, this, std::placeholders::_1));

    // Start a thread to spin the ROS node
    std::thread([this]() {
        rclcpp::spin(node_);
    }).detach();
}

void MainWindow::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    int x_offset = -1;
    int y_offset = -1;
    int z_offset = -1;

    for (size_t i = 0; i < msg->fields.size(); i++)
    {
        if (msg->fields[i].name == "x") x_offset = msg->fields[i].offset;
        if (msg->fields[i].name == "y") y_offset = msg->fields[i].offset;
        if (msg->fields[i].name == "z") z_offset = msg->fields[i].offset;
    }

    if (x_offset == -1 || y_offset == -1 || z_offset == -1)
    {
        qWarning() << "PointCloud2 message does not have x, y, or z field!";
        return;
    }

    QVector<QVector3D> points;

    for (size_t i = 0; i < msg->data.size(); i += msg->point_step)
    {
        float x = *reinterpret_cast<float*>(&msg->data[i + x_offset]);
        float y = *reinterpret_cast<float*>(&msg->data[i + y_offset]);
        float z = *reinterpret_cast<float*>(&msg->data[i + z_offset]);
        points.push_back(QVector3D(x, y, z));
    }
    pointCloudWidget->loadFromRos(points);


    pointCloudWidget->update();

    if(isSaveBin==true)
    {
        auto start = std::chrono::high_resolution_clock::now();
        //on_pushButton_clicked();

        //myMsleep(2000);
        //


        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);

        QString filePath="";
        if(fileName<=9)
        {
            filePath = QString::fromStdString("pointcloud")+"/"+"00000"+QString::number(fileName)+".bin"; // 请修改为你要保存的实际路径
        }
        else if(fileName<=99)
        {
            filePath = QString::fromStdString("pointcloud")+"/"+"0000"+QString::number(fileName)+".bin"; // 请修改为你要保存的实际路径
        }else
        {
            filePath = QString::fromStdString("pointcloud")+"/"+"000"+QString::number(fileName)+".bin"; // 请修改为你要保存的实际路径
        }
        save_to_bin(cloud, filePath.toStdString());
        isSaveBin=false;
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = end - start;
        ab++;
        totalTime = totalTime+duration;
        if(ab%10==0)
        {
            qDebug()<<totalTime.count();
        }

    }
    //emit newPointCloudReceived(points);

}

void MainWindow::onNewPointCloudReceived(const QVector<QVector3D>& points)
{

    //update();
}

void MainWindow::on_sendScenarioButton_clicked()
{
    //qDebug()<<"2222";

    //QJsonObject json;
    //json.insert("initial_position_x",QJsonValue("216.5098"));
    //json.insert("initial_position_y",QJsonValue("-1.16148"));
    //json.insert("initial_position_z",QJsonValue("-262.6914"));
    //json.insert("s",QJsonValue("1"));
    //json.insert("rientation_position_x1",QJsonValue("233.672"));
    //json.insert("rientation_position_y1",QJsonValue("-1.39062"));
    //json.insert("rientation_position_z1",QJsonValue("-288.3775"));
    //json.insert("rientation_position_x2",QJsonValue("249.025"));
    //json.insert("rientation_position_y2",QJsonValue("-1.617665"));
    //json.insert("rientation_position_z2",QJsonValue("-317.49"));

    QJsonArray targetsArray;
    for(const auto& wp : wayPoints)
    {
        if(wp.empty())
        {
            continue;
        }
        QJsonArray pointsArray;
        for(const auto& p : wp)
        {
            Eigen::Vector2d temp;
            lanelet::Id laneletID = p.laneid;
            temp = convert_lanelet_to_world(laneletID,p.s,p.t);

            Eigen::Vector2d temp2;
            temp2 = convert_world_to_awsim(temp);

            QJsonObject pointObject;
            pointObject["x"] = temp2.x();
            pointObject["y"] = temp2.y();
            pointsArray.append(pointObject);
        }
        QJsonObject targetObject;
        targetObject["points"] = pointsArray;
        targetsArray.append(targetObject);
    }

    QJsonObject dataObject;
    dataObject["targets"] = targetsArray;

    QJsonDocument document(dataObject);
    //document.setObject(json);
    QByteArray byteArray = document.toJson(QJsonDocument::Compact);

    tcpClient->write(byteArray);
    //int ret = tcpClient->write("11111\n");
    tcpClient->waitForBytesWritten();
    tcpClient->write("\n");
    tcpClient->waitForBytesWritten();
}

//
Eigen::Vector2d MainWindow::convert_lanelet_to_world(lanelet::Id id,double s_offset,double t_offset)
{
    //先将lanelet2转换成世界坐标
    lanelet::Lanelet lanelet = map->laneletLayer.get(id);
    lanelet::ConstLineString3d centerline = lanelet.centerline();
    auto movePoint_s = lanelet::geometry::interpolatedPointAtDistance(centerline, s_offset);
    auto movePoint_s_plus = lanelet::geometry::interpolatedPointAtDistance(centerline, s_offset+0.1);

    Eigen::Vector2d tangent_dir( movePoint_s_plus.x() - movePoint_s.x(), movePoint_s_plus.y()-movePoint_s.y());
    tangent_dir.normalize();

    Eigen::Vector2d normal_dir(-tangent_dir.y(), tangent_dir.x());
    Eigen::Vector2d final_point(movePoint_s_plus.x() + t_offset * normal_dir.x(), movePoint_s_plus.y() + t_offset * normal_dir.y());

    return final_point;
}

Eigen::Vector2d MainWindow::convert_world_to_awsim(Eigen::Vector2d point)
{
    Eigen::Vector2d temp;
    temp.x() = abs(point.y() - 50137.43);
    temp.y() = abs(point.x() - 81655.73);


    if(point.y()>50137.43 && point.x() > 81655.73)
    {
        temp.x() = -temp.x();
    }
    else if(point.y()>50137.43 && point.x() < 81655.73)
    {
        temp.x() = -temp.x();
        temp.y() = -temp.y();
    }
    else if (point.y()<50137.43 && point.x() < 81655.73)
    {
        temp.y() = -temp.y();
    }
    return temp;
}


void MainWindow::on_socket_ready_read()
{
    QByteArray data = tcpClient->readAll();
    QJsonDocument doc = QJsonDocument::fromJson(data);
    if (doc.isNull())
    {
        qDebug() << "Failed to parse JSON!";
    } else
    {
        //qDebug() << "rrrrr" << doc;
        // Handle the received JSON data
    }

    // 提取JSON对象
    QJsonObject jsonObject = doc.object();
    QString type = jsonObject["type"].toString();  // 读取type字段

    if (type == "vehicle_info")
    {
        VehicleInformation.clear();
        VehicleInformation = parseJsonToVehicles(doc);

        if(isSaveTxt)
        {
            saveVehicleInformation();
        }
    }
    else if (type == "lidar")
    {
        QJsonArray lidars = jsonObject["lidars"].toArray();
        QWidget *container = this->findChild<QWidget *>("widget_15");
        QVBoxLayout *layout = new QVBoxLayout(container);

        // 清除旧的复选框
        QLayoutItem *item;
        while ((item = layout->takeAt(0)) != nullptr)
        {
            delete item->widget();
            delete item;
        }

        // 对于每个LiDAR，创建一个复选框并添加到widget_15
        for (const auto &lidarValue : lidars)
        {
            QJsonObject lidarObject = lidarValue.toObject();
            LidarInfo lidarInfo;
            lidarInfo.name = lidarObject["name"].toString();
            QJsonObject posObject = lidarObject["position"].toObject();
            lidarInfo.position = QVector3D(
                posObject["x"].toDouble(),
                posObject["y"].toDouble(),
                posObject["z"].toDouble()
                );

            // 解析旋转信息
            QJsonObject rotObject = lidarObject["rotation"].toObject();
            lidarInfo.rotation = QVector3D(
                rotObject["x"].toDouble(),
                rotObject["y"].toDouble(),
                rotObject["z"].toDouble()
                );

            QCheckBox *checkBox = new QCheckBox(lidarInfo.name, container);
            layout->addWidget(checkBox);
            lidarinfos.append(lidarInfo);
            //以后在这里添加谁以谁为中心
            checkBoxList.append(checkBox);
        }

        container->setLayout(layout);



    }
}

void MainWindow::saveVehicleInformation()
{
    QString content = "";
    QString filePath = "";
    if(fileName <= 9) {
        filePath = QString::fromStdString("label") + "/" + "00000" + QString::number(fileName) + ".txt";
    } else if(fileName <= 99) {
        filePath = QString::fromStdString("label") + "/" + "0000" + QString::number(fileName) + ".txt";
    } else {
        filePath = QString::fromStdString("label") + "/" + "000" + QString::number(fileName) + ".txt";
    }

    for (const auto &vehicle : VehicleInformation) {
        content += vehicleToString(vehicle);
        content += "\n";
    }

    saveTextToFile(filePath, content);
    isSaveTxt = false;  // 重置保存标志
}

QString MainWindow::vehicleToString(const Vehicle &vehicle)
{
    QString result;

    // 拼接name
    if(vehicle.name=="Lexus RX450h 2015 Sample Sensor")
    {
        result="Ego";
    }
    else
    {
        int clonePose = vehicle.name.indexOf("(Clone)");
        if(clonePose != -1)
        {
            result = vehicle.name.left(clonePose);
            qDebug()<<result;
        }
        else
        {
            result=vehicle.name;
        }
    }



    //else if(vehicle.name=="Truck_2t")
    //{
    //    result=vehicle.name.chopped(3);
    //}
    //else
    //{
    //result=vehicle.name;
    //}
    //result += QString("Name: %1\n").arg(vehicle.name);

    result = result+" 0 0 0 0 0 0 0 ";

    // 拼接size
    //result += QString("Size: [%1, %2, %3]\n").arg(vehicle.size.x()).arg(vehicle.size.y()).arg(vehicle.size.z());
    result = result+QString::number(vehicle.size.y(),'f',2)+" ";
    result = result+QString::number(vehicle.size.x(),'f',2)+" ";
    result = result+QString::number(vehicle.size.z(),'f',2)+" ";
    // 拼接localPosition
    //result += QString("Local Position: [%1, %2, %3]\n").arg(vehicle.localPosition.x()).arg(vehicle.localPosition.y()).arg(vehicle.localPosition.z());

    QVector3D tempPosition=convert_unityxyz_to_local(vehicle.localPosition);  //vlp16
    //QVector3D tempPosition=convert_unityxyz_to_mid70_local(vehicle.localPosition);  //mid70

    result = result+QString::number(tempPosition.x(),'f',2)+" ";
    result = result+QString::number(tempPosition.y(),'f',2)+" ";
    result = result+QString::number(tempPosition.z(),'f',2)+" ";

    //float tempRotation = vehicle.rotation.y() + 180;
    //float angleInRadians = qDegreesToRadians(tempRotation);

    float tempRotation = vehicle.rotation.y() - 325;
    float adjustedRotation = 210 - tempRotation;
    float angleInRadians = qDegreesToRadians(adjustedRotation);
    angleInRadians = std::fmod(angleInRadians + M_PI, 2 * M_PI);
    if (angleInRadians < 0)
        angleInRadians += 2 * M_PI;
    angleInRadians -= M_PI;
    result = result+QString::number(angleInRadians, 'f', 2);
    return result;
}

QVector3D MainWindow::convert_unityxyz_to_mid70_local(const QVector3D& centerUnity)
{
    //QVector3D origin(209.1321,0,-260.204);
    QVector3D origin(223.83,0,-275.8);
    QVector3D center(abs(centerUnity.z()-origin.z()), abs(centerUnity.x()-origin.x()), centerUnity.y()-origin.y());
    if(centerUnity.x()>origin.x() && centerUnity.z()<origin.z())
    {
        //4
        //center.setX(-center.x());
        center.setY(-center.y());
    }
    else if(centerUnity.x()<origin.x() && centerUnity.z()>origin.z())
    {
        //2
        center.setX(-center.x());
    }
    else if(centerUnity.x()>origin.x() && centerUnity.z()>origin.z())
    {
        //3
        center.setX(-center.x());
        center.setY(-center.y());
    }
    center.setZ(-center.z());
    return center;
}

QVector3D MainWindow::convert_unityxyz_to_local(const QVector3D& centerUnity)
{
    QVector3D origin(209.1321,2,-260.204);
    QVector3D center(abs(centerUnity.z()-origin.z()), abs(centerUnity.x()-origin.x()), centerUnity.y()-origin.y());
    if(centerUnity.x()<origin.x() && centerUnity.z()>origin.z())
    {
        center.setX(-center.x());
        center.setY(-center.y());
    }
    else if(centerUnity.x()<origin.x() && centerUnity.z()<origin.z())
    {
        //center.setX(-center.x());
        center.setY(-center.y());
    }
    else if(centerUnity.x()>origin.x() && centerUnity.z()>origin.z())
    {
        center.setX(-center.x());
    }
    return center;
}

QList<Vehicle> MainWindow::parseJsonToVehicles(const QJsonDocument &jsonDoc)
{
    QList<Vehicle> vehiclesList;

    //QJsonDocument jsonDoc = QJsonDocument::fromJson(jsonString.toUtf8());
    if (!jsonDoc.isNull() && jsonDoc.isObject())
    {
        QJsonObject rootObj = jsonDoc.object();
        QJsonArray vehiclesArray = rootObj["vehicles"].toArray();

        for (const QJsonValue &value : vehiclesArray)
        {
            QJsonObject vehicleObj = value.toObject();

            // Extract data and create Vehicle object
            QString name = vehicleObj["name"].toString();

            QJsonObject localPositionObj = vehicleObj["localPosition"].toObject();
            QVector3D localPosition(localPositionObj["x"].toDouble(),
                                    localPositionObj["y"].toDouble(),
                                    localPositionObj["z"].toDouble());

            QJsonObject rotationObj = vehicleObj["rotation"].toObject();
            QVector3D rotation(  rotationObj["x"].toDouble(),
                                 rotationObj["y"].toDouble(),
                                 rotationObj["z"].toDouble());

            QJsonObject sizeObj = vehicleObj["size"].toObject();
            QVector3D size(sizeObj["x"].toDouble(),
                           sizeObj["y"].toDouble(),
                           sizeObj["z"].toDouble());
            QJsonObject forwardObj = vehicleObj["forward"].toObject();
            QVector3D forward(forwardObj["x"].toDouble(),
                              forwardObj["y"].toDouble(),
                              forwardObj["z"].toDouble());
            Vehicle vehicle(name, localPosition, rotation, size, forward);
            vehiclesList.append(vehicle);
        }
    }
    else
    {
        qDebug() << "Invalid JSON or not an object!";
    }

    return vehiclesList;
}

void MainWindow::on_pushButton_clicked()
{
    //timer->stop();
    // tcpClient->write("4\n");
    // tcpClient->waitForBytesWritten();

    // QThread::msleep(1000);



    // tcpClient->write("1\n");
    // tcpClient->waitForBytesWritten();
    // QThread::msleep(1000);

    // emit savePointcloutToBin();
    // emit saveInforToTxt();

    // fileName++;
    // qDebug()<<"2222";
    // //QTimer::singleShot(100, this, SLOT(yourSlotFunction()));

    // tcpClient->write("5\n");
    // tcpClient->waitForBytesWritten();

    timer->stop();
    tcpClient->write("1\n");
    tcpClient->waitForBytesWritten();
    QString data = "some data";  // 假设这是你要发送的数据
    emit sendValueToROS(data);   // 触发信号，发送数据
    fileName++;
    timer->start();
    // timer->start(100);
}


/*
QVector<GLfloat> MainWindow::computeCubeVerticesOpenGL(const QVector3D& centerUnity, const QVector3D& sizeUnity, const QQuaternion& rotation)
{
    QVector<GLfloat> vertices;

    // Convert Unity center to OpenGL by swapping Y and Z axes
    QVector3D center(centerUnity.x(), centerUnity.z(), centerUnity.y());
    QVector3D size(sizeUnity.x(), sizeUnity.z(), sizeUnity.y());

    // Calculate half sizes for convenience
    QVector3D half_sizes = size * 0.5f;

    // Define the 8 vertices of the cube in local coordinates
    QVector3D vertexPositions[8] = {
        QVector3D(-half_sizes.x(), -half_sizes.y(), -half_sizes.z()),  // Vertex 0
        QVector3D(half_sizes.x(), -half_sizes.y(), -half_sizes.z()),   // Vertex 1
        QVector3D(half_sizes.x(), half_sizes.y(), -half_sizes.z()),    // Vertex 2
        QVector3D(-half_sizes.x(), half_sizes.y(), -half_sizes.z()),   // Vertex 3
        QVector3D(-half_sizes.x(), -half_sizes.y(), half_sizes.z()),   // Vertex 4
        QVector3D(half_sizes.x(), -half_sizes.y(), half_sizes.z()),    // Vertex 5
        QVector3D(half_sizes.x(), half_sizes.y(), half_sizes.z()),     // Vertex 6
        QVector3D(-half_sizes.x(), half_sizes.y(), half_sizes.z())     // Vertex 7
    };

    // Apply the rotation and translation using the transformation matrix
    QMatrix4x4 transformationMatrix;
    transformationMatrix.setToIdentity();
    transformationMatrix.translate(center);
    //QQuaternion alignRotation = QQuaternion::fromAxisAndAngle(QVector3D(0, 1, 0), 180.0f);

    //QQuaternion temp(90.0f, 0.0f, 0.0f, 0.0f);
    //QQuaternion finalRotation = rotation.x()+90.0f;

    //transformationMatrix.rotate(90.0f, 0.0f, 0.0f, 1.0f);
    //transformationMatrix.rotate(finalRotation);

    transformationMatrix.rotate(rotation.x()+90.0f, rotation.y(), rotation.scalar(),rotation.z() );
    qDebug()<<rotation.x()<<" "<<rotation.y()<<" "<<rotation.z()<<" "<<rotation.scalar();
    for (int i = 0; i < 8; ++i) {
        QVector3D transformedVertex = transformationMatrix * vertexPositions[i];
        vertices << transformedVertex.x() << transformedVertex.y() << transformedVertex.z();
    }

    return vertices;
}
*/


/*
QVector<GLfloat> MainWindow::computeCubeVerticesOpenGL(const QVector3D& centerUnity, const QVector3D& sizeUnity,const QQuaternion& rotation) {
    QVector<GLfloat> vertices;

    // Convert Unity center and size to OpenGL
    QVector3D center(centerUnity.x(), centerUnity.z(), centerUnity.y());
    QVector3D size(sizeUnity.x(), sizeUnity.z(), sizeUnity.y());

    // Calculate half sizes for convenience
    QVector3D half_sizes = size * 0.5f;

    // Calculate the 8 vertices of the cube
    vertices << center.x() - half_sizes.x() << center.y() - half_sizes.y() << center.z() - half_sizes.z();  // Vertex 0
    vertices << center.x() + half_sizes.x() << center.y() - half_sizes.y() << center.z() - half_sizes.z();  // Vertex 1
    vertices << center.x() + half_sizes.x() << center.y() + half_sizes.y() << center.z() - half_sizes.z();  // Vertex 2
    vertices << center.x() - half_sizes.x() << center.y() + half_sizes.y() << center.z() - half_sizes.z();  // Vertex 3
    vertices << center.x() - half_sizes.x() << center.y() - half_sizes.y() << center.z() + half_sizes.z();  // Vertex 4
    vertices << center.x() + half_sizes.x() << center.y() - half_sizes.y() << center.z() + half_sizes.z();  // Vertex 5
    vertices << center.x() + half_sizes.x() << center.y() + half_sizes.y() << center.z() + half_sizes.z();  // Vertex 6
    vertices << center.x() - half_sizes.x() << center.y() + half_sizes.y() << center.z() + half_sizes.z();  // Vertex 7

    QMatrix4x4 transformationMatrix;
    transformationMatrix.setToIdentity();
    transformationMatrix.rotate(rotation);

    for (int i = 0; i < 8; ++i) {
        QVector4D transformedVertex = transformationMatrix * QVector4D(vertexPositions[i].x(), vertexPositions[i].y(), vertexPositions[i].z(), 1.0f);
        vertices << transformedVertex.x() << transformedVertex.y() << transformedVertex.z();
    }

    return vertices;
}
*/
void MainWindow::on_load_yaml_pushButton_clicked()
{
    vehicleListModel = new QStringListModel(ui->vehicle_listView);

    QString filter = "YAML files (*.yaml);;All Files (*)";
    scenarioFileName = QFileDialog::getOpenFileName(this, "Open YAML File", QDir::homePath(), filter);

    if(scenarioFileName=="")
    {
        return;
    }

    scenarioYamlObject = YAML::LoadFile(scenarioFileName.toStdString());

    if (scenarioYamlObject["OpenSCENARIO"]["Entities"])
    {
        YAML::Node scenarioObjects = scenarioYamlObject["OpenSCENARIO"]["Entities"]["ScenarioObject"];
        if(scenarioObjects.IsSequence())
        {
            for (const auto& scenarioObject : scenarioObjects)
            {
                if (scenarioObject["name"])
                {
                    std::string name = scenarioObject["name"].as<std::string>();
                    //std::cout << "ScenarioObject name: " << name << std::endl;
                    vehicleList << QString::fromStdString(name);
                }
            }
        }
    }
    else
    {
        qDebug()<<"No Entities";
    }

    ui->vehicle_listView->setModel(vehicleListModel);
    vehicleListModel->setStringList(vehicleList);

    //需要维护一个二维数组用来发送修改以及保存
    //yaml中点位分为两部分第一部分是初始点位
    add_init_position();

    //第二部分是后续位置
    add_other_position();
}

void MainWindow::add_init_position()
{
    int j = 0;
    if (scenarioYamlObject["OpenSCENARIO"]["Storyboard"]["Init"])
    {
        YAML::Node privateActions = scenarioYamlObject["OpenSCENARIO"]["Storyboard"]["Init"]["Actions"]["Private"];
        if (privateActions.IsSequence())
        {
            for (const auto& action : privateActions)
            {
                YAML::Node teleportAction = action["PrivateAction"][0]["TeleportAction"];
                if (teleportAction)
                {
                    WayPoint wayPoint;
                    wayPoint.laneid = teleportAction["Position"]["LanePosition"]["laneId"].as<int>();
                    wayPoint.s = teleportAction["Position"]["LanePosition"]["s"].as<double>();
                    wayPoint.t = teleportAction["Position"]["LanePosition"]["offset"].as<double>();
                    wayPoints[j].push_back(wayPoint);
                    j++;
                }
            }
        }
    }

    //point[0] = new QStandardItem("point0");
    //point[0]->setData(QVariant::fromValue(wayPoints[vehicleListRow][0]), Qt::UserRole);
}

void MainWindow::add_other_position()
{
    int i = 1;
    YAML::Node acts = scenarioYamlObject["OpenSCENARIO"]["Storyboard"]["Story"][0]["Act"];
    for (const auto& act : acts)
    {
        YAML::Node maneuverGroups = act["ManeuverGroup"];
        for (const auto& maneuverGroup : maneuverGroups)
        {
            YAML::Node maneuvers = maneuverGroup["Maneuver"];
            for (const auto& maneuver : maneuvers)
            {
                YAML::Node events = maneuver["Event"];
                for (const auto& event : events)
                {
                    YAML::Node conditions = event["StartTrigger"]["ConditionGroup"][0]["Condition"];
                    for (const auto& condition : conditions)
                    {
                        // 检查是否存在ByEntityCondition部分
                        std::string entityName;
                        if (condition["ByEntityCondition"])
                        {
                            YAML::Node entityRef = condition["ByEntityCondition"]["TriggeringEntities"]["EntityRef"];
                            for (const auto& entity : entityRef)
                            {
                                entityName = entity["entityRef"].as<std::string>();
                                if(entityName != vehicleList[i].toStdString())
                                {
                                    i++;
                                    if(i>vehicleList.count())
                                    {
                                        break;
                                    }
                                }
                                if (entityName == vehicleList[i].toStdString())
                                {
                                    //
                                    WayPoint wayPoint;
                                    wayPoint.laneid = condition["ByEntityCondition"]["EntityCondition"]["ReachPositionCondition"]["Position"]["LanePosition"]["laneId"].as<int>();
                                    wayPoint.s = condition["ByEntityCondition"]["EntityCondition"]["ReachPositionCondition"]["Position"]["LanePosition"]["s"].as<double>();
                                    wayPoint.t = condition["ByEntityCondition"]["EntityCondition"]["ReachPositionCondition"]["Position"]["LanePosition"]["offset"].as<double>();
                                    //
                                    //
                                    wayPoints[i].push_back(wayPoint);
                                    //std::cout << "Entity: " << entityName << ", Offset value: " << offset << std::endl;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

void MainWindow::on_vehicle_listView_clicked(const QModelIndex &index)
{
    wayPointListModel->clear();

    vehicleListRow = index.row();
    qDebug()<< vehicleListRow;


    //std::vector<WayPoint> wayPoints[10];
    //先添加初始位置
    //add_init_position(wayPoints);

    //wayPointListModel->appendRow(point[0]);


    //再添加剩余位置
    //add_other_position(wayPoints);

    //wayPointList << QString::number(row);

    int i = 0;
    for(const auto& wayPoint : wayPoints[vehicleListRow])
    {
        point[i] = new QStandardItem(QString::fromStdString("point")+QString::number(i));
        point[i]->setData(QVariant::fromValue(wayPoint), Qt::UserRole);
        wayPointListModel->appendRow(point[i]);
        i++;
    }

    ui->vehicle_detail_listView->setModel(wayPointListModel);

}


void MainWindow::on_vehicle_detail_listView_clicked(const QModelIndex &index)
{
    wayPointListRow = index.row();
    WayPoint wayPoint;
    wayPoint = point[wayPointListRow]->data(Qt::UserRole).value<WayPoint>();

    ui->laneid_lineEdit->setText(QString::number(wayPoint.laneid));
    ui->s_lineEdit->setText(QString::number(wayPoint.s));
    ui->t_lineEdit->setText(QString::number(wayPoint.t));
    qDebug()<< wayPoint.laneid<<" "<<wayPoint.s<<" "<<wayPoint.t;
    //qDebug() << "WayPoint - laneId: " << wayPoints[vehicleListRow][wayPointListRow].laneid << ", s: " << wayPoints[vehicleListRow][wayPointListRow].s << ", offset: " << wayPoints[vehicleListRow][wayPointListRow].t;

    Eigen::Vector2d convertResult;
    lanelet::Id laneletID = wayPoint.laneid;
    convertResult = convert_lanelet_to_world(laneletID,wayPoint.s,wayPoint.t);
    ui->x_lineEdit->setText(QString::number(convertResult.x(),'f', 7));
    ui->y_lineEdit->setText(QString::number(convertResult.y(),'f', 7));

}



void MainWindow::on_pushButton_8_clicked()
{
    timer = new QTimer();
    connect(timer, &QTimer::timeout, this, [=]() {
        pointCloudWidget->clearCube();
        tcpClient->write("1\n");
        tcpClient->waitForBytesWritten();

        for(auto car : VehicleInformation)
        {
            //QQuaternion rotation(0.0f, 0.0f, 0.0f, 1.0f);
            //car.rotation=rotation;
            car.localPosition.setY(car.localPosition.y()+0.25);
            pointCloudWidget->addCube(car.localPosition, car.size, car.rotation,car.forward);
        }
    });
    timer->start(100);
}


void MainWindow::save_to_bin(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const std::string& filename)
{
    std::ofstream out(filename, std::ios::out | std::ios::binary);
    if (!out)
    {
        std::cerr << "Cannot open file for writing!" << std::endl;
        return;
    }

    for (const auto& point : cloud->points)
    {
        out.write((char*)&point.x, sizeof(float));
        out.write((char*)&point.y, sizeof(float));
        out.write((char*)&point.z, sizeof(float));
        out.write((char*)&point.intensity, sizeof(float));
    }
    out.close();
}

bool MainWindow::saveTextToFile(const QString &filePath, const QString &content)
{
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        // 无法打开文件进行写入
        return false;
    }

    QTextStream out(&file);
    out << content;
    file.close();
    return true;
}

void MainWindow::on_checkBox_stateChanged(int arg1)
{
    if(arg1 ==Qt::Checked)
    {
        qDebug()<<"1111";
        tcpClient->write("2\n");
        tcpClient->waitForBytesWritten();
    }
    else
    {
        qDebug()<<"2222";
        tcpClient->write("3\n");
        tcpClient->waitForBytesWritten();
    }
}


void MainWindow::on_pushButton_9_clicked(bool checked)
{
    /*
     * 4\n 暂停
     * 5\n 恢复
     *
     */
    //qDebug()<<"qqq";
    int ret = tcpClient->write("4\n");
    ret = tcpClient->waitForBytesWritten();
    //timer->stop();
}


void MainWindow::on_pushButton_10_clicked(bool checked)
{
    tcpClient->write("5\n");
    tcpClient->waitForBytesWritten();
    //timer->start(100);
}


void MainWindow::on_pushButton_11_clicked(bool checked)
{
    //tcpClient->write("1\n");
    //tcpClient->waitForBytesWritten();
    int i = 0;
    while(true)
    {
        auto start = std::chrono::high_resolution_clock::now();
        on_pushButton_clicked();
        auto end = std::chrono::high_resolution_clock::now();
        myMsleep(2000);
        std::chrono::duration<double, std::milli> duration = end - start;
        totalTime = totalTime+duration;
        qDebug()<<totalTime.count();
        i++;
        if(i==10)
        {
            break;
        }
    }


}

void MainWindow::myMsleep(int msec)
{
    QTime dieTime = QTime::currentTime().addMSecs(msec);
    while( QTime::currentTime() < dieTime )
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100); //非阻塞式

}


void MainWindow::on_pushButton_12_clicked(bool checked)
{
    //把情报发送给点云融合程序
    QVector<LidarInfo> updatedLidarInfos;
    for (int i = 0; i < checkBoxList.size(); ++i)
    {
        if (checkBoxList[i]->isChecked())
        {
            updatedLidarInfos.append(lidarinfos[i]);
        }
    }
    lidarinfos = updatedLidarInfos; // 更新原来的列表

    emit sendLidarValueToRos(lidarinfos);
}


void MainWindow::on_pushButton_13_clicked(bool checked)
{


    //test4 最佳方法，
    //发送脚本
    //on_sendScenarioButton_clicked();
    //等1秒钟等脚本跑起来
    //myMsleep(3000);

    int i =0;
    while(true)
    {
        auto start = std::chrono::high_resolution_clock::now();
        //暂停AWSIM
        tcpClient->write("4\n");
        tcpClient->waitForBytesWritten();
        //等1秒钟硬延时，等待AWSIM充分暂停
        QThread::msleep(1000);



        //保存文件
        emit savePointcloutToBin();
        emit saveInforToTxt();
        //发送获取车辆请求
        tcpClient->write("1\n");
        tcpClient->waitForBytesWritten();
        //等待车辆信息到达
        QThread::msleep(1000);

        qDebug()<<"2222";
        //QTimer::singleShot(100, this, SLOT(yourSlotFunction()));
        //恢复AWSIM
        tcpClient->write("5\n");
        tcpClient->waitForBytesWritten();
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = end - start;
        totalTime = totalTime+duration;
        i++;
        fileName++;

        if(i%10==0)
        {
            qDebug()<<totalTime.count();
        }

        if(i==100)
        {
            break;
        }
        myMsleep(2000);
    }
    qDebug()<<totalTime.count();

}


void MainWindow::on_pushButton_14_clicked(bool checked)
{
    tcpClient->write("6\n");
    tcpClient->waitForBytesWritten();
}

void MainWindow::handleDataFromRos(const QString& data)
{
    //qDebug()<<data;
    emit saveInforToTxt();
}
