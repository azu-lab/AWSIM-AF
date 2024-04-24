#include "point_cloud_merge.h"

LidarSubscriber::LidarSubscriber():Node("lidar_subscriber")
{

    input_topics.resize(4);
    subscription.resize(input_topics.size());

    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos.durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE);

    output_frame_ = static_cast<std::string>(declare_parameter("output_frame", "world"));
    if (output_frame_.empty())
    {
        RCLCPP_ERROR(get_logger(), "Need an 'output_frame' parameter to be set before continuing!");
        return;
    }

    // Optionalc_transform_broadcaster.h:7 parameters
    maximum_queue_size_ = static_cast<int>(declare_parameter("max_queue_size", 5));
    timeout_sec_ = static_cast<double>(declare_parameter("timeout_sec", 0.1));

    input_offset_ = declare_parameter("input_offset", std::vector<double>{});
    if (!input_offset_.empty() && input_topics.size() != input_offset_.size())
    {
        RCLCPP_ERROR(get_logger(), "The number of topics does not match the number of offsets.");
        return;
    }

    //initialize offset map
    for (size_t i = 0; i < input_offset_.size(); ++i)
    {
        offset_map_[input_topics[i]] = input_offset_[i];
    }

    // tf2 listener
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);



    tf_publisher.resize(input_topics.size());

    transform_main.resize(input_topics.size());
    for(int i=0;i<input_topics.size();++i)
    {
        tf_publisher[i] = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        rclcpp::Time now;
        transform_main[i].header.stamp = now;
        transform_main[i].header.frame_id = "world";
        transform_main[i].child_frame_id = "world"+std::to_string(i+1);
    }

    tf2::Quaternion q1;
    q1.setRPY(180,180,0);
    //q1.setRPY(0,-M_PI/2,0);


    transform_main[0].transform.rotation.x = 180;
    transform_main[1].transform.rotation.x = 180;
    transform_main[2].transform.rotation.x = 180;
    transform_main[3].transform.rotation.x = 180;

    //transform_main[0].transform.translation.x = 0.0;
    //transform_main[0].transform.translation.y = 0.0;
    //transform_main[0].transform.translation.z = -8.65;

    //transform_main[1].transform.translation.x = -5.57;
    //transform_main[1].transform.translation.y = 30.89;
    //transform_main[1].transform.translation.z = -8.65;

    //transform_main[2].transform.translation.x = -39.21;
    //transform_main[2].transform.translation.y = 32.3;
    //transform_main[2].transform.translation.z = -8.65;

    //transform_main[3].transform.translation.x = -32.3;
    //transform_main[3].transform.translation.y = 10.4;
    //transform_main[3].transform.translation.z = -8.65;


    // //1  3  2  4  lidarinfor里的顺序
    // //lidar3
    // transform_main[0].transform.translation.x = 0.0;
    // transform_main[0].transform.translation.y = 0.0;

    // //lidar2
    // transform_main[1].transform.translation.x = -5.57;
    // transform_main[1].transform.translation.y = -30.89;

    // //lidar1
    // transform_main[2].transform.translation.x = -39.21;
    // transform_main[2].transform.translation.y = -32.3;

    // //lidar4
    // transform_main[3].transform.translation.x = -32.3;
    // transform_main[3].transform.translation.y = -10.4;

    //1  3  2  4  lidarinfor里的顺序
    //lidar3
    transform_main[0].transform.translation.x = 0.0;
    transform_main[0].transform.translation.y = 0.0;

    //lidar2
    transform_main[1].transform.translation.x = 0.0;
    transform_main[1].transform.translation.y = 0.0;

    //lidar1
    transform_main[2].transform.translation.x = 0.0;
    transform_main[2].transform.translation.y = 0.0;

    //lidar4
    transform_main[3].transform.translation.x = 0.0;
    transform_main[3].transform.translation.y = 0.0;


    for(size_t i=0;i<input_topics.size();++i)
    {
        //transform_main[i].transform.rotation.x = q1.x();
        //transform_main[i].transform.rotation.y = q1.y();
        //transform_main[i].transform.rotation.z = q1.z();
        //transform_main[i].transform.rotation.w = q1.w();
        tf_publisher[i]->sendTransform(transform_main[i]);
    }

    //publish
    pub_output_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output", rclcpp::SensorDataQoS().keep_last(maximum_queue_size_));

    for(size_t i=0;i<input_topics.size();++i)
    {
        //input topics names
        input_topics[i]=std::string("/lidar")+std::to_string(i+1)+std::string("/pointcloud_ex");
        std::cout<<input_topics[i]<<std::endl;

        //First input_topics_.size()
        cloud_stdmap_.insert(std::make_pair(input_topics[i], nullptr));
        cloud_stdmap_tmp_ = cloud_stdmap_;

        std::cout<<"222"<<std::endl;
        std::function<void(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)> cb = std::bind(&LidarSubscriber::cloud_callback, this,std::placeholders::_1, input_topics[i]);
        subscription[i].reset();
        //subscriber to topics
        subscription[i] = this->create_subscription<sensor_msgs::msg::PointCloud2>(input_topics[i], rclcpp::SensorDataQoS().keep_last(maximum_queue_size_), cb);
    }

    //auto twist_cb = std::bind(&LidarSubscriber::twist_callback, this, std::placeholders::_1);
    //sub_twist_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/vehicle/status/velocity_status", rclcpp::QoS{100}, twist_cb);

    //set timer
    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(timeout_sec_));
    timer_ = rclcpp::create_timer(this, get_clock(), period_ns,std::bind(&LidarSubscriber::timer_callback, this));
}

LidarSubscriber::LidarSubscriber(const rclcpp::NodeOptions & node_options):Node("lidar_subscriber", node_options)
{


}

void LidarSubscriber::cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input_ptr, const std::string & topic_name)
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto input = std::make_shared<sensor_msgs::msg::PointCloud2>(*input_ptr);

    const bool is_already_subscribed_this = (cloud_stdmap_[topic_name] != nullptr);
    const bool is_already_subscribed_tmp = std::any_of(std::begin(cloud_stdmap_tmp_), std::end(cloud_stdmap_tmp_),[](const auto & e) { return e.second != nullptr; });

    if (is_already_subscribed_this)
    {
        cloud_stdmap_tmp_[topic_name] = input;

        if (!is_already_subscribed_tmp)
        {
            auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(timeout_sec_));
            try
            {
                setPeriod(period.count());
            } catch (rclcpp::exceptions::RCLError & ex)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
            }
            timer_->reset();
        }

    }
    else
    {
        //std::cout<<"1"<<std::endl;
        cloud_stdmap_[topic_name] = input;

        const bool is_subscribed_all = std::all_of(std::begin(cloud_stdmap_), std::end(cloud_stdmap_),[](const auto & e) { return e.second != nullptr; });
        //std::cout<<"2"<<std::endl;
        if (is_subscribed_all)
        {
            for (const auto & e : cloud_stdmap_tmp_)
            {
                //std::cout<<"5"<<std::endl;
                if (e.second != nullptr)
                {
                    //std::cout<<"6"<<std::endl;
                    cloud_stdmap_[e.first] = e.second;
                }
            }
            //std::cout<<"3"<<std::endl;
            std::for_each(std::begin(cloud_stdmap_tmp_), std::end(cloud_stdmap_tmp_), [](auto & e) {e.second = nullptr;});
            //std::cout<<"4"<<std::endl;
            timer_->cancel();
            //std::cout<<"7"<<std::endl;
            publish();
        }
        else if (offset_map_.size() > 0)
        {
            timer_->cancel();
            auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(timeout_sec_ - offset_map_[topic_name]));
            try {
                setPeriod(period.count());
            } catch (rclcpp::exceptions::RCLError & ex) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
            }
            timer_->reset();
        }
    }

}

void LidarSubscriber::publish()
{
    bool all_clouds_available = std::all_of(cloud_stdmap_.begin(), cloud_stdmap_.end(),
                                            [](const auto& entry) { return entry.second != nullptr; });

    if (!all_clouds_available) {
        RCLCPP_WARN(this->get_logger(), "Not all point clouds are available.");
        return; // 可以选择清空 cloud_stdmap_ 或者其他适当的处理方式
    }

    //std::chrono::milliseconds timestamp_threshold(100);
    //timestamps_within_threshold(cloud_stdmap_, timestamp_threshold);
    //stop_watch_ptr_->toc("processing_time", true);
    sensor_msgs::msg::PointCloud2::SharedPtr concat_cloud_ptr_ = nullptr;
    //收到来自main的按钮点击标注事件后触发，这里相当于是将等待四个lidar的数据到达时间去除了
    if(startLabel==true)
    {
        startLabel=false;
        QString data = "some data123";
        emit sendValueToMain(data);
        fileName=fileName+1;
        isSaveBin=true;
    }


    //返回一个信息给main，告诉main可以获取标注信息了，

    not_subscribed_topic_names_.clear();
    for (const auto & e : cloud_stdmap_)
    {
        if (e.second != nullptr)
        {
            sensor_msgs::msg::PointCloud2::SharedPtr transformed_cloud_ptr(new sensor_msgs::msg::PointCloud2());
            transformPointCloud(e.second, transformed_cloud_ptr);
            if (concat_cloud_ptr_ == nullptr)
            {
                concat_cloud_ptr_ = transformed_cloud_ptr;
                //concat_cloud_ptr_ = std::make_shared<sensor_msgs::msg::PointCloud2>(*e.second);
            }
            else
            {
                LidarSubscriber::combineClouds(concat_cloud_ptr_, transformed_cloud_ptr, concat_cloud_ptr_);
                //LidarSubscriber::combineClouds(concat_cloud_ptr_, std::make_shared<sensor_msgs::msg::PointCloud2>(*e.second), concat_cloud_ptr_);
            }

        } else
        {
            not_subscribed_topic_names_.insert(e.first);
        }
    }
    if (concat_cloud_ptr_)
    {
        auto output = std::make_unique<sensor_msgs::msg::PointCloud2>(*concat_cloud_ptr_);
        //在发送之前将数据保存下来。
        if(isSaveBin==true)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(*output, *cloud);

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
        }


        pub_output_->publish(std::move(output));
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "concat_cloud_ptr_ is nullptr, skipping pointcloud publish.");
    }
    updater_.force_update();

    cloud_stdmap_ = cloud_stdmap_tmp_;
    std::for_each(std::begin(cloud_stdmap_tmp_), std::end(cloud_stdmap_tmp_), [](auto & e) {e.second = nullptr;});


}

void LidarSubscriber::save_to_bin(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const std::string& filename)
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

bool LidarSubscriber::timestamps_within_threshold(const std::map<std::string, std::shared_ptr<const sensor_msgs::msg::PointCloud2>>& cloud_map, const std::chrono::milliseconds& threshold) {
    auto it = cloud_map.begin();
    auto min_time = rclcpp::Time(it->second->header.stamp);
    auto max_time = min_time;

    for (const auto& entry : cloud_map) {
        if (entry.second) { // 这个检查现在是多余的，因为你已经检查过了
            const auto timestamp = rclcpp::Time(entry.second->header.stamp);
            if (timestamp < min_time) min_time = timestamp;
            if (timestamp > max_time) max_time = timestamp;
        }
    }
    qDebug()<<(max_time - min_time).seconds();
    return (max_time - min_time) <= rclcpp::Duration(threshold);
}


void LidarSubscriber::setPeriod(const int64_t new_period)
{
    if (!timer_)
    {
        return;
    }
    int64_t old_period = 0;
    rcl_ret_t ret = rcl_timer_get_period(timer_->get_timer_handle().get(), &old_period);
    if (ret != RCL_RET_OK)
    {
        rclcpp::exceptions::throw_from_rcl_error(ret, "Couldn't get old period");
    }
    ret = rcl_timer_exchange_period(timer_->get_timer_handle().get(), new_period, &old_period);
    if (ret != RCL_RET_OK)
    {
        rclcpp::exceptions::throw_from_rcl_error(ret, "Couldn't exchange_period");
    }
}

void LidarSubscriber::timer_callback()
{
    using std::chrono_literals::operator""ms;
    timer_->cancel();
    if (mutex_.try_lock()) {
        publish();
        mutex_.unlock();
    } else {
        try {
            std::chrono::nanoseconds period = 10ms;
            setPeriod(period.count());
        } catch (rclcpp::exceptions::RCLError & ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
        }
        timer_->reset();
    }
}



void LidarSubscriber::transformPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in, sensor_msgs::msg::PointCloud2::SharedPtr & out)
{
    // Transform the point clouds into the specified output frame
    if (output_frame_ != in->header.frame_id)
    {
        //out = std::make_shared<sensor_msgs::msg::PointCloud2>(*in);
        //out->header.frame_id = output_frame_;

        // TODO(YamatoAndo): use TF2
        if (!pcl_ros::transformPointCloud(output_frame_, *in, *out, *tf2_buffer_))
        {
            RCLCPP_ERROR(this->get_logger(),"[transformPointCloud] Error converting first input dataset from %s to %s.",in->header.frame_id.c_str(), output_frame_.c_str());
            return;
        }
    }
    else
    {
        out = std::make_shared<sensor_msgs::msg::PointCloud2>(*in);
    }
}


void LidarSubscriber::combineClouds(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in1, const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in2,sensor_msgs::msg::PointCloud2::SharedPtr & out)
{

    //std::cout<<"2224"<<std::endl;
    if (twist_ptr_queue_.empty())
    {
        pcl::concatenatePointCloud(*in1, *in2, *out);
        out->header.stamp = std::min(rclcpp::Time(in1->header.stamp), rclcpp::Time(in2->header.stamp));
        return;
    }

}


void LidarSubscriber::handleDataFromMain(const QString& data) {
    //RCLCPP_INFO(this->get_logger(), "Received data from Qt: %s", data.toStdString().c_str());
    //qDebug()<<data;
    // 进一步的处理逻辑
    startLabel=true;
}

void LidarSubscriber::handleLidarInfo(QList<LidarInfo> lidarinfos)
{
    qDebug()<<lidarinfos[0].name;
    qDebug()<<lidarinfos[0].position;
    qDebug()<<lidarinfos[0].rotation;
    li=lidarinfos;

    transform_main[1].transform.translation.x = -qAbs(li[2].position.z()-li[1].position.z());
    qDebug()<<transform_main[1].transform.translation.x;
    transform_main[1].transform.translation.y = -qAbs(li[2].position.x()-li[1].position.x());
    qDebug()<<transform_main[1].transform.translation.y;

    //transform_main[1].transform.translation.x = -5.57;
    //transform_main[1].transform.translation.y = -30.89;

    //lidar1
    transform_main[2].transform.translation.x = -qAbs(li[0].position.z()-li[1].position.z());
    transform_main[2].transform.translation.y = -qAbs(li[0].position.x()-li[1].position.x());
    //transform_main[2].transform.translation.x = -39.21;
    //transform_main[2].transform.translation.y = -32.3;

    //lidar4
    transform_main[3].transform.translation.x = -qAbs(li[3].position.z()-li[1].position.z());
    transform_main[3].transform.translation.y = -qAbs(li[3].position.x()-li[1].position.x());

    for(size_t i=0;i<input_topics.size();++i)
    {
        //transform_main[i].transform.rotation.x = q1.x();
        //transform_main[i].transform.rotation.y = q1.y();
        //transform_main[i].transform.rotation.z = q1.z();
        //transform_main[i].transform.rotation.w = q1.w();
        tf_publisher[i]->sendTransform(transform_main[i]);
    }
}
