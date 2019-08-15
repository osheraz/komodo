
#include "ricboard_pub.h"

RicboardPub::RicboardPub(ros::NodeHandle &nh)
{
    nh_ = &nh;

    /* get ric params */
    ros::param::get("~load_ric_hw", load_ric_hw_);

    if (load_ric_hw_)
    {
        if (!ros::param::get(RIC_PORT_PARAM, ric_port_))
        {
            ROS_ERROR("[komodo2_hw/ricboard_pub]: %s param is missing on param server. make sure that you load this param exist in ricboard_config.yaml "
                              "and that your launch includes this param file. shutting down...", RIC_PORT_PARAM);
            ros::shutdown();
            exit (EXIT_FAILURE);
        }

        try{
            ric_.connect(ric_port_);
            ROS_INFO("[komodo2_hw/ricboard_pub]: ricboard port opened successfully \nport name: %s \nbaudrate: 115200", ric_port_.c_str());
        }catch (ric::ConnectionExeption e) {
            ROS_ERROR("[komodo2_hw/ricboard_pub]: can't open ricboard port. make sure that ricboard is connected. shutting down...");
            ros::shutdown();
            exit(1);
        }

        /* ric publishers */
        ric_gps_pub_ = nh.advertise<sensor_msgs::NavSatFix>("GPS/fix", 10);
        rear_urf_pub_ = nh.advertise<sensor_msgs::Range>("URF/rear", 10);
        right_urf_pub_ = nh.advertise<sensor_msgs::Range>("URF/right", 10);
        left_urf_pub_ = nh.advertise<sensor_msgs::Range>("URF/left", 10);
        ric_imu_pub_ = nh.advertise<sensor_msgs::Imu>("IMU/data", 10);
        ric_mag_pub_ = nh.advertise<sensor_msgs::MagneticField>("IMU/magnetic", 10);

        ric_pub_timer_ = nh.createTimer(ros::Duration(RIC_PUB_INTERVAL), &RicboardPub::pubTimerCB, this);
        ric_dead_timer_ = nh.createTimer(ros::Duration(RIC_DEAD_TIMEOUT), &RicboardPub::ricDeadTimerCB, this);
        ROS_INFO("[komodo2_hw/ricboard_pub]: ricboard is up");
        espeak_pub_ = nh.advertise<std_msgs::String>("/espeak_node/speak_line", 10);
        /*speakMsg("rik board is up", 1); */
    }
    else
        ROS_WARN("[komodo2_hw/ricboard_pub]: ric hardware is disabled");
}

void RicboardPub::startLoop()
{
    if (!load_ric_hw_)
        return;
    t = new boost::thread(boost::bind(&RicboardPub::loop, this));
}

void RicboardPub::stopLoop()
{
    if (!load_ric_hw_)
        return;
    t->interrupt();
}

void RicboardPub::loop()
{
    if (!load_ric_hw_)
        return;
    while (ros::ok() && !t->interruption_requested())
    {
        ric_.loop();
        if (ric_.isBoardAlive())
        {
            ric::protocol::error err_msg;
            std::string logger_msg;
            int32_t logger_val;
            ric_disconnections_counter_ = 0;
            ric_dead_timer_.stop();
            ric_pub_timer_.start();

            if (ric_.readLoggerMsg(logger_msg, logger_val))
                ROS_INFO("[komodo2_hw/ricboard_pub]: ric logger is saying: '%s', value: %i", logger_msg.c_str(), logger_val);
            if (ric_.readErrorMsg(err_msg))
            {
                std::string comp_name = ric::RicInterface::compType2String((ric::protocol::Type)err_msg.comp_type);
                std::string err_desc = ric::RicInterface::errCode2String((ric::protocol::ErrCode)err_msg.code);
                if (err_msg.code != (uint8_t)ric::protocol::ErrCode::CALIB)
                {
                    ROS_ERROR("[komodo2_hw/ricboard_pub]: ric detected critical '%s' error in %s. shutting down...",
                              err_desc.c_str(), comp_name.c_str());
                    ros::shutdown();
                    exit(EXIT_FAILURE);
                }
                ROS_WARN("[komodo2_hw/ricboard_pub]: ric detected '%s' warning in %s",
                         err_desc.c_str(), comp_name.c_str());
            }
        }
        else
        {
            ric_dead_timer_.start();
            ric_pub_timer_.stop();
        }
    }
}

void RicboardPub::ricDeadTimerCB(const ros::TimerEvent &event)
{
    if (!load_ric_hw_)
        return;
    ric_disconnections_counter_++;
    if (ric_disconnections_counter_ >= MAX_RIC_DISCONNECTIONS)
    {
        speakMsg("rik board disconnected, shutting down", 1);
        ROS_ERROR("[komodo2_hw/ricboard_pub]: ricboard disconnected. shutting down...");
        ros::shutdown();
        exit(EXIT_FAILURE);
    }
}

void RicboardPub::pubTimerCB(const ros::TimerEvent &event)
{
    if (!load_ric_hw_ || !ric_.isBoardAlive())
        return;

    ric::sensors_state sensors = ric_.getSensorsState();

    /* publish ultrasonic */
    sensor_msgs::Range rear_range_msg;
    rear_range_msg.header.stamp = ros::Time::now();
    rear_range_msg.header.frame_id = "rear_urf_link";
    rear_range_msg.min_range = HSV_URF_MIN_RANGE;
    rear_range_msg.max_range = HSV_URF_MAX_RANGE;
    rear_range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    rear_range_msg.range = sensors.urf_rear.distance_mm / 1000.0;
    rear_range_msg.field_of_view = HSV_URF_FOV;
    rear_urf_pub_.publish(rear_range_msg);

    sensor_msgs::Range right_urf_msg;
    right_urf_msg.header.stamp = ros::Time::now();
    right_urf_msg.header.frame_id = "right_urf_link";
    right_urf_msg.min_range = HSV_URF_MIN_RANGE;
    right_urf_msg.max_range = HSV_URF_MAX_RANGE;
    right_urf_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    right_urf_msg.range = sensors.urf_right.distance_mm / 1000.0;
    right_urf_msg.field_of_view = HSV_URF_FOV;
    right_urf_pub_.publish(right_urf_msg);

    sensor_msgs::Range left_urf_msg;
    left_urf_msg.header.stamp = ros::Time::now();
    left_urf_msg.header.frame_id = "left_urf_link";
    left_urf_msg.min_range = HSV_URF_MIN_RANGE;
    left_urf_msg.max_range = HSV_URF_MAX_RANGE;
    left_urf_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    left_urf_msg.range = sensors.urf_left.distance_mm / 1000.0;
    left_urf_msg.field_of_view = HSV_URF_FOV;
    left_urf_pub_.publish(left_urf_msg);

    /* publish imu */
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "base_link";


    double roll, pitch, yaw;
    pitch = -sensors.imu.roll_rad;
    roll = -sensors.imu.pitch_rad;
    yaw = sensors.imu.yaw_rad - M_PI / 2;


    //wrap to PI
    if (yaw > M_PI )
        yaw -= 2 * M_PI;
    else if (yaw < -M_PI)
        yaw += 2 * M_PI;

    /*ROS_INFO("ROLL %f, PITCH %f, YAW %f", roll * 180 / M_PI,
             pitch * 180 / M_PI,
             yaw * 180 / M_PI);*/

    tf::Quaternion orientation_q = tf::createQuaternionFromRPY(roll,
                                                               pitch,
                                                               yaw);

    imu_msg.orientation.x = orientation_q.x();
    imu_msg.orientation.y = orientation_q.y();
    imu_msg.orientation.z = orientation_q.z();
    imu_msg.orientation.w = orientation_q.w();
    imu_msg.angular_velocity.x = -1 * sensors.imu.gyro_y_rad;
    imu_msg.angular_velocity.y = -1 * sensors.imu.gyro_x_rad;
    imu_msg.angular_velocity.z = -1 * sensors.imu.gyro_z_rad;
    imu_msg.linear_acceleration.x = sensors.imu.accl_y_rad * G_FORCE;
    imu_msg.linear_acceleration.y = sensors.imu.accl_x_rad * G_FORCE;
    imu_msg.linear_acceleration.z = sensors.imu.accl_z_rad * G_FORCE;
    ric_imu_pub_.publish(imu_msg);

    sensor_msgs::MagneticField mag_msg;
    mag_msg.header.stamp = ros::Time::now();
    mag_msg.header.frame_id = "base_link";
    mag_msg.magnetic_field.x = sensors.imu.mag_x_rad;
    mag_msg.magnetic_field.y = sensors.imu.mag_y_rad;
    mag_msg.magnetic_field.z = sensors.imu.mag_z_rad;
    ric_mag_pub_.publish(mag_msg);

    /* publish gps if data is available */
    if (sensors.gps.satellites > 0)
    {
        sensor_msgs::NavSatFix gps_msg;
        sensor_msgs::NavSatStatus gps_status;
        gps_msg.header.stamp = ros::Time::now();
        gps_msg.header.frame_id = "base_link";
        gps_status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
        gps_status.status = sensor_msgs::NavSatStatus::SERVICE_GPS;

        gps_msg.latitude = sensors.gps.lat;
        gps_msg.longitude = sensors.gps.lon;
        gps_msg.status = gps_status;
        gps_msg.header.stamp = ros::Time::now();

        ric_gps_pub_.publish(gps_msg);
    }
}























