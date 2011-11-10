#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>

#include "elektron.hpp"

ros::Time cmd_time;

Protonek *p;

void twistCallback(const geometry_msgs::TwistConstPtr& msg) {
        double rotational_term = msg->angular.z;
        double rvel = -msg->linear.x + rotational_term;
        double lvel = -msg->linear.x - rotational_term;

        if (rvel > 1.0)
                rvel = 1.0;
        if (rvel < -1.0)
                rvel = -1.0;

        if (lvel > 1.0)
                lvel = 1.0;
        if (lvel < -1.0)
                lvel = -1.0;

        // turbo button
        if (msg->angular.x < 0.5)
        {
                rvel /= 2.0;
                lvel /= 2.0;
        }

        // stop button
        if (msg->angular.y > 0.5)
                p->stopMotors();
        else
                p->runMotors();

        // feature 1 button
        if (msg->linear.y > 0.5)
                p->trick1();

        // feature 2 button
        if (msg->linear.z > 0.5)
                p->trick2();

        p->setVelocity(lvel, rvel);
}

int main(int argc, char** argv) {
        ros::init(argc, argv, "elektron_base_node");
        ros::NodeHandle n;
        ros::NodeHandle nh("~");

        ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry> ("odom", 1);
        ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu> ("imu", 1);
        ros::Publisher currentL_pub = n.advertise<geometry_msgs::PointStamped> ("currentL", 1);
        ros::Publisher currentR_pub = n.advertise<geometry_msgs::PointStamped> ("currentR", 1);
        ros::Publisher speedL_pub = n.advertise<geometry_msgs::PointStamped> ("speedL", 1);
        ros::Publisher speedR_pub = n.advertise<geometry_msgs::PointStamped> ("speedR", 1);

        ros::Subscriber twist_sub = n.subscribe("cmd_vel", 1, &twistCallback);

        ros::Rate loop_rate(100);

        std::string dev;
        
        nh.param<std::string>("device", dev, "/dev/ttyUSB0");

        Protonek::Parameters parL, parR;
        // lewy mostek
        nh.param<int>("parL_currentKp", parL.currentKp,0);
        nh.param<int>("parL_currentKi", parL.currentKi,2);
        nh.param<int>("parL_currentKd", parL.currentKd,0);
        nh.param<int>("parL_maxCurrent", parL.maxCurrent,50);
        nh.param<int>("parL_speedKp", parL.speedKp,8);
        nh.param<int>("parL_speedKi", parL.speedKi,0);
        nh.param<int>("parL_speedKd", parL.speedKd,0);

        // prawy mostek
        nh.param<int>("parR_currentKp", parR.currentKp,0);
        nh.param<int>("parR_currentKi", parR.currentKi,2);
        nh.param<int>("parR_currentKd", parR.currentKd,0);
        nh.param<int>("parR_maxCurrent", parR.maxCurrent,50);
        nh.param<int>("parR_speedKp", parR.speedKp,8);
        nh.param<int>("parR_speedKi", parR.speedKi,0);
        nh.param<int>("parR_speedKd", parR.speedKd,0);

        int useSpeedRegulator;
        nh.param<int>("use_speed_regulator",useSpeedRegulator, 1);

        double balanceAngle;
        nh.param<double>("balance_angle",balanceAngle, 90);
        
        nav_msgs::Odometry odom;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        sensor_msgs::Imu imu;
        imu.header.frame_id = "imu";
        
        geometry_msgs::PointStamped currentL, currentR, speedL, speedR;
        currentL.header.frame_id = "currentL";
        currentR.header.frame_id = "currentR";
        speedL.header.frame_id = "speedL";
        speedR.header.frame_id = "speedR";

        // initialize hardware
        p = new Protonek(dev, parL, parR);

        if (useSpeedRegulator)
                p->enableSpeedRegulator();
        else
                p->disableSpeedRegulator();

        p->balanceAngle = balanceAngle;
        
        double angular_pos = 0;
        if (p->isConnected()) {

                while (ros::ok()) {
                        double x, y, th, xvel, thvel;
                        double accX, accY, accZ, omegaY, pitch, pitch2;

                        ros::Time current_time = ros::Time::now();

                        p->update();
                        p->updateOdometry();
                        p->getOdometry(x, y, th);
                        p->getVelocity(xvel, thvel);
                        p->getImu(accX, accY, accZ, omegaY, pitch, pitch2);

                        //since all odometry is 6DOF we'll need a quaternion created from yaw
                        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

                        //next, we'll publish the odometry message over ROS
                        odom.header.stamp = current_time;

                        //set the position
                        odom.pose.pose.position.x = x;
                        odom.pose.pose.position.y = y;
                        odom.pose.pose.position.z = 0.0;
                        odom.pose.pose.orientation = odom_quat;

                        odom.pose.covariance[0] = 0.00001;
                        odom.pose.covariance[7] = 0.00001;
                        odom.pose.covariance[14] = 10.0;
                        odom.pose.covariance[21] = 1.0;
                        odom.pose.covariance[28] = 1.0;
                        odom.pose.covariance[35] = thvel + 0.001;

                        //set the velocity
                        odom.child_frame_id = "base_link";
                        odom.twist.twist.linear.x = xvel;
                        odom.twist.twist.linear.y = 0.0;
                        odom.twist.twist.angular.z = thvel;

                        //publish the message
                        odom_pub.publish(odom);

                        // imu
                        imu.header.stamp = current_time;

                        imu.orientation.x = pitch;
                        imu.orientation.y = pitch2;
                        imu.orientation_covariance[0] = 0.01;

//angular_pos += (omegaY+3)*0.1;

                        imu.angular_velocity.y = omegaY;          // odczyt z gyro
                        imu.angular_velocity.z = 0;          // odczyt z gyro
                        imu.angular_velocity_covariance[0] = 0.01;
                        imu.angular_velocity_covariance[3] = 0.01;
                        imu.angular_velocity_covariance[6] = 0.01;
                        
                        imu.linear_acceleration.x = accX;       // odczyt z akcelerometru
                        imu.linear_acceleration.y = accY;
                        imu.linear_acceleration.z = accZ;
                        imu.linear_acceleration_covariance[0] = 0.01;
                        imu.linear_acceleration_covariance[3] = 0.01;
                        imu.linear_acceleration_covariance[6] = 0.01;
                        
                        imu_pub.publish(imu);

                        // prad i predkosc
                        currentL.point.x = p->bridgeL.currentMeasured;
                        currentL.point.y = p->bridgeL.currentGiven;
                        currentL.header.stamp = current_time;
                        currentL_pub.publish(currentL);

                        speedL.point.x = p->bridgeL.speedMeasured;
                        speedL.point.y = p->bridgeL.speedGiven;
                        speedL.header.stamp = current_time;
                        speedL_pub.publish(speedL);
                        
                        currentR.point.x = p->bridgeR.currentMeasured;
                        currentR.point.y = p->bridgeR.currentGiven;
                        currentR.header.stamp = current_time;
                        currentR_pub.publish(currentR);

                        speedR.point.x = p->bridgeR.speedMeasured;
                        speedR.point.y = p->bridgeR.speedGiven;
                        speedR.header.stamp = current_time;
                        speedR_pub.publish(speedR);
                        
                        ros::spinOnce();
                        loop_rate.sleep();
                }
        } else {
                ROS_ERROR("Connection to device %s failed", dev.c_str());
        }

        return 0;
}
