#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int8MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

using namespace std;

const int WINDOW_SZ = 5;
const double LOOPTIME = 100;
double speed_req = 0;
double angular_speed_req = 0;
double speed_req_left = 0;
double speed_req_right = 0;
double l_rpm = 0;
double r_rpm = 0;
string port = "/dev/ttyUSB0";
double fakewheelDia = 0.1007;
double fakewheelBase = 0.240;
double fakeTrack = 0.280;
double wheelDia = 0.01007 / 2;
double wheelBase = 0.240;
double Track = 0.280;
double speed_act_upper_left = 0.0;
double speed_act_upper_right = 0.0;
double speed_act_lower_left = 0.0;
double speed_act_lower_right = 0.0;
double speed_act_left = 0.0;
double speed_act_right = 0.0;
double speed_dt = 0.0;
double two_pi = 6.28319;
double x_pos = 0.0;
double y_pos = 0.0;
double theta = 0.0;
double wheel_cir = (wheelDia * M_PI) / 60;
ros::Time current_time;
ros::Time speed_time(0.0);

// Structures from serialstm.h
struct Hostmessage {
    uint8_t HleftID = 0x01;
    int16_t Hleftspeed = 0;
    uint8_t HrightID = 0x02;
    int16_t Hrightspeed = 0;
    uint8_t LleftID = 0x03;
    int16_t Lleftspeed = 0;
    uint8_t LrightID = 0x04;
    int16_t Lrightspeed = 0;
};

struct recvMessage {
    uint8_t HleftID;
    int16_t Hleftspeed;
    uint8_t HrightID;
    int16_t Hrightspeed;
    uint8_t LleftID;
    int16_t Lleftspeed;
    uint8_t LrightID;
    int16_t Lrightspeed;
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t Q0;
    int16_t Q1;
    int16_t Q2;
    int16_t Q3;
    int16_t sensor1;
    int16_t sensor2;
    uint8_t d80nk1;
    uint8_t d80nk2;
    uint8_t d80nk3;
    uint8_t d80nk4;
};

// SerialSTM class
class SerialSTM {
public:
    serial::Serial ser;
    ros::NodeHandle n_ser;
    ros::Publisher front_pub;
    ros::Publisher back_pub;
    ros::Publisher imu_pub;
    ros::Publisher wsad_pub;
    ros::Publisher odom_pub;
    sensor_msgs::Range front_dist;
    sensor_msgs::Range back_dist;
    sensor_msgs::Imu imu_msgs;
    std_msgs::Int8MultiArray wsad;

    SerialSTM(string port, int baud) : port(port), baud(baud) {
        ser.setPort(port);
        ser.setBaudrate(baud);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        try {
            ser.open();
        } catch (const std::exception& e) {
            cout << "Unable to open port: " << e.what() << endl;
            throw;
        }
        front_pub = n_ser.advertise<sensor_msgs::Range>("front_dist", 1000);
        back_pub = n_ser.advertise<sensor_msgs::Range>("back_dist", 1000);
        imu_pub = n_ser.advertise<sensor_msgs::Imu>("imu", 1000);
        wsad_pub = n_ser.advertise<std_msgs::Int8MultiArray>("WSAD", sizeof(wsad));
        odom_pub = n_ser.advertise<nav_msgs::Odometry>("odom", 50);
    }

    uint8_t getcrc(uint8_t* Bytecode, int len) {
        uint8_t sum = 0;
        for (int i = 0; i < len; i++) {
            sum += Bytecode[i];
        }
        return sum;
    }

    int notopen(std::string &result) {
        if (!ser.isOpen()) {
            cout << "serial port is not opened" << endl;
            return 0;
        }
        result = ser.read(ser.available());
        return 0;
    }

    void readSpeed(recvMessage* recvmsg, uint8_t* bufferArray) {
        if (!ser.isOpen()) {
            cout << "serial port is not opened" << endl;
            return;
        }
        recvmsg->HleftID = bufferArray[1];
        recvmsg->Hleftspeed = (int16_t)((bufferArray[2] << 8) | (bufferArray[3] & 0xFF));
        recvmsg->HrightID = bufferArray[4] & 0xFF;
        recvmsg->Hrightspeed = (int16_t)((bufferArray[5] << 8) | (bufferArray[6] & 0xFF));
        recvmsg->LleftID = bufferArray[7];
        recvmsg->Lleftspeed = (int16_t)((bufferArray[8] << 8) | (bufferArray[9] & 0xFF));
        recvmsg->LrightID = bufferArray[10] & 0xFF;
        recvmsg->Lrightspeed = (int16_t)((bufferArray[11] << 8) | (bufferArray[12] & 0xFF));
        recvmsg->acc_x = ((bufferArray[13] << 8) & 0xFF) | (bufferArray[14] & 0xFF);
        recvmsg->acc_y = ((bufferArray[15] << 8) & 0xFF) | (bufferArray[16] & 0xFF);
        recvmsg->acc_z = ((bufferArray[17] << 8) & 0xFF) | (bufferArray[18] & 0xFF);
        recvmsg->gyro_x = ((bufferArray[19] << 8) & 0xFF) | (bufferArray[20] & 0xFF);
        recvmsg->gyro_y = ((bufferArray[21] << 8) & 0xFF) | (bufferArray[22] & 0xFF);
        recvmsg->gyro_z = ((bufferArray[23] << 8) & 0xFF) | (bufferArray[24] & 0xFF);
        recvmsg->Q0 = ((bufferArray[25] << 8) & 0xFF) | (bufferArray[26] & 0xFF);
        recvmsg->Q1 = ((bufferArray[27] << 8) & 0xFF) | (bufferArray[28] & 0xFF);
        recvmsg->Q2 = ((bufferArray[29] << 8) & 0xFF) | (bufferArray[30] & 0xFF);
        recvmsg->Q3 = ((bufferArray[31] << 8) & 0xFF) | (bufferArray[32] & 0xFF);
        recvmsg->sensor1 = ((bufferArray[33] << 8) & 0xFF) | (bufferArray[34] & 0xFF);
        recvmsg->sensor2 = ((bufferArray[35] << 8) & 0xFF) | (bufferArray[36] & 0xFF);
        recvmsg->d80nk1 = (bufferArray[37]);
        recvmsg->d80nk2 = (bufferArray[38]);
        recvmsg->d80nk3 = (bufferArray[39]);
        recvmsg->d80nk4 = (bufferArray[40]);
    }

    void IMUPublish(recvMessage* recvmsg) {
        imu_msgs.linear_acceleration.x = recvmsg->acc_x;
        imu_msgs.linear_acceleration.y = recvmsg->acc_y;
        imu_msgs.linear_acceleration.z = recvmsg->acc_z;
        imu_msgs.angular_velocity.x = recvmsg->gyro_x;
        imu_msgs.angular_velocity.y = recvmsg->gyro_y;
        imu_msgs.angular_velocity.z = recvmsg->gyro_z;
        imu_msgs.header.stamp = ros::Time::now();
        imu_pub.publish(imu_msgs);
    }

    void distancePublish(recvMessage* recvmsg) {
        front_dist.min_range = 20.0;
        front_dist.max_range = 720.0;
        front_dist.range = recvmsg->sensor1;
        front_dist.header.stamp = ros::Time::now();
        front_pub.publish(front_dist);

        back_dist.min_range = 20.0;
        back_dist.max_range = 720.0;
        back_dist.range = recvmsg->sensor2;
        back_dist.header.stamp = ros::Time::now();
        back_pub.publish(back_dist);
    }

    void bumpPublish(recvMessage* recvmsg) {
        wsad.data = {recvmsg->d80nk1 - 48, recvmsg->d80nk2 - 48, recvmsg->d80nk3 - 48, recvmsg->d80nk4 - 48};
        wsad_pub.publish(wsad);
    }

    void putSpeed(Hostmessage* hostmsg) {
        if (!ser.isOpen()) {
            cout << "serial port is not opened" << endl;
            return;
        }
        uint8_t sendByte[14];
        sendByte[0] = 0x00;
        sendByte[1] = hostmsg->HleftID;
        sendByte[2] = (hostmsg->Hleftspeed >> 8) & 0xFF;
        sendByte[3] = hostmsg->Hleftspeed & 0xFF;
        sendByte[4] = hostmsg->HrightID;
        sendByte[5] = (hostmsg->Hrightspeed >> 8) & 0xFF;
        sendByte[6] = hostmsg->Hrightspeed & 0xFF;
        sendByte[7] = hostmsg->LleftID;
        sendByte[8] = (hostmsg->Lleftspeed >> 8) & 0xFF;
        sendByte[9] = (hostmsg->Lleftspeed & 0xFF);
        sendByte[10] = hostmsg->LrightID;
        sendByte[11] = (hostmsg->Lrightspeed >> 8) & 0xFF;
        sendByte[12] = hostmsg->Lrightspeed & 0xFF;
        sendByte[13] = getcrc(sendByte, 13);
        ser.write(sendByte, sizeof(sendByte));
    }

private:
    string port;
    int baud;
};

// Command velocity callback
void cmd_handle(const geometry_msgs::Twist& cmd_vel) {
    speed_req = cmd_vel.linear.x;
    angular_speed_req = cmd_vel.angular.z;
    speed_req_left = speed_req - (angular_speed_req * (fakeTrack / 2));
    speed_req_right = speed_req + (angular_speed_req * (fakeTrack / 2));
    l_rpm = trunc((speed_req_left / (M_PI * fakewheelDia)) * 60);
    r_rpm = trunc((speed_req_right / (M_PI * fakewheelDia)) * 60);
}

// Odometry calculation and publishing
void handle_speed(const recvMessage& recv, tf::TransformBroadcaster& broadcaster, ros::Publisher& odom_pub, double loop_time) {
    speed_act_upper_left = recv.Lrightspeed; // Corresponds to TopLeftWheel
    speed_act_upper_right = recv.Lleftspeed; // Corresponds to TopRightWheel
    speed_act_lower_left = recv.Hrightspeed; // Corresponds to BottomLeftWheel
    speed_act_lower_right = recv.Hleftspeed; // Corresponds to BottomRightWheel
    speed_dt = loop_time / 1000; // Convert ms to s
    speed_time = ros::Time::now();
    speed_act_left = (speed_act_upper_left + speed_act_lower_left) * wheel_cir / 2;
    speed_act_right = -1 * (speed_act_upper_right + speed_act_lower_right) * wheel_cir / 2;
    ROS_INFO("speed_act_left: %f, speed_act_right: %f", speed_act_left, speed_act_right);
    // Odometry calculations
    double dt = speed_dt;
    double dxy = (speed_act_left + speed_act_right) * dt / 2;
    double dth = ((speed_act_right - speed_act_left) * dt) / Track;
    double linear_scale_positive = 1.0;
    double linear_scale_negative = 1.0;
    double angular_scale_positive = 1.0;
    double angular_scale_negative = 1.0;

    if (dth > 0) dth *= angular_scale_positive;
    if (dth < 0) dth *= angular_scale_negative;
    if (dxy > 0) dxy *= linear_scale_positive;
    if (dxy < 0) dxy *= linear_scale_negative;

    double dx = cos(dth) * dxy;
    double dy = sin(dth) * dxy;

    x_pos += (cos(theta) * dx - sin(theta) * dy);
    y_pos += (sin(theta) * dx + cos(theta) * dy);
    theta += dth;
    theta = atan2(sin(theta), cos(theta));

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta + M_PI);

    // Publish transform
    geometry_msgs::TransformStamped t;
    t.header.frame_id = "/odom";
    t.child_frame_id = "/base_link";
    t.transform.translation.x = x_pos;
    t.transform.translation.y = y_pos;
    t.transform.translation.z = 0.0;
    t.transform.rotation = odom_quat;
    t.header.stamp = speed_time;
    broadcaster.sendTransform(t);

    // Publish odometry
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = speed_time;
    odom_msg.header.frame_id = "/odom";
    odom_msg.pose.pose.position.x = x_pos;
    odom_msg.pose.pose.position.y = y_pos;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;
    if (speed_act_left == 0 && speed_act_right == 0) {
        odom_msg.pose.covariance[0] = 1e-9;
        odom_msg.pose.covariance[7] = 1e-3;
        odom_msg.pose.covariance[8] = 1e-9;
        odom_msg.pose.covariance[14] = 1e6;
        odom_msg.pose.covariance[21] = 1e6;
        odom_msg.pose.covariance[28] = 1e6;
        odom_msg.pose.covariance[35] = 1e-9;
        odom_msg.twist.covariance[0] = 1e-9;
        odom_msg.twist.covariance[7] = 1e-3;
        odom_msg.twist.covariance[8] = 1e-9;
        odom_msg.twist.covariance[14] = 1e6;
        odom_msg.twist.covariance[21] = 1e6;
        odom_msg.twist.covariance[28] = 1e6;
        odom_msg.twist.covariance[35] = 1e-9;
    } else {
        odom_msg.pose.covariance[0] = 1e-3;
        odom_msg.pose.covariance[7] = 1e-3;
        odom_msg.pose.covariance[8] = 0.0;
        odom_msg.pose.covariance[14] = 1e6;
        odom_msg.pose.covariance[21] = 1e6;
        odom_msg.pose.covariance[28] = 1e6;
        odom_msg.pose.covariance[35] = 1e3;
        odom_msg.twist.covariance[0] = 1e-3;
        odom_msg.twist.covariance[7] = 1e-3;
        odom_msg.twist.covariance[8] = 0.0;
        odom_msg.twist.covariance[14] = 1e6;
        odom_msg.twist.covariance[21] = 1e6;
        odom_msg.twist.covariance[28] = 1e6;
        odom_msg.twist.covariance[35] = 1e3;
    }
    double vx = (dt == 0) ? 0 : (speed_act_left + speed_act_right) / 2;
    double vth = (dt == 0) ? 0 : (speed_act_right - speed_act_left) / Track;
    odom_msg.child_frame_id = "/base_link";
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = dth;
    odom_pub.publish(odom_msg);
}

uint8_t checksum(uint8_t data[], int len) {
    int16_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc = (crc + data[i]) & 0xFF;
    }
    return crc;
}

void allTopicPublish(SerialSTM* pb, recvMessage* receive) {
    pb->IMUPublish(receive);
    pb->distancePublish(receive);
    pb->bumpPublish(receive);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_interface_node");
    ros::NodeHandle n;
    ros::NodeHandle nh_private_("~");
    recvMessage recv;
    Hostmessage hostmsg;

    // Load parameters
    n.getParam("ser_port", port);
    n.param<double>("fakewheelDia", fakewheelDia, 0.1007/2);
    n.param<double>("fakewheelBase", fakewheelBase, 0.240);
    n.param<double>("fakeTrack", fakeTrack, 0.280);
    n.param<double>("wheelDia", wheelDia, 0.01007 / 2);
    n.param<double>("wheelBase", wheelBase, 0.240);
    n.param<double>("Track", Track, 0.280);
    wheel_cir = (wheelDia * M_PI) / 60;

    // Initialize serial and publishers
    SerialSTM serial(port, 115200);
    tf::TransformBroadcaster broadcaster;
    ros::Subscriber cmd_sub = n.subscribe("cmd_vel", 1000, cmd_handle);

    std::string data, result;
    ros::Rate loop_rate(100);
    ros::Time last_publish_time = ros::Time::now();

    while (ros::ok()) {
        uint8_t bufferArray[42];
        serial.notopen(result);
        if (result.length() == 42) {
            for (int i = 0; i < 42; i++) {
                bufferArray[i] = result[i];
            }
        }

        if ((ros::Time::now() - last_publish_time).toSec() * 1000 >= LOOPTIME) {
            allTopicPublish(&serial, &recv);
            last_publish_time = ros::Time::now();
        }

        hostmsg.Hleftspeed = l_rpm;
        hostmsg.Lleftspeed = l_rpm;
        hostmsg.Hrightspeed = -r_rpm;
        hostmsg.Lrightspeed = -r_rpm;
        serial.putSpeed(&hostmsg);
        if (checksum(bufferArray, 42)) {
            serial.readSpeed(&recv, bufferArray);
            handle_speed(recv, broadcaster, serial.odom_pub, LOOPTIME);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}