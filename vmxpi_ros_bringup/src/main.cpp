#include "TitanDriver_ros_wrapper.h"
#include "navX_ros_wrapper.h"
#include "IOwd_ros.h"
#include "DI_ros.h"
#include "DO_ros.h"
#include <unistd.h>
#include <std_msgs/Float32.h>
#include <vmxpi_ros/Float.h>
#include <cmath>
#include <mutex>
#include <thread>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"

std::mutex command_mutex;  // Protects shared data

static double leftSpeed = 0.0, rightSpeed = 0.0, backSpeed = 0.0;
static double left_encoder = 0.0, right_encoder = 0.0, back_encoder = 0.0;
static double left_count = 0.0, right_count = 0.0, back_count = 0.0;
static double angle, angle_t;
static const double PI = 3.14159265;

// Callbacks for Encoder Distance values
void motor0Callback(const std_msgs::Float32::ConstPtr& msg) {
   left_encoder = msg->data;
}
void motor1Callback(const std_msgs::Float32::ConstPtr& msg) {
   right_encoder = msg->data;
}
void motor2Callback(const std_msgs::Float32::ConstPtr& msg) {
   back_encoder = msg->data;
}
void angleCallback(const std_msgs::Float32::ConstPtr& msg) {
    angle = std::abs(msg->data);
}
void yawCallback(const std_msgs::Float32::ConstPtr& msg) {
    angle_t = msg->data;
}

// Callbacks for Encoder count values
void enc0Callback(const std_msgs::Int32::ConstPtr& msg) {
   left_count = msg->data;
}
void enc1Callback(const std_msgs::Int32::ConstPtr& msg) {
   right_count = msg->data;
}
void enc2Callback(const std_msgs::Int32::ConstPtr& msg) {
   back_count = msg->data;
}

class Robot {
private: 
    std::thread control_loop_thread;
    std::chrono::steady_clock::time_point last_cmd_time;
    double cmd_linear_x = 0.0, cmd_linear_y = 0.0, cmd_angular_z = 0.0;
    const double COMMAND_TIMEOUT = 0.5; // 0.5 seconds
    double filtered_leftSpeed = 0.0, filtered_rightSpeed = 0.0, filtered_backSpeed = 0.0;
    const double alpha = 0.1; // Smoothing factor
public:
    ros::ServiceClient set_m_speed, enable_client, disable_client;
    ros::ServiceClient resetAngle, res_encoder_client, stop_motors_client;
    ros::Publisher dummyPublish;

    ros::Subscriber motor0_dist, motor1_dist, motor2_dist, angle_sub, yawAngle_sub;
    ros::Subscriber enc0_sub, enc1_sub, enc2_sub;
    ros::Subscriber vel_sub;

    Robot(ros::NodeHandle* nh) {
        set_m_speed = nh->serviceClient<vmxpi_ros::MotorSpeed>("titan/set_motor_speed");

        motor0_dist = nh->subscribe("titan/encoder0/distance", 1, motor0Callback);
        motor1_dist = nh->subscribe("titan/encoder1/distance", 1, motor1Callback);
        motor2_dist = nh->subscribe("titan/encoder2/distance", 1, motor2Callback);
        angle_sub = nh->subscribe("navx/angle", 1, angleCallback);
        yawAngle_sub = nh->subscribe("navx/yaw", 1, yawCallback);
        dummyPublish = nh->advertise<geometry_msgs::Twist>("dummy/cmd_vel", 1);

        enc0_sub = nh->subscribe("titan/encoder0/count", 1, enc0Callback);
        enc1_sub = nh->subscribe("titan/encoder1/count", 1, enc1Callback);
        enc2_sub = nh->subscribe("titan/encoder2/count", 1, enc2Callback);

        enable_client = nh->serviceClient<std_srvs::Trigger>("titan/enable");
        disable_client = nh->serviceClient<std_srvs::Trigger>("titan/disable");

        resetAngle = nh->serviceClient<std_srvs::Empty>("reset_navx");
        res_encoder_client = nh->serviceClient<std_srvs::Trigger>("titan/reset_encoder");
        stop_motors_client = nh->serviceClient<std_srvs::Trigger>("titan/stop_motors");

        // Subscribe to cmd_vel topic
        vel_sub = nh->subscribe<geometry_msgs::Vector3>("cmd_vel", 10, &Robot::cmdVelCallback, this);

        // Initialize last_cmd_time
        last_cmd_time = std::chrono::steady_clock::now();

        // Start the control loop
        control_loop_thread = std::thread(&Robot::controlLoop, this);
    }

    void cmdVelCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(command_mutex);
        last_cmd_time = std::chrono::steady_clock::now();

        cmd_linear_x = msg->x;
        cmd_linear_y = msg->y;
        cmd_angular_z = msg->z;
    }

    void holonomicDrive(double x, double y, double z) {
        rightSpeed = -0.33 * y - 0.58 * x - 0.33 * z;
        leftSpeed = -0.33 * y + 0.58 * x - 0.33 * z;
        backSpeed = 0.67 * y - 0.33 * z;

        double max_speed = std::abs(rightSpeed);
        if (std::abs(leftSpeed) > max_speed) {
            max_speed = std::abs(leftSpeed);
        }
        if (std::abs(backSpeed) > max_speed) {
            max_speed = std::abs(backSpeed);
        }
        if (max_speed > 1.0) {
            rightSpeed /= max_speed;
            leftSpeed /= max_speed;
            backSpeed /= max_speed;
        }
    }

    void publish_motors() {
        // Apply smoothing
        // filtered_leftSpeed = alpha * leftSpeed + (1 - alpha) * filtered_leftSpeed;
        // filtered_rightSpeed = alpha * rightSpeed + (1 - alpha) * filtered_rightSpeed;
        // filtered_backSpeed = alpha * backSpeed + (1 - alpha) * filtered_backSpeed;

        vmxpi_ros::MotorSpeed msg1;

        // Left Motor (Motor 0)
        msg1.request.speed = leftSpeed;
        msg1.request.motor = 0;
        if (!set_m_speed.call(msg1)) {
            ROS_ERROR("Failed to set speed for Left Motor");
        }

        // Right Motor (Motor 1)
        msg1.request.speed = rightSpeed;
        msg1.request.motor = 1;
        if (!set_m_speed.call(msg1)) {
            ROS_ERROR("Failed to set speed for Right Motor");
        }

        // Back Motor (Motor 2)
        msg1.request.speed = backSpeed;
        msg1.request.motor = 2;
        if (!set_m_speed.call(msg1)) {
            ROS_ERROR("Failed to set speed for Back Motor");
        }
    }

    void controlLoop() {
        ros::Rate rate(10); // 10 Hz control loop
        while (ros::ok()) {
            {
                std::lock_guard<std::mutex> lock(command_mutex);
                auto now = std::chrono::steady_clock::now();
                double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_cmd_time).count();

                if (elapsed > COMMAND_TIMEOUT) {
                    // No cmd_vel received for COMMAND_TIMEOUT seconds, stop the robot
                    cmd_linear_x = 0.0;
                    cmd_linear_y = 0.0;
                    cmd_angular_z = 0.0;
                }

                // Compute motor speeds using the latest cmd_vel
                holonomicDrive(cmd_linear_x, cmd_linear_y, cmd_angular_z);

                // Publish motor commands
                publish_motors();

                // Publish dummy twist message
                geometry_msgs::Twist msg;
                msg.linear.x = cmd_linear_x;
                msg.linear.y = cmd_linear_y;
                msg.angular.z = cmd_angular_z;
                dummyPublish.publish(msg);
            }
            rate.sleep();
        }
    }

    ~Robot() {
        if (control_loop_thread.joinable()) {
            control_loop_thread.join();
        }
    }

};

int main(int argc, char** argv) {
    system("/usr/local/frc/bin/frcKillRobot.sh");

    ROS_INFO_STREAM("Main thread: " << syscall(SYS_gettid));

    ros::init(argc, argv, "vmxpi_ros_wrapper");
    ros::NodeHandle nh;

    VMXPi vmx(true, (uint8_t)50);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    // Initialize ROS wrappers
    // DigitalOutputROS d_o(&nh, &vmx, 10);
    // ROS_INFO("Digital Output is now started");

    // DigitalInputROS button1(&nh, &vmx, 11);
    // ROS_INFO("Digital Input is now started");

    TitanDriverROSWrapper titan(&nh, &vmx);
    ROS_INFO("Titan driver is now started");

    navXROSWrapper navx(&nh, &vmx);
    ROS_INFO("navX driver is now started");

    // Instantiate Robot class
    Robot robot(&nh);

    ros::waitForShutdown();

    ROS_INFO("ROS SHUTDOWN");
    return 0;
}
