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
#include "geometry_msgs/Twist.h"

std::mutex command_mutex;  // Protects shared data

static double leftSpeed = 0.0, rightSpeed = 0.0, backSpeed = 0.0;
static double left_encoder = 0.0, right_encoder = 0.0, back_encoder = 0.0;
static double left_count = 0.0, right_count = 0.0, back_count = 0.0;
static double angle, angle_t;
static const double PI = 3.14159265;

struct PID {
    double kp, ki, kd;
    double prev_error;
    double integral;

    PID(double p, double i, double d) : kp(p), ki(i), kd(d), prev_error(0), integral(0) {}

    double compute(double setpoint, double measured, double dt) {
        double error = setpoint - measured;
        integral += error * dt;
        // Clamp the integral term to avoid windup
        const double integral_limit = 1000.0; // adjust this limit as needed
        if(integral > integral_limit) integral = integral_limit;
        if(integral < -integral_limit) integral = -integral_limit;
        
        double derivative = (error - prev_error) / dt;
        prev_error = error;
        return (kp * error) + (ki * integral) + (kd * derivative);
    }
};

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
    double target_w_left = 0, target_w_right = 0, target_w_back = 0;
    double target_rpm_left = 0, target_rpm_right = 0, target_rpm_back = 0;
    double meas_rpm_left = 0, meas_rpm_right = 0, meas_rpm_back = 0;
    double final_left = 0, final_right = 0, final_back = 0;
    int last_left_count = 0, last_right_count = 0, last_back_count = 0;
    const double alpha = 0.1; // Smoothing factor
    double TPR = 1464;
    std::chrono::steady_clock::time_point last_vel_time;
public:
    ros::ServiceClient set_m_speed, enable_client, disable_client;
    ros::ServiceClient resetAngle, res_encoder_client, stop_motors_client;

    ros::Subscriber motor0_dist, motor1_dist, motor2_dist, angle_sub, yawAngle_sub;
    ros::Subscriber enc0_sub, enc1_sub, enc2_sub;
    ros::Subscriber vel_sub;

    // PID controllers for each wheel (tuned gains are example values)
    PID pid_left  = PID(1.0, 2.0, 0.001);
    PID pid_right = PID(1.0, 2.0, 0.001);
    PID pid_back  = PID(1.0, 2.0, 0.001);

    Robot(ros::NodeHandle* nh) {
        set_m_speed = nh->serviceClient<vmxpi_ros::MotorSpeed>("titan/set_motor_speed");

        motor0_dist = nh->subscribe("titan/encoder0/distance", 1, motor0Callback);
        motor1_dist = nh->subscribe("titan/encoder1/distance", 1, motor1Callback);
        motor2_dist = nh->subscribe("titan/encoder2/distance", 1, motor2Callback);
        angle_sub = nh->subscribe("navx/angle", 1, angleCallback);
        yawAngle_sub = nh->subscribe("navx/yaw", 1, yawCallback);

        enc0_sub = nh->subscribe("titan/encoder0/count", 1, enc0Callback);
        enc1_sub = nh->subscribe("titan/encoder1/count", 1, enc1Callback);
        enc2_sub = nh->subscribe("titan/encoder2/count", 1, enc2Callback);

        enable_client = nh->serviceClient<std_srvs::Trigger>("titan/enable");
        disable_client = nh->serviceClient<std_srvs::Trigger>("titan/disable");

        resetAngle = nh->serviceClient<std_srvs::Empty>("reset_navx");
        res_encoder_client = nh->serviceClient<std_srvs::Trigger>("titan/reset_encoder");
        stop_motors_client = nh->serviceClient<std_srvs::Trigger>("titan/stop_motors");

        // Subscribe to cmd_vel topic
        vel_sub = nh->subscribe<geometry_msgs::Twist>("cmd_vel", 10, &Robot::cmdVelCallback, this);

        // Initialize last_cmd_time
        last_cmd_time = std::chrono::steady_clock::now();
        last_vel_time = std::chrono::steady_clock::now();

        // Start the control loop
        control_loop_thread = std::thread(&Robot::controlLoop, this);
    }

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(command_mutex);
        last_cmd_time = std::chrono::steady_clock::now();

        cmd_linear_x = msg->linear.x;
        cmd_linear_y = msg->linear.y;
        cmd_angular_z = msg->angular.z;
    }

    double applyDeadband(double rpm, double threshold = 1.0) {
        return (std::abs(rpm) < threshold) ? 0.0 : rpm;
    }

    void holonomicDrive(double x, double y, double z) {
        const double r = 0.051;

        rightSpeed = -0.33 * y - 0.58 * x - 0.33 * z;
        leftSpeed = -0.33 * y + 0.58 * x - 0.33 * z;
        backSpeed = 0.67 * y - 0.33 * z;

        // Convert to angular vel
        target_w_left = leftSpeed / r;
        target_w_right = rightSpeed / r;
        target_w_back = backSpeed / r;

        // Convert to rpm
        target_rpm_left = target_w_left * 60 / (2 * PI);
        target_rpm_right = target_w_right * 60 / (2 * PI);
        target_rpm_back = target_w_back * 60 / (2 * PI);

        // double max_speed = std::abs(rightSpeed);
        // if (std::abs(leftSpeed) > max_speed) {
        //     max_speed = std::abs(leftSpeed);
        // }
        // if (std::abs(backSpeed) > max_speed) {
        //     max_speed = std::abs(backSpeed);
        // }
        // if (max_speed > 1.0) {
        //     rightSpeed /= max_speed;
        //     leftSpeed /= max_speed;
        //     backSpeed /= max_speed;
        // }
    }

    void publish_motors() {
        vmxpi_ros::MotorSpeed msg1;

        // Left Motor (Motor 0)
        msg1.request.speed = final_left;
        msg1.request.motor = 0;
        if (!set_m_speed.call(msg1)) {
            ROS_ERROR("Failed to set speed for Left Motor");
        }

        // Right Motor (Motor 1)
        msg1.request.speed = final_right;
        msg1.request.motor = 1;
        if (!set_m_speed.call(msg1)) {
            ROS_ERROR("Failed to set speed for Right Motor");
        }

        // Back Motor (Motor 2)
        msg1.request.speed = final_back;
        msg1.request.motor = 2;
        if (!set_m_speed.call(msg1)) {
            ROS_ERROR("Failed to set speed for Back Motor");
        }
    }

    void controlLoop() {
        ros::Rate rate(100); // 10 Hz control loop
        while (ros::ok()) {
            {
                std::lock_guard<std::mutex> lock(command_mutex);
                auto now = std::chrono::steady_clock::now();
                // Compute dt since last velocity update
                double dt = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_vel_time).count();
                if (dt < 1e-4 || dt > 0.1) {  // adjust thresholds as appropriate
                    last_vel_time = now;
                    rate.sleep();
                    continue;
                }
                last_vel_time = now;
    
                // Calculate elapsed time since the last received velocity command
                double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_cmd_time).count();
                bool zeroCommand = (std::abs(cmd_linear_x) < 1e-6 &&
                                    std::abs(cmd_linear_y) < 1e-6 &&
                                    std::abs(cmd_angular_z) < 1e-6) ||
                                   (elapsed > COMMAND_TIMEOUT);
    
                if (zeroCommand) {
                    // Stop the robot
                    final_left = 0;
                    final_right = 0;
                    final_back = 0;

                    // Reset PID state for all wheels
                    pid_left.integral = 0.0;
                    pid_left.prev_error = 0.0;
                    pid_right.integral = 0.0;
                    pid_right.prev_error = 0.0;
                    pid_back.integral = 0.0;
                    pid_back.prev_error = 0.0;
                } else {
                    // Calculate the encoder differences since the last cycle
                    int current_left  = left_count;
                    int current_right = right_count;
                    int current_back  = back_count;
                    
                    int delta_left  = current_left  - last_left_count;
                    int delta_right = current_right - last_right_count;
                    int delta_back  = current_back  - last_back_count;
                    
                    last_left_count  = current_left;
                    last_right_count = current_right;
                    last_back_count  = current_back;
                    
                    // Calculate measured RPM for each wheel:
                    // RPM = (delta_ticks / TPR) / dt * 60.0
                    meas_rpm_left  = (delta_left  / TPR) / dt * 60.0;
                    meas_rpm_right = applyDeadband((delta_right / TPR) / dt * 60.0);
                    meas_rpm_back  = (delta_back  / TPR) / dt * 60.0;
    
                    // Compute target speeds using the latest cmd_vel command
                    holonomicDrive(cmd_linear_x, cmd_linear_y, cmd_angular_z);
    
                    // Compute PID outputs
                    double output_left  = pid_left.compute(target_rpm_left, meas_rpm_left, dt);
                    double output_right = pid_right.compute(target_rpm_right, meas_rpm_right, dt);
                    double output_back  = pid_back.compute(target_rpm_back, meas_rpm_back, dt);
                    
                    const double max_motor_rpm = 160.0;
                    final_left  = output_left  / max_motor_rpm;
                    final_right = output_right / max_motor_rpm;
                    final_back  = output_back  / max_motor_rpm;
    
                    // Clamp motor speeds to [-1.0, 1.0]
                    final_left  = std::max(-1.0, std::min(final_left,  1.0));
                    final_right = std::max(-1.0, std::min(final_right, 1.0));
                    final_back  = std::max(-1.0, std::min(final_back,  1.0));
                    
                    ROS_INFO("dt: %.2f", dt);
                    ROS_INFO_STREAM("Measured RPM: left " << meas_rpm_left 
                        << ", right " << meas_rpm_right 
                        << ", back " << meas_rpm_back);
                    ROS_INFO_STREAM("PID output: left " << output_left 
                        << ", right " << output_right 
                        << ", back " << output_back);
                    ROS_INFO_STREAM("Target RPM: left " << target_rpm_left 
                        << ", right " << target_rpm_right 
                        << ", back " << target_rpm_back);
                }
                // Publish motor commands every loop iteration
                publish_motors();
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

    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Initialize ROS wrappers
    // DigitalOutputROS d_o(&nh, &vmx, 10);
    // ROS_INFO("Digital Output is now started");

    // DigitalInputROS button1(&nh, &vmx, 11);
    // ROS_INFO("Digital Input is now started");

    TitanDriverROSWrapper titan(&nh, &vmx);
    ROS_INFO("Titan driver is now started");

    // navXROSWrapper navx(&nh, &vmx);
    // ROS_INFO("navX driver is now started");

    // Instantiate Robot class
    Robot robot(&nh);

    ros::waitForShutdown();

    ROS_INFO("ROS SHUTDOWN");
    return 0;
}
