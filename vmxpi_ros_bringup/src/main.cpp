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
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <sys/syscall.h>
#include <sys/types.h>

std::mutex command_mutex;  // Protect shared cmd_vel variables

static const double PI = 3.14159265;

// PID Controller Structure with Integral Clamping
struct PID {
    double kp, ki, kd;
    double prev_error;
    double integral;

    PID(double p, double i, double d) : kp(p), ki(i), kd(d), prev_error(0.0), integral(0.0) {}

    double compute(double setpoint, double measured, double dt) {
        double error = setpoint - measured;
        integral += error * dt;
        // Clamp the integral term to avoid windup
        const double integral_limit = 1000.0;
        if(integral > integral_limit) integral = integral_limit;
        if(integral < -integral_limit) integral = -integral_limit;
        
        double derivative = (error - prev_error) / dt;
        prev_error = error;
        return (kp * error) + (ki * integral) + (kd * derivative);
    }
};

// Encoder callbacks (assuming the sign is correct for your configuration)
void enc0Callback(const std_msgs::Int32::ConstPtr& msg) {
   // Left encoder
   // (No inversion here if not needed)
   // Adjust as necessary.
   // In this example, we assume the raw count is valid.
   // For simplicity, we use a global variable.
   // (In a real system you might encapsulate these in the class.)
   static_cast<void>(msg); // placeholder if not used
}
void enc1Callback(const std_msgs::Int32::ConstPtr& msg) {
   // Right encoder
   static_cast<void>(msg);
}
void enc2Callback(const std_msgs::Int32::ConstPtr& msg) {
   // Back encoder
   static_cast<void>(msg);
}

// In this example, we'll use global variables for encoder counts
// In a more encapsulated design, these could be members of Robot.
static int global_left_count = 0;
static int global_right_count = 0;
static int global_back_count = 0;

void leftEncCallback(const std_msgs::Int32::ConstPtr& msg) {
    global_left_count = msg->data;
}
void rightEncCallback(const std_msgs::Int32::ConstPtr& msg) {
    // Invert right encoder if needed
    global_right_count = msg->data; // adjust sign if necessary
}
void backEncCallback(const std_msgs::Int32::ConstPtr& msg) {
    global_back_count = msg->data;
}

// The Robot class encapsulates velocity control using PID
class Robot {
private:
    std::thread control_loop_thread;
    std::chrono::steady_clock::time_point last_cmd_time;
    std::chrono::steady_clock::time_point last_vel_time;

    // Latest cmd_vel values (feedforward targets)
    double cmd_linear_x = 0.0, cmd_linear_y = 0.0, cmd_angular_z = 0.0;
    const double COMMAND_TIMEOUT = 0.5; // seconds

    // Calculated target wheel speeds (in RPM) from kinematics
    double target_rpm_left = 0.0, target_rpm_right = 0.0, target_rpm_back = 0.0;
    // Measured wheel speeds (RPM) computed from encoder counts
    double meas_rpm_left = 0.0, meas_rpm_right = 0.0, meas_rpm_back = 0.0;
    // Final motor commands (normalized -1 to 1)
    double final_left = 0.0, final_right = 0.0, final_back = 0.0;

    // Variables to compute encoder differences
    int last_left_count = 0, last_right_count = 0, last_back_count = 0;
    // Encoder ticks per revolution
    const double TPR = 1464;

    // PID controllers for each wheel (tuned gains)
    PID pid_left  = PID(1.0, 1.5, 0.0);
    PID pid_right = PID(1.0, 1.5, 0.0);
    PID pid_back  = PID(1.0, 1.5, 0.0);

public:
    ros::ServiceClient set_m_speed;
    ros::Subscriber vel_sub;
    ros::Subscriber left_enc_sub, right_enc_sub, back_enc_sub;

    Robot(ros::NodeHandle* nh) {
        set_m_speed = nh->serviceClient<vmxpi_ros::MotorSpeed>("titan/set_motor_speed");

        // Subscribe to encoder count topics
        left_enc_sub = nh->subscribe("titan/encoder0/count", 1, leftEncCallback);
        right_enc_sub = nh->subscribe("titan/encoder1/count", 1, rightEncCallback);
        back_enc_sub = nh->subscribe("titan/encoder2/count", 1, backEncCallback);

        // Subscribe to cmd_vel topic
        vel_sub = nh->subscribe<geometry_msgs::Twist>("cmd_vel", 10, &Robot::cmdVelCallback, this);

        // Initialize timers
        last_cmd_time = std::chrono::steady_clock::now();
        last_vel_time = std::chrono::steady_clock::now();

        // Initialize encoder last counts
        last_left_count = global_left_count;
        last_right_count = global_right_count;
        last_back_count = global_back_count;

        // Start the control loop thread (running at 100 Hz)
        control_loop_thread = std::thread(&Robot::controlLoop, this);
    }

    // Callback to update cmd_vel (feedforward target)
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(command_mutex);
        last_cmd_time = std::chrono::steady_clock::now();
        cmd_linear_x = msg->linear.x;
        cmd_linear_y = msg->linear.y;
        cmd_angular_z = msg->angular.z;
    }

    // Kinematic conversion: Convert cmd_vel to target wheel speeds (RPM)
    // Adjust these equations to match your robot's geometry and kinematics.
    void computeKinematics(double x, double y, double z) {
        // Example: simple scaling (replace with your kinematics if needed)
        // Here, we compute motor speeds (linear) then convert to angular speeds.
        // You can modify these equations to your desired configuration.
        const double r = 0.051;  // wheel radius in meters
        // Using a simple holonomic drive model (adjust coefficients as needed)
        // For this example, we assume:
        //   leftSpeed, rightSpeed, backSpeed are linear speeds (m/s)
        double leftSpeed  = -0.33 * y + 0.58 * x - 0.33 * z;
        double rightSpeed = -0.33 * y - 0.58 * x - 0.33 * z;
        double backSpeed  = 0.67 * y - 0.33 * z;

        // Convert linear speeds (m/s) to angular wheel speeds (rad/s)
        double target_w_left  = leftSpeed / r;
        double target_w_right = rightSpeed / r;
        double target_w_back  = backSpeed / r;

        // Convert angular speeds to RPM
        target_rpm_left  = target_w_left  * 60.0 / (2.0 * PI);
        target_rpm_right = target_w_right * 60.0 / (2.0 * PI);
        target_rpm_back  = target_w_back  * 60.0 / (2.0 * PI);
    }

    // Publish the motor commands via the service client
    void publish_motors() {
        vmxpi_ros::MotorSpeed msg;
        // Left Motor (Motor 0)
        msg.request.speed = final_left;
        msg.request.motor = 0;
        if (!set_m_speed.call(msg)) {
            ROS_ERROR("Failed to set speed for Left Motor");
        }
        // Right Motor (Motor 1)
        msg.request.speed = final_right;
        msg.request.motor = 1;
        if (!set_m_speed.call(msg)) {
            ROS_ERROR("Failed to set speed for Right Motor");
        }
        // Back Motor (Motor 2)
        msg.request.speed = final_back;
        msg.request.motor = 2;
        if (!set_m_speed.call(msg)) {
            ROS_ERROR("Failed to set speed for Back Motor");
        }
    }

    // The control loop runs at 100 Hz
    void controlLoop() {
        ros::Rate rate(100);
        while (ros::ok()) {
            {
                std::lock_guard<std::mutex> lock(command_mutex);
                auto now = std::chrono::steady_clock::now();
                double dt = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_vel_time).count();
                last_vel_time = now; // update dt reference

                // Calculate encoder differences (for RPM computation)
                int current_left  = global_left_count;
                int current_right = global_right_count;
                int current_back  = global_back_count;
                
                int delta_left  = current_left  - last_left_count;
                int delta_right = current_right - last_right_count;
                int delta_back  = current_back  - last_back_count;
                
                last_left_count  = current_left;
                last_right_count = current_right;
                last_back_count  = current_back;
                
                // Compute measured RPM: RPM = (delta_ticks / TPR) / dt * 60
                meas_rpm_left  = (delta_left  / TPR) / dt * 60.0;
                meas_rpm_right = (delta_right / TPR) / dt * 60.0;
                meas_rpm_back  = (delta_back  / TPR) / dt * 60.0;

                // If no cmd_vel has been received for COMMAND_TIMEOUT, or all cmd_vel components are 0, stop motors.
                double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_cmd_time).count();
                bool zeroCommand = (std::abs(cmd_linear_x) < 1e-6 &&
                                    std::abs(cmd_linear_y) < 1e-6 &&
                                    std::abs(cmd_angular_z) < 1e-6) ||
                                   (elapsed > COMMAND_TIMEOUT);
                if (zeroCommand) {
                    target_rpm_left  = 0.0;
                    target_rpm_right = 0.0;
                    target_rpm_back  = 0.0;
                } else {
                    // Compute kinematics from cmd_vel into target RPM for each wheel
                    computeKinematics(cmd_linear_x, cmd_linear_y, cmd_angular_z);
                }

                // Compute PID corrections (output in RPM)
                double output_left  = pid_left.compute(target_rpm_left, meas_rpm_left, dt);
                double output_right = pid_right.compute(target_rpm_right, meas_rpm_right, dt);
                double output_back  = pid_back.compute(target_rpm_back, meas_rpm_back, dt);
                
                // Scale the PID output into a normalized motor command [-1, 1]
                // Adjust max_motor_rpm to your system’s expected maximum RPM.
                const double max_motor_rpm = 160.0;
                final_left  = output_left  / max_motor_rpm;
                final_right = output_right / max_motor_rpm;
                final_back  = output_back  / max_motor_rpm;

                // Clamp final commands to [-1, 1]
                final_left  = std::max(-1.0, std::min(final_left,  1.0));
                final_right = std::max(-1.0, std::min(final_right, 1.0));
                final_back  = std::max(-1.0, std::min(final_back,  1.0));

                // Optionally, print debug info:
                ROS_INFO_STREAM("Measured RPM: left " << meas_rpm_left 
                    << ", right " << meas_rpm_right 
                    << ", back " << meas_rpm_back);
                ROS_INFO_STREAM("PID Output (RPM): left " << output_left 
                    << ", right " << output_right 
                    << ", back " << output_back);
                ROS_INFO_STREAM("Final Command: left " << final_left 
                    << ", right " << final_right 
                    << ", back " << final_back);
                
                // Publish motor commands
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

    TitanDriverROSWrapper titan(&nh, &vmx);
    ROS_INFO("Titan driver is now started");

    Robot robot(&nh);

    ros::waitForShutdown();
    ROS_INFO("ROS SHUTDOWN");
    return 0;
}
