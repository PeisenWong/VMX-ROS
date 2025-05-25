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
#include <fstream>

std::mutex command_mutex;  // Protects shared data

static double leftSpeed = 0.0, rightSpeed = 0.0, backSpeed = 0.0;
static float left_encoder = 0.0, right_encoder = 0.0, back_encoder = 0.0;
static double left_count = 0.0, right_count = 0.0, back_count = 0.0;
static double angle, angle_t;
static const double PI = 3.14159265;
bool seen_left = false, seen_right = false, seen_back = false;
int last_left_count = 0, last_right_count = 0, last_back_count = 0;

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

struct ABT_t {
    double alpha, beta, gamma;
    double *input;
    double *pos_output;
    double *vel_output;
    double *acc_output;
};

void ABTInit(double a, double b, double g,
    double *ip, double *pos, double *vel, double *acc, ABT_t *filt)
{
    filt->alpha       = a;
    filt->beta        = b;
    filt->gamma       = g;
    filt->input       = ip;
    filt->pos_output  = pos;
    filt->vel_output  = vel;
    filt->acc_output  = acc;
}

void ABTEstimateInit(ABT_t *filt)
{
    // *(filt->input) = 0.0;
    *(filt->pos_output) = 0.0;
    *(filt->vel_output) = 0.0;
    *(filt->acc_output) = 0.0;
}

void ABTStop(ABT_t *filt)
{
    // *(filt->input) = 0.0;
    *(filt->pos_output) = *(filt->input);
    *(filt->vel_output) = 0.0;
    *(filt->acc_output) = 0.0;
}

void ABT(ABT_t *filt, double dt)
{
    double pos_predict = *(filt->pos_output)
                      + dt  * (*(filt->vel_output))
                      + 0.5f * dt * dt * (*(filt->acc_output));
    double vel_predict = *(filt->vel_output)
                      + dt * (*(filt->acc_output));

    double residual = *filt->input - pos_predict;

    *(filt->pos_output) = pos_predict + filt->alpha * residual;
    *(filt->vel_output) = vel_predict + (filt->beta / dt) * residual;
    *(filt->acc_output) = *(filt->acc_output)
                        + (filt->gamma * 0.5f
                           / (dt * dt)) * residual;
}

// Callbacks for Encoder count values
void enc0Callback(const std_msgs::Int32::ConstPtr& msg) {
   left_count = msg->data;
//    if(!seen_left)
//    {
//     last_left_count = left_count;
//     seen_left = true;
//    }
}
void enc1Callback(const std_msgs::Int32::ConstPtr& msg) {
   right_count = msg->data;
//    if(!seen_right)
//    {
//     last_right_count = right_count;
//     seen_right = true;
//    }
}
void enc2Callback(const std_msgs::Int32::ConstPtr& msg) {
   back_count = msg->data;
//    if(!seen_back)
//    {
//     last_back_count = back_count;
//     seen_back = true;
//    }
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
    const double alpha = 0.1; // Smoothing factor
    double TPR = 1464;
    double cumDistLeft = 0, cumDistRight = 0, cumDistBack = 0;
    double r = 0.051;
    int first = 0;
    std::chrono::steady_clock::time_point last_vel_time;
    std::ofstream rpm_log;
    std::chrono::steady_clock::time_point start_time;
public:
    ros::ServiceClient set_m_speed, enable_client, disable_client;
    ros::ServiceClient resetAngle, res_encoder_client, stop_motors_client;

    ros::Subscriber enc0_sub, enc1_sub, enc2_sub;
    ros::Subscriber vel_sub;

    // PID controllers for each wheel (tuned gains loaded from ROS parameters)
    PID pid_left;
    PID pid_right;
    PID pid_back;

    ABT_t fleft_pos_data, fright_pos_data, fback_pos_data;
    double rawDistLeft, rawDistRight, rawDistBack;
    double fFLeftPos, fFLeftVel, fFLeftAcc;
    double fFRightPos, fFRightVel, fFRightAcc;
    double fFBackPos, fFBackVel, fFBackAcc;

    FILE* rpm_log_fp;

    Robot(ros::NodeHandle* nh)
      : pid_left(1.5, 1.0, 0.001),  // temporary default values; will be updated from params
        pid_right(1.5, 1.0, 0.001),
        pid_back(1.5, 1.0, 0.001)
    {
        start_time = std::chrono::steady_clock::now();
        rpm_log.open("/home/pi/rpm_log.csv", std::ios::out | std::ios::trunc);
        if (!rpm_log.is_open()) {
            ROS_FATAL("Could not open /home/pi/rpm_log.csv: %s", strerror(errno));
        } else {
        rpm_log << "timestamp,left_target,left_measured,"
                << "right_target,right_measured,"
                << "back_target,back_measured\n";
        rpm_log.flush();
        }

        // Get PID parameters from ROS parameters
        double p_left, i_left, d_left;
        double p_right, i_right, d_right;
        double p_back, i_back, d_back;
        double a, b, g;
        
        nh->param("pid/left/p", p_left, 1.0);
        nh->param("pid/left/i", i_left, 1.0);
        nh->param("pid/left/d", d_left, 0.001);
        
        nh->param("pid/right/p", p_right, 1.0);
        nh->param("pid/right/i", i_right, 1.0);
        nh->param("pid/right/d", d_right, 0.001);
        
        nh->param("pid/back/p", p_back, 1.0);
        nh->param("pid/back/i", i_back, 1.0);
        nh->param("pid/back/d", d_back, 0.001);

        nh->param("a", a, 0.8);
        nh->param("b", b, 0.4);
        nh->param("g", g, 0.2);
        
        // Re-initialize our PID objects with the fetched parameters
        pid_left = PID(p_left, i_left, d_left);
        pid_right = PID(p_right, i_right, d_right);
        pid_back = PID(p_back, i_back, d_back);

        ABTInit(a, b, g,
            &rawDistLeft, &fFLeftPos, &fFLeftVel, &fFLeftAcc,
            &fleft_pos_data);
        ABTEstimateInit(&fleft_pos_data);

        ABTInit(a, b, g,
                &rawDistRight, &fFRightPos, &fFRightVel, &fFRightAcc,
                &fright_pos_data);
        ABTEstimateInit(&fright_pos_data);

        ABTInit(a, b, g,
                &rawDistBack, &fFBackPos, &fFBackVel, &fFBackAcc,
                &fback_pos_data);
        ABTEstimateInit(&fback_pos_data);

        set_m_speed = nh->serviceClient<vmxpi_ros::MotorSpeed>("titan/set_motor_speed");

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

        // Initialize timers
        last_cmd_time = std::chrono::steady_clock::now();
        last_vel_time = std::chrono::steady_clock::now();

        // Right after starting your subscriptions and before starting the control loop:
        last_left_count  = left_count;
        last_right_count = right_count;
        last_back_count  = back_count;

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

        rightSpeed = -0.33 * y - 0.58 * x - 0.33 * z;
        leftSpeed = -0.33 * y + 0.58 * x - 0.33 * z;
        backSpeed = 0.67 * y - 0.33 * z;

        // Convert to angular velocity and then to RPM
        target_w_left = leftSpeed / r;
        target_w_right = rightSpeed / r;
        target_w_back = backSpeed / r;

        target_rpm_left = target_w_left * 60 / (2 * PI);
        target_rpm_right = target_w_right * 60 / (2 * PI);
        target_rpm_back = target_w_back * 60 / (2 * PI);
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
        ros::Rate rate(20); 
        while (ros::ok()) {
            {
                auto now = std::chrono::steady_clock::now();
                // Compute dt since last velocity update
                double dt = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_vel_time).count();
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

                    ABTStop(&fleft_pos_data);
                    ABTStop(&fright_pos_data);
                    ABTStop(&fback_pos_data);
                } else {
                    // Calculate the encoder differences since the last cycle
                    // int current_left  = left_count;
                    // int current_right = right_count;
                    // int current_back  = back_count;
                    
                    // int delta_left  = current_left  - last_left_count;
                    // int delta_right = current_right - last_right_count;
                    // int delta_back  = current_back  - last_back_count;
                    
                    // last_left_count  = current_left;
                    // last_right_count = current_right;
                    // last_back_count  = current_back;
                    
                    // // Calculate measured RPM for each wheel:
                    // // RPM = (delta_ticks / TPR) / dt * 60.0
                    // meas_rpm_left  = (delta_left  / TPR) / dt * 60.0;
                    // meas_rpm_right = applyDeadband((delta_right / TPR) / dt * 60.0);
                    // meas_rpm_back  = (delta_back  / TPR) / dt * 60.0;

                    // 1) delta counts
                    int currL = left_count, currR = right_count, currB = back_count;
                    int dL, dR, dB;
                    
                    dL = currL - last_left_count;
                    dR = currR - last_right_count;
                    dB = currB - last_back_count;
                    

                    // 2) convert to **distance** (meters)
                    // L = 2pi*r
                    // 1 revolution : 1464
                    // x revolution : delta_ticks
                    // x (rev) = delta_ticks / 1464
                    // x(m) = delta_ticks / 1464 * L
                    const double wheelCirc = 2.0 * M_PI * r;
                    cumDistLeft  += double(dL) / TPR * wheelCirc;
                    cumDistRight += double(dR) / TPR * wheelCirc;
                    cumDistBack  += double(dB) / TPR * wheelCirc;

                    rawDistLeft   = cumDistLeft;
                    rawDistRight  = cumDistRight;
                    rawDistBack   = cumDistBack;

                    ABT(&fleft_pos_data, 0.05);
                    ABT(&fright_pos_data, 0.05);
                    ABT(&fback_pos_data, 0.05);

                    // v = rw
                    // w = RPM / 60 * 2pi
                    // RPM = w * 60 / (2pi)
                    // RPM = (v / r) * 60 / (2pi)
                    meas_rpm_left  = (fFLeftVel  / r) * 60.0 / (2 * M_PI);
                    meas_rpm_right = (fFRightVel / r) * 60.0 / (2 * M_PI);
                    meas_rpm_back  = (fFBackVel  / r) * 60.0 / (2 * M_PI);

                    double vx, vy, vz;
                    {
                      std::lock_guard<std::mutex> lock(command_mutex);
                      vx = cmd_linear_x;
                      vy = cmd_linear_y;
                      vz = cmd_angular_z;
                    }
    
                    // Compute target speeds using the latest cmd_vel command
                    holonomicDrive(vx, vy, vz);
    
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
                    
                    ROS_INFO("----- PID DEBUG -----");
                    ROS_INFO("Wheel       | Target RPM | Measured RPM | Error    | Integral | Derivative | PID Output");
                    ROS_INFO("Left wheel  | %9.2f | %12.2f | %8.2f | %8.2f | %10.2f | %10.2f",
                             target_rpm_left, meas_rpm_left, pid_left.prev_error, pid_left.integral,
                             (pid_left.prev_error - pid_left.integral) / dt, output_left);
                    ROS_INFO("Right wheel | %9.2f | %12.2f | %8.2f | %8.2f | %10.2f | %10.2f",
                             target_rpm_right, meas_rpm_right, pid_right.prev_error, pid_right.integral,
                             (pid_right.prev_error - pid_right.integral) / dt, output_right);
                    ROS_INFO("Back wheel  | %9.2f | %12.2f | %8.2f | %8.2f | %10.2f | %10.2f",
                             target_rpm_back, meas_rpm_back, pid_back.prev_error, pid_back.integral,
                             (pid_back.prev_error - pid_back.integral) / dt, output_back);
                    ROS_INFO("---------------------");

                    double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time).count();
                    if (rpm_log.is_open()) {
                        rpm_log 
                            << elapsed << ","
                            << target_rpm_left  << "," << meas_rpm_left  << ","
                            << target_rpm_right << "," << meas_rpm_right << ","
                            << target_rpm_back  << "," << meas_rpm_back   << "\n";
                        rpm_log.flush();
                    }
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
        if (rpm_log.is_open()) {
            rpm_log.close();
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
