#include "TitanDriver_ros_wrapper.h"
#include "navX_ros_wrapper.h"
#include "IOwd_ros.h"
#include "DI_ros.h"
#include "DO_ros.h"

#include <unistd.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <vmxpi_ros/Float.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <cmath>
#include <mutex>
#include <thread>
#include "geometry_msgs/Twist.h"
#include <fstream>
#include <errno.h>
#include <string.h>
#include <sys/syscall.h>

//—————————————————————————————————————————————————————————————————————————————
//                              PID CONTROLLER
//—————————————————————————————————————————————————————————————————————————————
struct PID
{
    double kp, ki, kd;
    double prev_error;
    double integral;

    PID(double p = 0.0, double i = 0.0, double d = 0.0)
        : kp(p), ki(i), kd(d), prev_error(0.0), integral(0.0) {}

    /// Compute one PID step. Clamps the integral to avoid windup.
    double compute(double setpoint, double measured, double dt)
    {
        double error = setpoint - measured;
        integral += error * dt;

        // Clamp integral term
        const double integral_limit = 1000.0;
        if (integral > integral_limit) integral = integral_limit;
        if (integral < -integral_limit) integral = -integral_limit;

        double derivative = (error - prev_error) / dt;
        prev_error = error;

        return (kp * error) + (ki * integral) + (kd * derivative);
    }
};

//—————————————————————————————————————————————————————————————————————————————
//                              ABT FILTER (g-h-k)
//—————————————————————————————————————————————————————————————————————————————
class ABTFilter
{
public:
    double alpha, beta, gamma;
    double input;       // raw integrated position (cumDist)
    double pos_output;  // filtered position
    double vel_output;  // filtered velocity
    double acc_output;  // filtered acceleration

    ABTFilter(double a=0.0, double b=0.0, double g=0.0)
        : alpha(a), beta(b), gamma(g),
          input(0.0), pos_output(0.0), vel_output(0.0), acc_output(0.0)
    {
    }

    /// Initialize the filter state at startup.
    void initialize(double init_pos = 0.0)
    {
        pos_output = init_pos;
        vel_output = 0.0;
        acc_output = 0.0;
        input = init_pos;
    }

    /// “Stop” mode: sets position to the latest input and zeroes velocity/acc.
    void stop()
    {
        pos_output = input;
        vel_output = 0.0;
        acc_output = 0.0;
    }

    /// One iteration of the g-h-k (ABT) update, given dt (in seconds).
    void update(double dt)
    {
        // 1) Predict
        double pos_predict = pos_output + dt * vel_output + 0.5 * dt * dt * acc_output;
        double vel_predict = vel_output + dt * acc_output;

        // 2) Compute residual
        double residual = input - pos_predict;

        // 3) Correct
        pos_output = pos_predict + alpha * residual;
        vel_output = vel_predict + (beta / dt) * residual;
        acc_output = acc_output + (gamma * 0.5 / (dt * dt)) * residual;
    }
};

//—————————————————————————————————————————————————————————————————————————————
//                           GLOBAL ENCODER CALLBACKS
//—————————————————————————————————————————————————————————————————————————————
static std::mutex     enc_mutex;
static double         left_count  = 0.0;
static double         right_count = 0.0;
static double         back_count  = 0.0;

void enc0Callback(const std_msgs::Int32::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(enc_mutex);
    left_count = static_cast<double>(msg->data);
}

void enc1Callback(const std_msgs::Int32::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(enc_mutex);
    right_count = static_cast<double>(msg->data);
}

void enc2Callback(const std_msgs::Int32::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(enc_mutex);
    back_count = static_cast<double>(msg->data);
}

//—————————————————————————————————————————————————————————————————————————————
//                                  ROBOT CLASS
//—————————————————————————————————————————————————————————————————————————————
class Robot
{
private:
    ros::Subscriber                enc0_sub, enc1_sub, enc2_sub;
    ros::Subscriber                vel_sub;
    ros::ServiceClient             set_m_speed, enable_client, disable_client;
    ros::ServiceClient             resetAngle, res_encoder_client, stop_motors_client;

    std::thread                    control_loop_thread;
    std::chrono::steady_clock::time_point last_cmd_time, last_vel_time;
    const double                   COMMAND_TIMEOUT = 0.5; // seconds

    // Last cmd_vel components (protected by this mutex)
    std::mutex                     cmd_mutex;
    double                         cmd_linear_x = 0.0;
    double                         cmd_linear_y = 0.0;
    double                         cmd_angular_z = 0.0;

    // Target wheel speeds (rad/s and RPM)
    double                         target_w_left  = 0.0;
    double                         target_w_right = 0.0;
    double                         target_w_back  = 0.0;

    double                         target_rpm_left  = 0.0;
    double                         target_rpm_right = 0.0;
    double                         target_rpm_back  = 0.0;

    // Measured (filtered) RPM from ABT outputs
    double                         meas_rpm_left  = 0.0;
    double                         meas_rpm_right = 0.0;
    double                         meas_rpm_back  = 0.0;

    // Final normalized motor commands in [-1, +1]
    double                         final_left  = 0.0;
    double                         final_right = 0.0;
    double                         final_back  = 0.0;

    // PID controllers
    PID                            pid_left;
    PID                            pid_right;
    PID                            pid_back;

    // ABT filters
    ABTFilter                      abt_left;
    ABTFilter                      abt_right;
    ABTFilter                      abt_back;

    // Totals for distance integration
    double                         cumDistLeft  = 0.0;
    double                         cumDistRight = 0.0;
    double                         cumDistBack  = 0.0;

    // Wheel & encoder parameters
    const double                   r = 0.051;         // wheel radius (m)
    const int                      TPR = 1464;        // ticks per revolution

    // Last raw encoder counts (for delta calculation)
    int                            last_left_count_int  = 0;
    int                            last_right_count_int = 0;
    int                            last_back_count_int  = 0;

    // For logging
    std::ofstream                  rpm_log;
    std::chrono::steady_clock::time_point start_time;

public:
    Robot(ros::NodeHandle* nh)
        : pid_left(1.0, 1.0, 0.001),
          pid_right(1.0, 1.0, 0.001),
          pid_back(1.0, 1.0, 0.001),
          abt_left(0.8, 0.4, 0.2),
          abt_right(0.8, 0.4, 0.2),
          abt_back(0.8, 0.4, 0.2)
    {
        start_time = std::chrono::steady_clock::now();

        // Try opening the CSV log
        rpm_log.open("/home/pi/rpm_log.csv", std::ios::out | std::ios::trunc);
        if (!rpm_log.is_open())
        {
            ROS_FATAL("Could not open /home/pi/rpm_log.csv: %s", strerror(errno));
            // If we fail, we continue without logging to avoid shutting down.
        }
        else
        {
            rpm_log << "timestamp,left_target,left_measured,"
                    << "right_target,right_measured,"
                    << "back_target,back_measured\n";
            rpm_log.flush();
        }

        // Load PID parameters from ROS param server (if provided)
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

        pid_left  = PID(p_left, i_left, d_left);
        pid_right = PID(p_right, i_right, d_right);
        pid_back  = PID(p_back, i_back, d_back);

        // Initialize ABT filters
        abt_left.initialize(0.0);
        abt_right.initialize(0.0);
        abt_back.initialize(0.0);

        // Create publishers/subscribers/controllers
        set_m_speed       = nh->serviceClient<vmxpi_ros::MotorSpeed>("titan/set_motor_speed");
        enable_client     = nh->serviceClient<std_srvs::Trigger>("titan/enable");
        disable_client    = nh->serviceClient<std_srvs::Trigger>("titan/disable");
        resetAngle        = nh->serviceClient<std_srvs::Empty>("reset_navx");
        res_encoder_client = nh->serviceClient<std_srvs::Trigger>("titan/reset_encoder");
        stop_motors_client = nh->serviceClient<std_srvs::Trigger>("titan/stop_motors");

        enc0_sub = nh->subscribe("titan/encoder0/count", 1, enc0Callback);
        enc1_sub = nh->subscribe("titan/encoder1/count", 1, enc1Callback);
        enc2_sub = nh->subscribe("titan/encoder2/count", 1, enc2Callback);

        vel_sub = nh->subscribe<geometry_msgs::Twist>("cmd_vel", 10, &Robot::cmdVelCallback, this);

        last_cmd_time = std::chrono::steady_clock::now();
        last_vel_time = std::chrono::steady_clock::now();

        // Initialize last encoder count integers
        {
            std::lock_guard<std::mutex> lock(enc_mutex);
            last_left_count_int  = static_cast<int>(left_count);
            last_right_count_int = static_cast<int>(right_count);
            last_back_count_int  = static_cast<int>(back_count);
        }

        // Start control loop thread
        control_loop_thread = std::thread(&Robot::controlLoop, this);
    }

    /// Callback whenever we receive a new /cmd_vel
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(cmd_mutex);
        last_cmd_time = std::chrono::steady_clock::now();
        cmd_linear_x  = msg->linear.x;
        cmd_linear_y  = msg->linear.y;
        cmd_angular_z = msg->angular.z;
    }

    /// Convert desired vx, vy, wz into wheel angular velocities (rad/s)
    void holonomicDrive(double vx, double vy, double wz)
    {
        // Following your original mixing equations:
        double rightSpeed = -0.33 * vy - 0.58 * vx - 0.33 * wz;
        double leftSpeed  = -0.33 * vy + 0.58 * vx - 0.33 * wz;
        double backSpeed  =  0.67 * vy - 0.33 * wz;

        // Convert angular velocity (rad/s) into RPM
        target_w_left  = leftSpeed;
        target_w_right = rightSpeed;
        target_w_back  = backSpeed;

        target_rpm_left  = (target_w_left  / r) * 60.0 / (2.0 * M_PI);
        target_rpm_right = (target_w_right / r) * 60.0 / (2.0 * M_PI);
        target_rpm_back  = (target_w_back  / r) * 60.0 / (2.0 * M_PI);
    }

    /// Main control loop (20 Hz)
    void controlLoop()
    {
        ros::Rate rate(20); // 20 Hz => dt ≈ 0.05 s per iteration
        while (ros::ok())
        {
            auto now = std::chrono::steady_clock::now();

            // 1) Compute dt since last velocity update
            double dt = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_vel_time).count();
            last_vel_time = now;

            if (dt <= 0.0) {
                dt = 1e-6;  
            }

            // 2) Check if we should stop due to timeout or zero command
            double elapsed_since_cmd = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_cmd_time).count();
            bool zeroCommand = false;
            {
                std::lock_guard<std::mutex> lock(cmd_mutex);
                zeroCommand = (std::fabs(cmd_linear_x)  < 1e-6 &&
                               std::fabs(cmd_linear_y)  < 1e-6 &&
                               std::fabs(cmd_angular_z) < 1e-6)
                              || (elapsed_since_cmd > COMMAND_TIMEOUT);
            }

            if (zeroCommand)
            {
                // Zero everything, reset integrals, and stop ABT
                final_left  = 0.0;
                final_right = 0.0;
                final_back  = 0.0;

                pid_left.integral  = 0.0; pid_left.prev_error  = 0.0;
                pid_right.integral = 0.0; pid_right.prev_error = 0.0;
                pid_back.integral  = 0.0; pid_back.prev_error  = 0.0;

                abt_left.stop();
                abt_right.stop();
                abt_back.stop();
            }
            else
            {
                // 1) Read current encoder counts
                int currL, currR, currB;
                {
                    std::lock_guard<std::mutex> lock(enc_mutex);
                    currL = static_cast<int>(left_count);
                    currR = static_cast<int>(right_count);
                    currB = static_cast<int>(back_count);
                }

                // 2) Compute delta ticks since last loop
                int dL = currL - last_left_count_int;
                int dR = currR - last_right_count_int;
                int dB = currB - last_back_count_int;

                last_left_count_int  = currL;
                last_right_count_int = currR;
                last_back_count_int  = currB;

                // 3) Convert ticks → meters
                // wheelCirc = 2π r; ticks → rev; rev→distance
                const double wheelCirc = 2.0 * M_PI * r;
                cumDistLeft  += (static_cast<double>(dL) / static_cast<double>(TPR)) * wheelCirc;
                cumDistRight += (static_cast<double>(dR) / static_cast<double>(TPR)) * wheelCirc;
                cumDistBack  += (static_cast<double>(dB) / static_cast<double>(TPR)) * wheelCirc;

                // 4) Feed cumDist into ABT filters
                abt_left.input  = cumDistLeft;
                abt_right.input = cumDistRight;
                abt_back.input  = cumDistBack;

                abt_left.update(dt);
                abt_right.update(dt);
                abt_back.update(dt);

                // 5) Extract filtered velocity → RPM
                meas_rpm_left  = (abt_left.vel_output  / r) * 60.0 / (2.0 * M_PI);
                meas_rpm_right = (abt_right.vel_output / r) * 60.0 / (2.0 * M_PI);
                meas_rpm_back  = (abt_back.vel_output  / r) * 60.0 / (2.0 * M_PI);

                // 6) Compute desired wheel RPM from latest cmd_vel
                double vx, vy, wz;
                {
                    std::lock_guard<std::mutex> lock(cmd_mutex);
                    vx = cmd_linear_x;
                    vy = cmd_linear_y;
                    wz = cmd_angular_z;
                }
                holonomicDrive(vx, vy, wz);

                // 7) PID computation
                double out_left  = pid_left.compute(target_rpm_left,  meas_rpm_left,  dt);
                double out_right = pid_right.compute(target_rpm_right, meas_rpm_right, dt);
                double out_back  = pid_back.compute(target_rpm_back,  meas_rpm_back,  dt);

                // Normalize into [-1, +1] based on max RPM
                const double max_motor_rpm = 160.0; 
                final_left  = out_left  / max_motor_rpm;
                final_right = out_right / max_motor_rpm;
                final_back  = out_back  / max_motor_rpm;

                // Clamp to [-1, 1]
                final_left  = std::max(-1.0, std::min(final_left,  1.0));
                final_right = std::max(-1.0, std::min(final_right, 1.0));
                final_back  = std::max(-1.0, std::min(final_back,  1.0));

                // 8) (Optional) Log RPMs to CSV
                if (rpm_log.is_open())
                {
                    double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time).count();
                    rpm_log 
                        << elapsed << ","
                        << target_rpm_left  << "," << meas_rpm_left  << ","
                        << target_rpm_right << "," << meas_rpm_right << ","
                        << target_rpm_back  << "," << meas_rpm_back   << "\n";
                    rpm_log.flush();
                }
            }

            // 9) Publish to motors every iteration
            publishMotorCommands();

            rate.sleep();
        }
    }

    /// Send final_left, final_right, final_back ([-1,1]) to Titan via service calls
    void publishMotorCommands()
    {
        vmxpi_ros::MotorSpeed msg;

        msg.request.motor = 0;
        msg.request.speed = final_left;
        if (!set_m_speed.call(msg))
        {
            ROS_ERROR("Failed to set speed for Left Motor");
        }

        msg.request.motor = 1;
        msg.request.speed = final_right;
        if (!set_m_speed.call(msg))
        {
            ROS_ERROR("Failed to set speed for Right Motor");
        }

        msg.request.motor = 2;
        msg.request.speed = final_back;
        if (!set_m_speed.call(msg))
        {
            ROS_ERROR("Failed to set speed for Back Motor");
        }
    }

    ~Robot()
    {
        if (control_loop_thread.joinable())
        {
            control_loop_thread.join();
        }
        if (rpm_log.is_open())
        {
            rpm_log.close();
        }
    }
};

//—————————————————————————————————————————————————————————————————————————————
//                                  MAIN ENTRY
//—————————————————————————————————————————————————————————————————————————————
int main(int argc, char** argv)
{
    // If frcKillRobot.sh is needed, keep it; otherwise you can comment it out.
    system("/usr/local/frc/bin/frcKillRobot.sh");

    ros::init(argc, argv, "vmxpi_ros_wrapper");
    ros::NodeHandle nh;

    // Instantiate the hardware wrapper (Titan + VMXPi) before Robot
    VMXPi vmx(true, (uint8_t)50);
    TitanDriverROSWrapper titan(&nh, &vmx);
    ROS_INFO("Titan driver is now started");

    // (NavX wrapper can be enabled if needed)
    // navXROSWrapper navx(&nh, &vmx);

    Robot robot(&nh);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::waitForShutdown();
    ROS_INFO("ROS SHUTDOWN");

    return 0;
}
