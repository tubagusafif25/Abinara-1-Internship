#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Header.h>
#include <cmath>

class IKSolver {
public:
    IKSolver() : nh_() {
        L1_ = 0.3; 
        L2_ = 0.3; 
        z_offset_ = 0.11; 
    
        joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
        target_sub_ = nh_.subscribe("/ik_target", 1, &IKSolver::targetCallback, this);
    
        ROS_INFO("IK Solver Node started");
        ROS_INFO("Publishing to: /joint_states");
        ROS_INFO("Subscribing to: /ik_target");
        ROS_INFO("Workspace: radius = %.2f m", L1_ + L2_);
    
        publish_joints(0.0, 0.0, 0.0);
}

    void targetCallback(const geometry_msgs::Point::ConstPtr& msg) {
        double x_target = msg->x;
        double y_target = msg->y;
        
        double z_target_relative = msg->z - z_offset_; 

        double q1, q2, q3;
        bool success = solve_ik(x_target, y_target, z_target_relative, q1, q2, q3);

        if (success) {
            ROS_INFO("IK Solution (q1, q2, q3): [%.3f, %.3f, %.3f]", q1, q2, q3);

            double x_actual, y_actual, z_actual_relative;
            solve_fk(q1, q2, q3, x_actual, y_actual, z_actual_relative);

            double delta_error = std::sqrt(
                std::pow(x_target - x_actual, 2) + 
                std::pow(y_target - y_actual, 2) + 
                std::pow(z_target_relative - z_actual_relative, 2)
            );

            ROS_INFO("Target (relative to hip): (%.3f, %.3f, %.3f)", x_target, y_target, z_target_relative);
            ROS_INFO("Actual (relative to hip): (%.3f, %.3f, %.3f)", x_actual, y_actual, z_actual_relative);
            ROS_INFO("Delta Error: %.6f", delta_error);

            publish_joints(q1, q2, q3);
        } else {
            ROS_WARN("Target (%.3f, %.3f, %.3f) is unreachable.", msg->x, msg->y, msg->z);
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher joint_pub_;
    ros::Subscriber target_sub_;
    double L1_, L2_, z_offset_;

    bool solve_ik(double x, double y, double z, double& q1, double& q2, double& q3) {
        try {
            q1 = std::atan2(y, x);

            double r = std::sqrt(x*x + y*y);

            double D_sq = r*r + z*z;
            
            double cos_q3_num = D_sq - L1_*L1_ - L2_*L2_;
            double cos_q3_den = 2 * L1_ * L2_;

            if (cos_q3_den == 0) return false; 
            
            double cos_q3 = cos_q3_num / cos_q3_den;

            if (std::abs(cos_q3) > 1.0) {
                return false; 
            }

            double sin_q3 = -std::sqrt(1 - cos_q3*cos_q3); 
            q3 = std::atan2(sin_q3, cos_q3);

            double k1 = L1_ + L2_ * cos_q3;
            double k2 = L2_ * sin_q3;
            q2 = std::atan2(z, r) - std::atan2(k2, k1);

            return true;
        } catch (const std::exception& e) {
            ROS_ERROR("Math error in IK: %s", e.what());
            return false;
        }
    }

    void solve_fk(double q1, double q2, double q3, double& x, double& y, double& z) {
        double r_plane = L1_ * std::cos(q2) + L2_ * std::cos(q2 + q3);
        
        x = std::cos(q1) * r_plane;
        y = std::sin(q1) * r_plane;
        z = L1_ * std::sin(q2) + L2_ * std::sin(q2 + q3);
    }

    void publish_joints(double q1, double q2, double q3) {
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.header.frame_id = "world";
        joint_state.name = {"hip", "shoulder", "elbow"};
        joint_state.position = {q1, q2, q3};
        joint_pub_.publish(joint_state);
    
    ROS_INFO("Publishing joint states to /joint_states");
}
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ik_solver_node");
    IKSolver solver;
    ros::spin();
    return 0;
}