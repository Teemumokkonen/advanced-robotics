/*
 This script contains and method for sending plans for the arm controller

 The plans are sent as lists of the following:
    - desired position xd
    - desired velocity xd_dot
    - desired acceleration xd_dotdot

The controller itself has been implemented as actions server, which sends the commands to the arm controller
and receives the feedback from there to see the progression of the commands.
 */


#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <command_msgs/planAction.h>
#include <std_msgs/Float64MultiArray.h>

// from kdl packages
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>              // inverse dynamics
#include <kdl/chainjnttojacsolver.hpp> 
#include <kdl/chainfksolverpos_recursive.hpp>

#include <Eigen/Core>

#define A 0.1
#define PI 3.141592
#define D2R PI / 180.0
#define R2D 180.0 / PI
#define SaveDataMax 49
#define t_set 1
#define b 2.5
#define f 1

bool update_pose = false;

namespace trajectory_planners
{
class trajectory_planner {
    public:
        bool init(ros::NodeHandle n) {
            // init the action client
            ac = new actionlib::SimpleActionClient<command_msgs::planAction>("plan/elfin", true);
            pose_subs_ = n.subscribe<std_msgs::Float64MultiArray>("elfin/computed_velocity_controller/q", 1000, &trajectory_planner::pose_callback, this);
            n_joints_ = 6;
            q_.data = Eigen::VectorXd::Zero(n_joints_);
            qd_set_.data = Eigen::VectorXd::Zero(n_joints_);
            qd_.data = Eigen::VectorXd::Zero(n_joints_);
            qd_dot.data = Eigen::VectorXd::Zero(n_joints_);
            qd_ddot.data = Eigen::VectorXd::Zero(n_joints_);
            return true;
        }

        bool plan_and_send(ros::NodeHandle n) {

            ROS_INFO("Waiting for action server to start");
            ac->waitForServer();
            ROS_INFO("Server started, please select trajectory or goal pose");

            // create plan for 100 steps
            ROS_INFO("x:=1 sine trajectory, x:=2 selectable poses for each joint, x:=3 plan in task space x:=4 exit");
            int x;
            std::cin >> x; // Get user input from the keyboard
            if (x == 1) {
                ROS_INFO("Making trajectory");
                t = 0;
                for (int i = 0; i < 4000; i++) {
                    t = t + 0.001;
                    for (size_t j = 0; j < n_joints_; j++) {
                        goal.qd_ddot.push_back(-M_PI * M_PI / 4 * 45 * D2R * sin(M_PI / 2 * t)); // desired acceleration for the controller
                        goal.qd_dot.push_back(M_PI / 2 * 45 * D2R * cos(M_PI / 2 * t)); // this value is the desired velocity to match with the controller
                        goal.qd.push_back(45 * D2R * sin(M_PI / 2* t)); // desired position for each joint
                    }
                }
                bool loop;
                ROS_INFO("should trajectory be looped?");
                ROS_INFO("loop: true or false");
                std::cin >> loop;
                goal.loop = loop;
                goal.task_space = false;
                
            }

            if (x == 2) {
                ROS_INFO("Making straight line trajectory");
                t = 0;
                float v = 1.0;
                float tau = 0.0;
                float step_size = 0.001;
                t = t + 0.001;
                for (size_t j = 0; j < n_joints_; j++) { 
                    ROS_INFO("Enter angle for joint %li", j);
                    float q;
                    std::cin >> q; // Get user input from the keyboard
                    qd_set_(j) = q; // desired acceleration for the controller
                }

                while (tau <= 1.0) { 
                    tau = tau + step_size*v;
                    qd_dot.data = v*(qd_set_.data - q_.data); // this value is the desired velocity to match with the controller
                    qd_.data = (1.0 - tau)*q_.data + tau*qd_set_.data; // desired position for each joint

                    for (size_t j = 0; j < n_joints_; j++) {
                        goal.qd_ddot.push_back(0.0); // desired acceleration for the controller
                        goal.qd_dot.push_back(qd_dot(j)); // this value is the desired velocity to match with the controller
                        goal.qd.push_back(qd_(j)); // desired position for each joint
                    }
                }

                bool loop;
                ROS_INFO("should trajectory be looped?");
                ROS_INFO("loop: true or false");
                std::cin >> loop;
                goal.loop = loop;
                goal.task_space = false;
            }

            if (x==3) {
                // generate the plan in the task space
                
                // for the lack of definations do like:
                float c;
                ROS_INFO("Give pose x:");
                std::cin >> c;
                goal.qd.push_back(c); // ee x
                ROS_INFO("Give pose: y");
                std::cin >> c;
                goal.qd_dot.push_back(c); // ee y
                ROS_INFO("Give pose z:");
                std::cin >> c;
                goal.qd_ddot.push_back(c); // ee z
                goal.x_rot.push_back(0.0);
                goal.y_rot.push_back(0.0);
                goal.z_rot.push_back(0.0);
                goal.task_space = true;
                goal.loop = false;
            }


            ROS_INFO("sending goal");
            ac->sendGoal(goal);
            bool finished_before_timeout = ac->waitForResult(ros::Duration(30.0));

            if (finished_before_timeout)
            {
                actionlib::SimpleClientGoalState state = ac->getState();
                ROS_INFO("Action finished: %s",state.toString().c_str());
                goal.qd.clear();
                goal.qd_dot.clear();
                goal.qd_ddot.clear();
                return true;
            }
            else
                ROS_INFO("Action did not finish before the time out.");
                goal.qd.clear();
                goal.qd_dot.clear();
                goal.qd_ddot.clear();
                return true;
            //exit
            
        }

        void pose_callback(const std_msgs::Float64MultiArrayConstPtr &msg) {
        for (int i = 0; i < n_joints_; i++)
        {
            q_(i) = msg->data[i];
        }   
        }

    private:
        ros::Subscriber pose_subs_;
        actionlib::SimpleActionClient<command_msgs::planAction> *ac;
        std::vector<std::string> joint_names_;
        unsigned int n_joints_;
        command_msgs::planGoal goal;
        KDL::JntArray q_, qd_set_, qd_, qd_dot, qd_ddot;
        double t = 0.0;
};

};

void pose_callback() {

}

int main(int argc, char **agrv)
{
    ros::init(argc, agrv, "trajectory_planner");
    ros::NodeHandle n;
    trajectory_planners::trajectory_planner *planner = new trajectory_planners::trajectory_planner();
    ros::AsyncSpinner spinner(0);
    spinner.start();

    planner->init(n);
    bool cont = true;
    while (ros::ok()) {
        cont = planner->plan_and_send(n);
    }
    ros::waitForShutdown();
    return 0;
}