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

// from kdl packages
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>


#define A 0.1
#define PI 3.141592
#define D2R PI / 180.0
#define R2D 180.0 / PI
#define SaveDataMax 49
#define t_set 1
#define b 2.5
#define f 1

namespace trajectory_planners
{
class trajectory_planner {
    public:
        bool init() {
            // init the action client
            ac = new actionlib::SimpleActionClient<command_msgs::planAction>("plan/elfin", true);
            return true;
        }

        bool plan_and_send(ros::NodeHandle n) {

            ROS_INFO("Waiting for action server to start");
            ac->waitForServer();
            ROS_INFO("Server started, sending goal");

            n_joints_ = 6;
            // create plan for 100 steps
            for (int i = 0; i < 10000; i++) {
                t = t + 0.001;
                for (size_t j = 0; j < n_joints_; j++) {
                    goal.qd_ddot.push_back(-M_PI * M_PI / 4 * 45 * D2R * sin(M_PI / 2 * t)); // desired acceleration for the controller
                    goal.qd_dot.push_back(M_PI / 2 * 45 * D2R * cos(M_PI / 2 * t)); // this value is the desired velocity to match with the controller
                    goal.qd.push_back(45 * D2R * sin(M_PI / 2* t)); // desired position for each joint
                    double test = (45 * D2R * sin(M_PI / 2* t));

                    }
            }
            ac->sendGoal(goal);
            bool finished_before_timeout = ac->waitForResult(ros::Duration(30.0));

            if (finished_before_timeout)
            {
                actionlib::SimpleClientGoalState state = ac->getState();
                ROS_INFO("Action finished: %s",state.toString().c_str());
                return true;
            }
            else
                ROS_INFO("Action did not finish before the time out.");
                return false;
            //exit
            
        }
    private:
        actionlib::SimpleActionClient<command_msgs::planAction> *ac;
        std::vector<std::string> joint_names_;
        unsigned int n_joints_;
        command_msgs::planGoal goal;
        double t = 0.0;
};

};

int main(int argc, char **agrv)
{
    ros::init(argc, agrv, "trajectory_planner");
    ros::NodeHandle n;
    trajectory_planners::trajectory_planner *planner = new trajectory_planners::trajectory_planner();
    planner->init();
    planner->plan_and_send(n);
    return 0;
}