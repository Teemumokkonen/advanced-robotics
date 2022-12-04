
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <command_msgs/planAction.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <urdf/model.h>

// from kdl packages
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>              // inverse dynamics
#include <kdl/chainjnttojacsolver.hpp> 
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_kdl.h>
#include <Eigen/LU>
#include <Eigen/SVD>

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
        bool init(ros::NodeHandle n) {
            // init the action client
            pose_subs_ = n.subscribe<std_msgs::Float64MultiArray>("elfin/cvc/q", 1000, &trajectory_planner::pose_callback, this);
            target_pose_subs_ = n.subscribe<geometry_msgs::TransformStamped>("aruco_single/transform", 1000, &trajectory_planner::target_pose_callback, this);
            twist_error_pub_ = n.advertise<geometry_msgs::Twist>("Xerr_", 1000);
            vel_pub_ = n.advertise<geometry_msgs::Twist>("/elfin/cvc/command/test", 1000);
            timer = n.createTimer(ros::Duration(3), &trajectory_planner::timerCallback, this);
            n_joints_ = 6;
            q_.data = Eigen::VectorXd::Zero(n_joints_);
            qd_set_.data = Eigen::VectorXd::Zero(n_joints_);
            qd_.data = Eigen::VectorXd::Zero(n_joints_);
            qd_dot.data = Eigen::VectorXd::Zero(n_joints_);
            qd_ddot.data = Eigen::VectorXd::Zero(n_joints_);

            urdf::Model urdf;
            if (!urdf.initParam("robot_description"))
            {
                ROS_ERROR("Failed to parse urdf file");
                return false;
            }
            else
            {
                ROS_INFO("Found robot_description");
            }

            if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_))
            {
                ROS_ERROR("Failed to construct kdl tree");
                return false;
            }
            else
            {
                ROS_INFO("Constructed kdl tree");
            }

            // 4.2 kdl chain
            std::string root_name, tip_name;
            if (!n.getParam("/elfin/cvc/root_link", root_name))
            {
                ROS_ERROR("Could not find root link name");
                return false;
            }
            if (!n.getParam("/elfin/cvc/tip_link", tip_name))
            {
                ROS_ERROR("Could not find tip link name");
                return false;
            }
            if (!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
            {
                ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
                ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name);
                ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
                ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
                ROS_ERROR_STREAM("  The segments are:");

                KDL::SegmentMap segment_map = kdl_tree_.getSegments();
                KDL::SegmentMap::iterator it;

                for (it = segment_map.begin(); it != segment_map.end(); it++)
                    ROS_ERROR_STREAM("    " << (*it).first);

                return false;
            }
            else
            {
                ROS_INFO("Got kdl chain");
            }

            J_.resize(kdl_chain_.getNrOfJoints());
            J_inv_.resize(kdl_chain_.getNrOfJoints());
            J_temp_.resize(kdl_chain_.getNrOfJoints());
            J_trans_.resize(kdl_chain_.getNrOfJoints());
            Vcmd_jnt_.data = Eigen::VectorXd::Zero(n_joints_);
            q_dot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
            fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
            jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
            init_pose();
            return true;
            }


        void init_pose() {
            xd_.p(0) = 0.0;
            xd_.p(1) = 0.0;
            xd_.p(2) = 0.7; 
            xd_.M.RPY(0.0, 0.0, 0.0);
        }

        void pose_callback(const std_msgs::Float64MultiArrayConstPtr &msg) {
        for (int i = 0; i < n_joints_; i++)
        {
            q_(i) = msg->data[i];
        }

        fk_pos_solver_->JntToCart(q_, x_); // end effector poss 
        if (detection == false) {
            calc_diff();
        }
        }

        void target_pose_callback(const geometry_msgs::TransformStampedConstPtr &msg) {
            detection = true;
            timer.setPeriod(ros::Duration(3), true);
            // make a tag frame 
            xd_.M = xd_.M.Quaternion(msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w);
            xd_.p(0) = msg->transform.translation.x;
            xd_.p(1) = msg->transform.translation.y; // back away from frame
            xd_.p(2) = msg->transform.translation.z;

            //xd_ = x_ * xd_; // frame from ee-tag to world-tag
            xd_.M.DoRotX(PI/2); // rotate target frame to be in same orientation as end effector according to ENU
            calc_diff();
        }

        void calc_diff() {
            Xerr_.rot = diff(xd_ref.M, xd_.M);
            ROS_INFO("diff rot x: %f", Xerr_(3));
            ROS_INFO("diff rot y: %f", Xerr_(4));
            ROS_INFO("diff rot z: %f", Xerr_(5));


            //Xerr_.vel = 1.0 * diff(x_.p, xd_.p) / t_

            if (detection == false) {
                Xerr_ =  2.5 * diff(x_, xd_) / t_; // error from the frame
                jnt_to_jac_solver_->JntToJac(q_, J_); // jacobian of the joint
                J_inv_.data = J_.data.inverse(); // inverse of the jacobian 
                J_trans_.data = J_.data.transpose();

                Vcmd_ = Xerr_;
                //inv_solver_->CartToJnt(q_, Vcmd_, Vcmd_jnt_);
                for (size_t i = 0; i < n_joints_; i++) {
                    Vcmd_jnt_.data[i] = Vcmd_[i];
                }
                Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n_joints_, n_joints_);
                float det = J_.data.determinant();
                // check for the singulatities
                if (-0.00001 < det && det < 0.00001) {

                    J_temp_.data = (J_.data * J_trans_.data + 0.2 * I);
                    q_dot_cmd_.data = J_trans_.data * J_temp_.data.inverse() * Vcmd_jnt_.data;
                }
                else {
                    q_dot_cmd_.data = J_inv_.data * Vcmd_jnt_.data;
                }
            }
            else {
                xd_ref = xd_;
                xd_ref.p(0) = 0.0;
                xd_ref.p(1) = -0.4;
                xd_ref.p(2) = 0.0;

                xd_ref.p = xd_ref.p - xd_.p ;

                KDL::Vector u_temp;
                KDL::Vector vc;
                KDL::Vector wc;

                float theta = acos((xd_ref.M.data[0] + xd_ref.M.data[4] + xd_ref.M.data[8] - 1) / 2);
                u_temp(0) = xd_ref.M.data[5] - xd_ref.M.data[7];
                u_temp(1) = xd_ref.M.data[6] - xd_ref.M.data[2];
                u_temp(2) = xd_ref.M.data[1] - xd_ref.M.data[3];
                KDL::Vector u = (1/(2*sin(theta)))*u_temp;

                vc = -(xd_ref.p + (xd_.p * theta * u));
                wc =  -2.0 * theta * u;
                Vcmd_jnt_.data(0) = vc(0);
                Vcmd_jnt_.data(1) = vc(1);
                Vcmd_jnt_.data(2) = vc(2);
                Vcmd_jnt_.data(3) = wc(0);
                Vcmd_jnt_.data(4) = wc(1);
                Vcmd_jnt_.data(5) = wc(2);

                Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n_joints_, n_joints_);
                float det = J_.data.determinant();
                // check for the singulatities

                Vcmd_jnt_.data(5) = Xerr_(5);

                if (-0.00001 < det && det < 0.00001) {

                    J_temp_.data = (J_.data * J_trans_.data + 0.05 * I);
                    q_dot_cmd_.data = J_trans_.data * J_temp_.data.inverse() * Vcmd_jnt_.data;
                }
                else {
                    q_dot_cmd_.data = J_inv_.data * Vcmd_jnt_.data;
                }
            }




            twist_msgs_.linear.x = q_dot_cmd_(0);
            twist_msgs_.linear.y = q_dot_cmd_(1);
            twist_msgs_.linear.z = q_dot_cmd_(2);

            twist_msgs_.angular.x = q_dot_cmd_(3);
            twist_msgs_.angular.y = q_dot_cmd_(4);
            twist_msgs_.angular.z = q_dot_cmd_(5);

            if (print_state == 100) {

                ROS_INFO("\r");

                ROS_INFO("target frame x %f", xd_.p(0));
                ROS_INFO("target frame y %f", xd_.p(1));
                ROS_INFO("target frame z %f", xd_.p(2));
                
                ROS_INFO("current frame x %f", x_.p(0));
                ROS_INFO("current frame y %f", x_.p(1));
                ROS_INFO("current frame z %f", x_.p(2));
                print_state = 0;
            }
            else {
                print_state++;
            }
            vel_pub_.publish(twist_msgs_);
            publish_err();
        }

        void publish_err() {
            twist_err_msgs_.linear.x = Xerr_(0);
            twist_err_msgs_.linear.y = Xerr_(1);
            twist_err_msgs_.linear.z = Xerr_(2);
            twist_err_msgs_.angular.x = Xerr_(3);
            twist_err_msgs_.angular.y = Xerr_(4);
            twist_err_msgs_.angular.z = Xerr_(5);
            
            twist_error_pub_.publish(twist_err_msgs_);
        }

        void timerCallback(const ros::TimerEvent& event) {
            detection = false;
            xd_.p(0) = 0.0;
            xd_.p(1) = 0.0;
            xd_.p(2) = 0.7; 
            xd_.M.RPY(0.0, 0.0, 0.0);
        }

    private:
        int print_state = 0;
        KDL::Twist Vcmd_;
        boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
        boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_; // solver for jacobian
        ros::Subscriber pose_subs_;
        ros::Subscriber target_pose_subs_;
        ros::Publisher vel_pub_;
        ros::Publisher twist_error_pub_;
        KDL::Frame x_; // end effector frame
        KDL::Frame xd_, xd_ref; // end effector frame
        actionlib::SimpleActionClient<command_msgs::planAction> *ac;
        std::vector<std::string> joint_names_;
        unsigned int n_joints_;
        command_msgs::planGoal goal;
        KDL::JntArray q_, qd_set_, qd_, qd_dot, qd_ddot, q_dot_cmd_, Vcmd_jnt_;
        double t = 0.0;
        KDL::Chain kdl_chain_; // chain of the kinematic tree
        KDL::Tree kdl_tree_;   // kinematic tree based of the model urdf downloaded from the parameter server
        float t_ = 0.5;
        KDL::Twist Xerr_;
        KDL::Jacobian J_;
        KDL::Jacobian J_inv_;
        KDL::Jacobian J_trans_;
        KDL::Jacobian J_temp_;
        geometry_msgs::Twist twist_msgs_;
        geometry_msgs::Twist twist_err_msgs_;
        ros::Timer timer;
        bool detection = false;
};

};

void pose_callback() {

}

int main(int argc, char **agrv)
{
    ros::init(argc, agrv, "tag_follower");
    ros::NodeHandle n;
    trajectory_planners::trajectory_planner *planner = new trajectory_planners::trajectory_planner();
    ros::AsyncSpinner spinner(0);
    spinner.start();

    planner->init(n);
    bool cont = true;
    ros::waitForShutdown();
    return 0;
}