
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <command_msgs/planAction.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
#include <urdf/model.h>
#include <bits/stdc++.h>
// from kdl packages
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>              // inverse dynamics
#include <kdl/chainjnttojacsolver.hpp> 
#include <kdl/treejnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/treefksolver.hpp>
//#include <kdl/treefksolverpose.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_kdl.h>
#include <Eigen/LU>
#include <Eigen/SVD>

#include <math.h>

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
            des_subs = n.subscribe<std_msgs::Float64MultiArray>("/planner/des_frame", 1000, &trajectory_planner::des_callback, this);
            target_pose_subs_ = n.subscribe<geometry_msgs::TransformStamped>("target_pose", 1000, &trajectory_planner::target_pose_callback, this);
            sensor_filtered = n.subscribe<geometry_msgs::Wrench>("/elfin/cvc/sensor_filtered", 1000, &trajectory_planner::sensor_filtered_callback, this);
            twist_error_pub_ = n.advertise<geometry_msgs::Twist>("Xerr_", 1000);
            ee_wrench_in_world = n.advertise<geometry_msgs::Wrench>("ee_wrench_world", 1000);
            vel_pub_ = n.advertise<std_msgs::Float64MultiArray>("/elfin/cvc/command/test", 1000);

            n_joints_ = 6;
            q_.data = Eigen::VectorXd::Zero(n_joints_);
            q_dot_.data = Eigen::VectorXd::Zero(n_joints_);
            qd_set_.data = Eigen::VectorXd::Zero(n_joints_);
            qd_.data = Eigen::VectorXd::Zero(n_joints_);
            qd_dot.data = Eigen::VectorXd::Zero(n_joints_);
            qd_ddot.data = Eigen::VectorXd::Zero(n_joints_);

            qdes_.data = Eigen::VectorXd::Zero(n_joints_);
            qdes_dot_.data = Eigen::VectorXd::Zero(n_joints_);
            qdes_ddot_.data = Eigen::VectorXd::Zero(n_joints_);

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

            //Getting the gains

            ex_.data = Eigen::VectorXd::Zero(n_joints_);
            xdes_dot_.data = Eigen::VectorXd::Zero(n_joints_);
            xdes_ddot_.data = Eigen::VectorXd::Zero(n_joints_);
            xee_dot_.data = Eigen::VectorXd::Zero(n_joints_);
            ex_dot_.data = Eigen::VectorXd::Zero(n_joints_);
            ex_int_.data = Eigen::VectorXd::Zero(n_joints_);
            Kp_.resize(n_joints_);
            Kd_.resize(n_joints_);
            Ki_.resize(n_joints_);

            Kp_ws_.resize(n_joints_);
            Kd_ws_.resize(n_joints_);
            Ki_ws_.resize(n_joints_);

            M_t_.resize(6);
            K_Dt_.resize(6);
            K_Pt_.resize(6);

            std::vector<double> Kp(n_joints_), Ki(n_joints_), Kd(n_joints_), Mt(6), KDt(6), KPt(6);
            for (size_t i = 0; i < 3; i++){
                std::string si = boost::lexical_cast<std::string>(i + 1);
                if (n.getParam("/elfin/cvc/gains/comp_cof/M_t/mt_" +si, Mt[i]))
                {
                    M_t_(i) = Mt[i];
                }
                else
                {
                    ROS_ERROR("Cannot find M_t gain");
                    return false;
                }

                if (n.getParam("/elfin/cvc/gains/comp_cof/K_Dt/kdt_" + si , KDt[i]))
                {
                    K_Dt_(i) = KDt[i];
                }
                else
                {
                    ROS_ERROR("Cannot find K_Dt gain");
                    return false;
                }

                if (n.getParam("/elfin/cvc/gains/comp_cof/K_Pt/kpt_" + si, KPt[i]))
                {
                    K_Pt_(i) = KPt[i];
                }
                else
                {
                    ROS_ERROR("Cannot find K_Pt gain");
                    return false;
                }

            }
            for (size_t i = 0; i < n_joints_; i++)
            {
                std::string si = boost::lexical_cast<std::string>(i + 1);
                if (n.getParam("/elfin/cvc/gains/elfin_joint" + si + "/pid/p", Kp[i]))
                {
                    Kp_(i) = Kp[i];
                }
                else
                {
                    std::cout << "/elfin/cVc/gains/elfin_joint" + si + "/pid/p" << std::endl;
                    ROS_ERROR("Cannot find pid/p gain");
                    return false;
                }

                if (n.getParam("/elfin/cvc/gains/elfin_joint" + si + "/pid/i", Ki[i]))
                {
                    Ki_(i) = Ki[i];
                }
                else
                {
                    ROS_ERROR("Cannot find pid/i gain");
                    return false;
                }

                if (n.getParam("/elfin/cvc/gains/elfin_joint" + si + "/pid/d", Kd[i]))
                {
                    Kd_(i) = Kd[i];
                }
                else
                {
                    ROS_ERROR("Cannot find pid/d gain");
                    return false;
                }
                //
                if (n.getParam("/elfin/cvc/gains/elfin_joint" + si + "/pid_ws/p", Kp[i]))
                {
                    Kp_ws_(i) = Kp[i];
                }
                else
                {
                    std::cout << "/elfin/cVc/gains/elfin_joint" + si + "/pid_ws/p" << std::endl;
                    ROS_ERROR("Cannot find pid_ws/p gain");
                    return false;
                }

                if (n.getParam("/elfin/cvc/gains/elfin_joint" + si + "/pid_ws/i", Ki[i]))
                {
                    Ki_ws_(i) = Ki[i];
                }
                else
                {
                    ROS_ERROR("Cannot find pid_ws/i gain");
                    return false;
                }

                if (n.getParam("/elfin/cvc/gains/elfin_joint" + si + "/pid_ws/d", Kd[i]))
                {
                    Kd_ws_(i) = Kd[i];
                }
                else
                {
                    ROS_ERROR("Cannot find pid_ws/d gain");
                    return false;
                }
            }
            
            y_.data = Eigen::VectorXd::Zero(n_joints_);
            e_.data = Eigen::VectorXd::Zero(n_joints_);
            e_dot_.data = Eigen::VectorXd::Zero(n_joints_);
            e_int_.data = Eigen::VectorXd::Zero(n_joints_);
            time = 0;
            dt = 0;

            ///

            ROS_INFO("Number of joints = %d ", kdl_chain_.getNrOfJoints());
            J_.resize(kdl_chain_.getNrOfJoints());
            J2_.resize(6);
            J3_.resize(6);
            J4_.resize(6);
            J5_.resize(6);
            J6_.resize(6);

            J_inv_.resize(kdl_chain_.getNrOfJoints());
            J_dot_.resize(kdl_chain_.getNrOfJoints());
            J_temp_.resize(kdl_chain_.getNrOfJoints());
            J_trans_.resize(kdl_chain_.getNrOfJoints());
            Vcmd_jnt_.data = Eigen::VectorXd::Zero(n_joints_);
            q_dot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
            fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
            //fk_pos_solver_pose_.reset(new KDL::TreeFkSolverPos(kdl_tree_));
            jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
        
            TreeJntToJacSolver_.reset(new KDL::TreeJntToJacSolver(kdl_tree_));
            jnt_to_jac_dot_solver_.reset(new KDL::ChainJntToJacDotSolver(kdl_chain_));
            init_pose();
            init_obs();
            init_imp();
            return true;
            }

        void init_obs(){
            // populate artificial obstacle fields for obstacle

            KDL::Vector obs_point;
            obs_point(0) = 0.0; //x
            obs_point(1) = -0.45;  //y
            obs_point(2) = 0.2; //z
            obs_points_.push_back(obs_point);
            obs_point(2) = 0.4; //z
            obs_points_.push_back(obs_point);
            obs_point(2) = 0.6; //z
            obs_points_.push_back(obs_point);
            obs_point(2) = 0.8; //z
            obs_points_.push_back(obs_point);
        }
        void init_imp(){
            z_imp_dot_.data = Eigen::VectorXd::Zero(n_joints_);
            z_imp_ddot.data = Eigen::VectorXd::Zero(n_joints_);
            z_imp_.p(0) = 0;
            z_imp_.p(1) = 0;
            z_imp_.p(2) = 0;
            z_imp_.M = KDL::Rotation::Identity();
        }

        // initial pose for robot to match
        void init_pose() {
            xd_.p(0) = -0.3;
            xd_.p(1) = -0.2;
            xd_.p(2) = 0.5; 
            xd_.M = xd_.M.RPY(0.0, 0.0, 0.1);
        }

        void des_callback(const std_msgs::Float64MultiArrayConstPtr &msg){
            //ROS_INFO("first msg: %f", msg->data[0]);

            xdes_.p(0) = msg->data[0];
            xdes_.p(1) = msg->data[1];
            xdes_.p(2) = msg->data[2];
            xdes_.M = KDL::Rotation(KDL::Rotation::RPY(msg->data[3], msg->data[4], msg->data[5]));

            for(int i = 0; i < 6; i++){
                xdes_dot_.data[i] = msg->data[i+6];
                xdes_ddot_.data[i] = msg->data[i + 12];
            }
        }
        void pose_callback(const std_msgs::Float64MultiArrayConstPtr &msg) {
        for (int i = 0; i < n_joints_; i++)
        {
            q_(i) = msg->data[2*i];
            q_dot_(i) = msg->data[2*i+1];
        }
        double time_past = time;
        time = msg->data[12];
        dt = abs(time - time_past);
        if (dt > 1){dt = 0;}

        fk_pos_solver_->JntToCart(q_, xee_); // end effector poss 
        

///////////////////
        
        fk_pos_solver_->JntToCart(q_, x_2_, 3);
        fk_pos_solver_->JntToCart(q_, x_3_, 4);
        fk_pos_solver_->JntToCart(q_, x_4_, 5);
        fk_pos_solver_->JntToCart(q_, x_5_, 6);
        fk_pos_solver_->JntToCart(q_, x_6_, 7);
        fk_pos_solver_->JntToCart(q_, x_); // end effector poss 
        
        FK_vec_[0] = x_;
        FK_vec_[1] = x_2_;
        FK_vec_[2] = x_3_;
        FK_vec_[3] = x_4_;
        FK_vec_[4] = x_5_;
        FK_vec_[5] = x_6_;

        TreeJntToJacSolver_->JntToJac(q_, J2_, "elfin_joint1");
        TreeJntToJacSolver_->JntToJac(q_, J3_, "elfin_joint2");
        TreeJntToJacSolver_->JntToJac(q_, J4_, "elfin_joint3");
        TreeJntToJacSolver_->JntToJac(q_, J5_, "elfin_joint4");
        TreeJntToJacSolver_->JntToJac(q_, J6_, "elfin_joint5");
        jnt_to_jac_solver_->JntToJac(q_, J_); // jacobian of the joint
        jnt_to_jac_solver_->JntToJac(q_, J2_, 3); // jacobian of the joint
        jnt_to_jac_solver_->JntToJac(q_, J3_, 4); // jacobian of the joint
        jnt_to_jac_solver_->JntToJac(q_, J4_, 5); // jacobian of the joint
        jnt_to_jac_solver_->JntToJac(q_, J5_, 6); // jacobian of the joint
        jnt_to_jac_solver_->JntToJac(q_, J6_, 7); // jacobian of the joint

        Jac_vec_[0] = J_;
        Jac_vec_[1] = J2_;
        Jac_vec_[2] = J3_;
        Jac_vec_[3] = J4_;
        Jac_vec_[4] = J5_; 
        Jac_vec_[5] = J6_; 
//////////////////////////////
        xee_dot_.data = J_.data * q_dot_.data;

        //J_inv_.data = J_.data.inverse(); // inverse of the jacobian 
        float det = J_.data.determinant();
        J_trans_.data = J_.data.transpose();
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n_joints_, n_joints_);
        // check for the singulatities
        if (-0.0001 < det && det < 0.0001) {
            ROS_INFO("Singularity detected");

            J_temp_.data = (J_.data * J_trans_.data + 200 * (0.0001 - abs(det)) * I);
            J_inv_.data = J_trans_.data * J_temp_.data.inverse();
        }
        else {
            //J_temp_.data = (J_.data * J_trans_.data + 0 * (0.001 - abs(det)) * I);
            //J_temp_.data = J_trans_.data * J_temp_.data.inverse();
            J_inv_.data = J_.data.inverse();// * fmin(1, (1-(0.005 - abs(det))/0.004)) + J_temp_.data * fmax(0, (0.005 - abs(det))/0.004); 
        }

        q_dot_jntArry = KDL::JntArrayVel(q_, q_dot_);
        jnt_to_jac_dot_solver_->JntToJacDot(q_dot_jntArry, J_dot_);
        calc_diff();
        
        }

        void sensor_filtered_callback(const geometry_msgs::WrenchConstPtr &msg) {
            // make a tag frame 
            KDL::Wrench temp1;
            temp1.force.x(1);
            temp1.force.y(1);
            temp1.force.z(1);
            temp1.torque.x(1);
            temp1.torque.y(1);
            temp1.torque.z(1);
            KDL::Wrench temp = x_3_.M * temp1;
            

            //if(he_ != KDL::Wrench::Zero() && ((temp.force - he_.force).Norm() < 1.2*temp.force.Norm())){
            he_ = temp;
            //}

            //double alpha, beta, gama;
            //xee_.M.GetEulerZYX(alpha, beta, gama);
            //ROS_INFO("current frame x %f, %f, %f", alpha, beta, gama);
            //ROS_INFO("forces are: %f , %f, %f", he_.force.x(), he_.force.y(), he_.force.z());
            //ROS_INFO("forces2 are: %f , %f, %f", msg->force.x, msg->force.y, msg->force.z);
            //if (he_.force.x() < 0){

            //            ROS_WARN("current frame x %f, %f, %f", alpha, beta, gama);
            //            ROS_WARN("forces are: %f , %f, %f", he_.force.x(), he_.force.y(), he_.force.z());
            //            ROS_WARN("forces2 are: %f , %f, %f", msg->force.x, msg->force.y, msg->force.z);
            //}
            geometry_msgs::Wrench msg2;
            msg2.force.x = he_.force.x();
            msg2.force.x = he_.force.x();
            msg2.force.y = he_.force.y();
            msg2.torque.z = he_.torque.z();
            msg2.torque.y = he_.torque.y();
            msg2.torque.z = he_.torque.z();
            ee_wrench_in_world.publish(msg2);


            //xd_ = x_ * xd_; // frame from ee-tag to world-tag
            //xd_.M.DoRotX(PI/2); // rotate target frame to be in same orientation as end effector according to ENU
        }
        void target_pose_callback(const geometry_msgs::TransformStampedConstPtr &msg) {
            // make a tag frame 
            xd_.M = xd_.M.Quaternion(msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w);
            xd_.p(0) = msg->transform.translation.x;
            xd_.p(1) = msg->transform.translation.y; 
            xd_.p(2) = msg->transform.translation.z;

            //xd_ = x_ * xd_; // frame from ee-tag to world-tag
            //xd_.M.DoRotX(PI/2); // rotate target frame to be in same orientation as end effector according to ENU
        }

        void calc_diff() {
            Xerr_.rot = 10.0 * diff(x_.M, xd_.M) / t_;
            Xerr_.vel = 15.0 * diff(x_.p, xd_.p) / t_;


            //Xerr_ =  15.0 * diff(x_, xd_) / t_; // error from the frame
            

            Vcmd_ = Xerr_;
            //inv_solver_->CartToJnt(q_, Vcmd_, Vcmd_jnt_);
            for (size_t i = 0; i < n_joints_; i++) {
                Vcmd_jnt_.data[i] = Vcmd_[i];
            }
            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n_joints_, n_joints_);
            float det = J_.data.determinant();
            // check for the singulatities
            if (-0.00001 < det && det < 0.00001) {
                //ROS_INFO("Singularity detected");

                J_temp_.data = (J_.data * J_trans_.data + 0.05 * I);
                q_dot_cmd_.data = J_trans_.data * J_temp_.data.inverse() * Vcmd_jnt_.data;
            }
            else {
                q_dot_cmd_.data = J_inv_.data * Vcmd_jnt_.data;
            }

            //KDL::JntArray q = rep_potential_end_effector();
            KDL::JntArray q; 
            KDL::JntArray q_pot_rep;
            q.data = Eigen::VectorXd::Zero(n_joints_);

            for (int i = 0; i < FK_vec_.size() ; i++) {

                q_pot_rep.data = Eigen::VectorXd::Zero(n_joints_);
                q_pot_rep = rep_potential_sum(FK_vec_.at(i), Jac_vec_.at(i));

                for (int j = 0; j < n_joints_; j++) {
                    q(j) += q_pot_rep(j) + joint_limit_rep(j);
                    }
                }

            if (print_state == 100) {

                //ROS_INFO("\r");

                //ROS_INFO("target frame x %f", xd_.p(0));
                //ROS_INFO("target frame y %f", xd_.p(1));
                //ROS_INFO("target frame z %f", xd_.p(2));

                //ROS_INFO("\r");

                //ROS_INFO("repulsive joint 1: %f", q(0));
                //ROS_INFO("repulsive joint 2: %f", q(1));
                //ROS_INFO("repulsive joint 3: %f", q(2));
                //ROS_INFO("repulsive joint 4: %f", q(0));
                //ROS_INFO("repulsive joint 5: %f", q(1));
                //ROS_INFO("repulsive joint 6: %f", q(2));
                //ROS_INFO("\r");
                
                //ROS_INFO("attractive joint 1: %f", q_dot_cmd_(0));
                //ROS_INFO("attractive joint 2: %f", q_dot_cmd_(1));
                //ROS_INFO("attractive joint 3: %f", q_dot_cmd_(2));
                //ROS_INFO("attractive joint 4: %f", q_dot_cmd_(0));
                //ROS_INFO("attractive joint 5: %f", q_dot_cmd_(1));
                //ROS_INFO("attractive joint 6: %f", q_dot_cmd_(2));

                
                print_state = 0;
            }
            else {
                print_state++;
            }

            q_dot_cmd_.data = q_dot_cmd_.data + q.data; 


            twist_msgs_.linear.x = q_dot_cmd_(0);
            twist_msgs_.linear.y = q_dot_cmd_(1);
            twist_msgs_.linear.z = q_dot_cmd_(2);

            twist_msgs_.angular.x = q_dot_cmd_(3);
            twist_msgs_.angular.y = q_dot_cmd_(4);
            twist_msgs_.angular.z = q_dot_cmd_(5);

            ////
            y_ = WS_controller(xdes_, xdes_dot_, xdes_ddot_);
            //dance_mode();

            

            joint_cmds_.data.clear();

            for(int i = 0; i< n_joints_ ; i++){
                joint_cmds_.data.push_back(y_(i));
            }


            vel_pub_.publish(joint_cmds_);
            publish_err();
        }

        void dance_mode(){
            t = time;
            for(int i = 0; i < n_joints_; i+=4){
                qdes_.data[i] = 45 * D2R * sin(M_PI / 2* t);
                qdes_dot_.data[i] = 45 * D2R * M_PI/2 * cos(M_PI /2 * t);
                qd_ddot.data[i] = -45 * D2R * M_PI/2 * M_PI/2 * sin(M_PI / 2 * t);
            }


            e_.data = qdes_.data - q_.data;
            e_dot_.data = qdes_dot_.data - q_dot_.data; // error for the joint velocity from wanted speed and joint speed
            e_int_.data = e_int_.data + e_.data * dt;


            y_.data = qdes_ddot_.data + Kd_.data.cwiseProduct(e_dot_.data) + Kp_.data.cwiseProduct(e_.data) + Ki_.data.cwiseProduct(e_int_.data);
        }
        KDL::JntArray WS_controller(KDL::Frame xdes_temp_, KDL::JntArray xdes_dot_temp_, KDL::JntArray xdes_ddot_temp_){

            KDL::Twist ex_temp_local_ = diff(xee_, xdes_temp_);
            KDL::JntArray ex_local_;
            KDL::JntArray ex_dot_local_;
            KDL::JntArray ex_int_local_;
            KDL::JntArray y_local_;
            ex_local_.data = Eigen::VectorXd::Zero(n_joints_);
            ex_dot_local_.data = Eigen::VectorXd::Zero(n_joints_);
            ex_int_local_.data = Eigen::VectorXd::Zero(n_joints_);
            y_local_.data = Eigen::VectorXd::Zero(n_joints_);

            ex_local_.data[0] = ex_temp_local_(0);
            ex_local_.data[1] = ex_temp_local_(1);
            ex_local_.data[2] = ex_temp_local_(2);
            ex_local_.data[3] = ex_temp_local_(3);
            ex_local_.data[4] = ex_temp_local_(4);
            ex_local_.data[5] = ex_temp_local_(5);

            ex_dot_local_.data = xdes_dot_temp_.data - xee_dot_.data;
            ex_int_.data = ex_int_.data + ex_local_.data * dt;

            y_local_.data =  J_inv_.data * (xdes_ddot_temp_.data + Kp_ws_.data.cwiseProduct(ex_local_.data) + Kd_ws_.data.cwiseProduct(ex_dot_local_.data) + J_dot_.data * q_dot_.data);// + Ki_ws_.data.cwiseProduct(ex_int_local_.data));
            //dance_mode();
            return y_local_;


        }

        KDL::JntArray Impedance_conterller(KDL::Frame xdes_temp_, KDL::JntArray xdes_dot_temp_, KDL::JntArray xdes_ddot_temp_){

            KDL::Twist ex_temp_local_ = diff(xee_, xdes_temp_);
            KDL::JntArray ex_local_;
            KDL::JntArray ex_dot_local_;
            KDL::JntArray ex_int_local_;
            KDL::JntArray y_local_;
            ex_local_.data = Eigen::VectorXd::Zero(n_joints_);
            ex_dot_local_.data = Eigen::VectorXd::Zero(n_joints_);
            ex_int_local_.data = Eigen::VectorXd::Zero(n_joints_);
            y_local_.data = Eigen::VectorXd::Zero(n_joints_);

            ex_local_.data[0] = ex_temp_local_(0);
            ex_local_.data[1] = ex_temp_local_(1);
            ex_local_.data[2] = ex_temp_local_(2);
            ex_local_.data[3] = ex_temp_local_(3);
            ex_local_.data[4] = ex_temp_local_(4);
            ex_local_.data[5] = ex_temp_local_(5);

            ex_dot_local_.data = xdes_dot_temp_.data - xee_dot_.data;
            ex_int_.data = ex_int_.data + ex_local_.data * dt;

            y_local_.data =  J_inv_.data * (xdes_ddot_temp_.data + Kp_ws_.data.cwiseProduct(ex_local_.data) + Kd_ws_.data.cwiseProduct(ex_dot_local_.data) + J_dot_.data * q_dot_.data);// + Ki_ws_.data.cwiseProduct(ex_int_local_.data));
            //dance_mode();
            return y_local_;


        }

        void publish_err() {
            twist_err_msgs_.linear.x = ex_temp_(0);
            twist_err_msgs_.linear.y = ex_temp_(1);
            twist_err_msgs_.linear.z = ex_temp_(2);
            twist_err_msgs_.angular.x = ex_temp_(3);
            twist_err_msgs_.angular.y = ex_temp_(4);
            twist_err_msgs_.angular.z = ex_temp_(5);
            
            twist_error_pub_.publish(twist_err_msgs_);
        }

        KDL::JntArray rep_potential_end_effector(){
            KDL::JntArray q_dot_cmd_rep;
            float min_dist = 1000000;// init to high value
            int min_obs_point = 0;
            for (int i = 0; i < obs_points_.size(); i++) {
                float dist = sqrt(pow(obs_points_.at(i)(0) - x_.p(0), 2) + pow(obs_points_.at(i)(1) - x_.p(1), 2) + pow(obs_points_.at(i)(1) - x_.p(1), 2));

                if (dist < min_dist) {
                    min_dist = dist;
                    min_obs_point = i;
                }
            } 
            float q = 0.25; 
            KDL::JntArray p;
            p.data = Eigen::VectorXd::Zero(n_joints_);


            if (min_dist > q) {
                p(0) = 0.0;
                p(1) = 0.0; 
                p(2) = 0.0;
                p(3) = 0.0; // don't read rotations 
                p(4) = 0.0;
                p(5) = 0.0;
            }

            else {
                float k = 1.0;
                p(0) = k * ((1/min_dist) - (1/q)) * (1/pow(min_dist, 2)) * ((x_.p(0) - obs_points_.at(min_obs_point)(0))/min_dist); 
                p(1) = k * ((1/min_dist) - (1/q)) * (1/pow(min_dist, 2)) * ((x_.p(1) - obs_points_.at(min_obs_point)(1))/min_dist); 
                p(2) = k * ((1/min_dist) - (1/q)) * (1/pow(min_dist, 2)) * ((x_.p(2) - obs_points_.at(min_obs_point)(2))/min_dist);
                p(3) = 0.0; // don't read rotations 
                p(4) = 0.0;
                p(5) = 0.0;

            }

            q_dot_cmd_rep.data = J_trans_.data * p.data;
            return q_dot_cmd_rep;
        }

        KDL::JntArray rep_potential_sum(KDL::Frame x, KDL::Jacobian J){

            KDL::JntArray q_dot_cmd_rep;
            float min_dist = 1000000;// init to high value
            int min_obs_point = 0;
            for (int i = 0; i < obs_points_.size(); i++) {
                float dist = sqrt(pow(obs_points_.at(i)(0) - x.p(0), 2) + pow(obs_points_.at(i)(1) - x.p(1), 2) + pow(obs_points_.at(i)(1) - x.p(1), 2));

                if (dist < min_dist) {
                    min_dist = dist;
                    min_obs_point = i;
                }
            }
            float q = 0.30; 
            KDL::JntArray p;
            p.data = Eigen::VectorXd::Zero(n_joints_);


            if (min_dist > q) {
                p(0) = 0.0;
                p(1) = 0.0; 
                p(2) = 0.0;
                p(3) = 0.0; // don't read rotations 
                p(4) = 0.0;
                p(5) = 0.0;
            }

            else {
                float k = 1.5;
                p(0) = k * ((1/min_dist) - (1/q)) * (1/pow(min_dist, 2)) * ((x.p(0) - obs_points_.at(min_obs_point)(0))/min_dist); 
                p(1) = k * ((1/min_dist) - (1/q)) * (1/pow(min_dist, 2)) * ((x.p(1) - obs_points_.at(min_obs_point)(1))/min_dist); 
                p(2) = k * ((1/min_dist) - (1/q)) * (1/pow(min_dist, 2)) * ((x.p(2) - obs_points_.at(min_obs_point)(2))/min_dist);
                p(3) = 0.0; // don't read rotations 
                p(4) = 0.0;
                p(5) = 0.0;

            }

            q_dot_cmd_rep.data = J.data.transpose() * p.data;

            return q_dot_cmd_rep;
        }

        float joint_limit_rep(int joint) {
            KDL::JntArray q_dot__lim_rep;
            float curr = q_(joint);
            float rep = 0;
            
            if (joint_lim_.at(joint) != 3.14) {
                if (joint_lim_.at(joint) - curr < 0.05) {
                    rep = 0.1 * (curr - joint_lim_.at(joint));
                }
                else if (-joint_lim_.at(joint) + curr < 0.05) {
                    rep = 0.1 * (joint_lim_.at(joint) - curr);
                }

            }
            return rep;


        }

    private:
        int print_state = 0;
        KDL::Twist Vcmd_;
        //boost::scoped_ptr<KDL::TreeFKSolverPos> fk_pos_solver_pose_;
        boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
        boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_; // solver for jacobian
        boost::scoped_ptr<KDL::ChainJntToJacDotSolver> jnt_to_jac_dot_solver_;
        boost::scoped_ptr<KDL::TreeJntToJacSolver> TreeJntToJacSolver_;
        ros::Subscriber pose_subs_;
        ros::Subscriber des_subs;
        ros::Subscriber target_pose_subs_;
        ros::Subscriber sensor_filtered;
        ros::Publisher vel_pub_;
        ros::Publisher twist_error_pub_;
        ros::Publisher ee_wrench_in_world;
        KDL::Frame x_, x_2_, x_3_, x_4_, x_5_, x_6_; // end effector frame
        std::vector<KDL::Frame> FK_vec_ {x_, x_2_, x_3_, x_4_, x_5_, x_6_};

        KDL::Frame xd_, xd_ref; // end effector frame
        actionlib::SimpleActionClient<command_msgs::planAction> *ac;
        std::vector<std::string> joint_names_;
        unsigned int n_joints_;
        command_msgs::planGoal goal;
        KDL::JntArray q_, q_dot_, qd_set_, qd_, qd_dot, qd_ddot, q_dot_cmd_, Vcmd_jnt_;
        double t = 0.0;
        KDL::Chain kdl_chain_; // chain of the kinematic tree
        KDL::Tree kdl_tree_;   // kinematic tree based of the model urdf downloaded from the parameter server
        float t_ = 0.7;
        KDL::Twist Xerr_;
        KDL::Jacobian J_, J2_, J3_, J4_, J5_, J6_;
        std::vector<KDL::Jacobian> Jac_vec_ {J_, J2_, J3_, J4_, J5_, J6_};
        std::vector<float> joint_lim_ {3.14, 2.35, 2.61, 3.14, 2.56, 3.14};
        KDL::Jacobian J_inv_;
        KDL::Jacobian J_trans_;
        KDL::Jacobian J_temp_;
        std_msgs::Float64MultiArray joint_cmds_;
        geometry_msgs::Twist twist_msgs_;
        geometry_msgs::Twist twist_err_msgs_;
        std::vector<KDL::Vector> obs_points_;
        KDL::JntArray qdes_, qdes_dot_, qdes_ddot_;

        //Mine
        double time;
        double dt;

        KDL::JntArray Kp_, Ki_, Kd_;
        KDL::JntArray Kp_ws_, Ki_ws_, Kd_ws_;
        KDL::JntArray M_t_, K_Dt_, K_Pt_;
        KDL::JntArray y_;
        KDL::JntArray e_, e_dot_, e_int_;

        KDL::Frame xdes_; // x.p: frame position(3x1), x.m: frame orientation (3x3)
        KDL::Frame xee_;
        KDL::Frame z_imp_;

        // KDL::Twist xd_dot_, xd_ddot_;
        KDL::JntArray ex_;
        KDL::JntArray xdes_dot_, xdes_ddot_;
        KDL::JntArray xee_dot_;
        KDL::JntArray ex_dot_, ex_int_;
        KDL::JntArray z_imp_dot_;
        KDL::JntArray z_imp_ddot;
        
        KDL::Twist ex_temp_;

        KDL::Jacobian J_dot_;
        KDL::JntArrayVel q_dot_jntArry;
        KDL::Wrench he_;

};

};


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