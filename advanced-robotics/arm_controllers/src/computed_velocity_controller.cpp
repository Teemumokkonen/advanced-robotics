/*
 * This file contains plugin for controlling the robot with velocity controller and kinematic controller 
 * in Joint space and task space.
 * This controller can take commands in the task and Joint space.
 * 
 * Robot is always initialized in the some position in the task space control,
 * but the the control method can be changed on the fly to be either in joint space or in task space,
 * which is defined by the trajectory planner.
*/


// from ros-control meta packages
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
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
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_kdl.h>
#include <Eigen/LU>

#include <boost/scoped_ptr.hpp>
#include <boost/lexical_cast.hpp>

#include <actionlib/server/simple_action_server.h>
#include <command_msgs/planAction.h>

#define A 0.1
#define PI 3.141592
#define D2R PI / 180.0
#define R2D 180.0 / PI
#define SaveDataMax 49
#define t_set 1
#define b 2.5
#define f 1

class CommandServer {
    public:
        // Initialize action server (Node handle, action name, action callback function)
        CommandServer(std::string name, unsigned int n_joints) :
            as_(nh_, name, boost::bind(&CommandServer::executeCB, this, _1), false),
            action_name_(name),
            n_joints_ (n_joints)
        {
            // initialize a default state to be at zero for bot task and joint space
            command_slot_ = 0;

            // Some random non singular pose for the robot in task space. 
            pose_.p(0) = 0.0;
            pose_.p(1) = 0.0;
            pose_.p(2) = 0.7; 
            pose_.M.DoRotX(0.0);
            pose_.M.DoRotY(0.0);
            pose_.M.DoRotZ(0.0);
            commands_task_space.push_back(pose_);

            qd_.data = Eigen::VectorXd::Zero(n_joints_);
            qd_dot_.data = Eigen::VectorXd::Zero(n_joints_);
            qd_ddot_.data = Eigen::VectorXd::Zero(n_joints_);
            commands_qd.push_back(qd_);
            commands_qd_dot.push_back(qd_dot_);
            commands_qd_ddot.push_back(qd_ddot_);
            as_.start();

        }

        ~CommandServer(void)
        {
        }

        void executeCB(const command_msgs::planGoalConstPtr &goal) {
            ROS_INFO("Got request from the client");
            task_space_ = goal->task_space;
            if (task_space_ == false) {
                ROS_INFO("Getting commands in joint space");
                task_spaceplan(goal);
            }
            else {
                ROS_INFO("Getting commands in task space");
                task_space_plan(goal);
            }
        
        }

        void task_space_plan(const command_msgs::planGoalConstPtr &goal) {
            // Keep prev commands
            command_slot_ = 0;
            int command_flow = 0;
            ROS_INFO("request size %li", goal->qd.size());
            if(commands_task_space.size() > 1)  
            {
                ROS_INFO("removing elements");
                commands_task_space.erase(commands_task_space.begin(), commands_task_space.end() - 1);
            }

            for (int i = 0; i < goal->qd.size(); i++) {
                // pose
                pose_.p(0) = goal->qd.at(i);
                pose_.p(1) = goal->qd_dot.at(i);
                pose_.p(2) = goal->qd_ddot.at(i); 
                // rotations     
                pose_.M = pose_.M.RPY(goal->x_rot.at(i), goal->y_rot.at(i), goal->z_rot.at(i));
                //pose_.M.DoRotX(goal->x_rot.at(i));
                //pose_.M.DoRotY(goal->y_rot.at(i));
                //pose_.M.DoRotZ(goal->z_rot.at(i));
                commands_task_space.push_back(pose_);
            }   

            ROS_INFO("command size %li", commands_qd.size());
            commands_task_space.erase(commands_task_space.begin());
            loop = goal->loop;
            ROS_INFO("Request has been parsed, plan has been set");
            result_.pose_reached = true;
            as_.setSucceeded(result_);
        }

        void task_spaceplan(const command_msgs::planGoalConstPtr &goal) {
            // Keep prev commands
            command_slot_ = 0;
            int command_flow = 0;
            ROS_INFO("request size for task space commands size %li", goal->qd.size());
            if(commands_qd.size() > 1)  
            {
                ROS_INFO("removing elements");
                commands_qd.erase(commands_qd.begin(), commands_qd.end() - 1);
                commands_qd_dot.erase(commands_qd_dot.begin(), commands_qd_dot.end() - 1);
                commands_qd_ddot.erase(commands_qd_ddot.begin(), commands_qd_ddot.end() - 1);
            }

            for (int i = 0; i < goal->qd.size() - 1; i++) {
                qd_(command_flow) = goal->qd.at(i);
                qd_dot_(command_flow) = goal->qd_dot.at(i);
                qd_ddot_(command_flow) = goal->qd_ddot.at(i);
                if (command_flow == n_joints_ - 1) {
                    commands_qd.push_back(qd_);
                    commands_qd_dot.push_back(qd_dot_);
                    commands_qd_ddot.push_back(qd_ddot_);
                    command_flow = 0;
                }
                else {
                    command_flow++;
                }
            }
            ROS_INFO("command size %li", commands_qd.size());
            commands_qd.erase(commands_qd.begin());
            commands_qd_dot.erase(commands_qd_dot.begin());
            commands_qd_ddot.erase(commands_qd_ddot.begin());
            loop = goal->loop;
            ROS_INFO("Request has been parsed, plan has been set");
            result_.pose_reached = true;
            as_.setSucceeded(result_);
        }

        void get_state(KDL::JntArray& qd, KDL::JntArray& qd_dot, KDL::JntArray& qd_ddot) {
            if (command_slot_ < commands_qd.size()) {
                qd = commands_qd.at(command_slot_);
                qd_dot = commands_qd_dot.at(command_slot_);
                qd_ddot = commands_qd_ddot.at(command_slot_);
                command_slot_++;
            }

            else {
                if (loop == true) {
                    command_slot_ = 0; 
                }
                else {
                    qd = commands_qd.at(command_slot_ - 1);
                    qd_dot = commands_qd_dot.at(command_slot_ - 1);
                    qd_ddot = commands_qd_ddot.at(command_slot_ - 1);
                }
            }
        }

        void get_ee_point(KDL::Frame& xd_){
            if (command_slot_ < commands_task_space.size()) {
                xd_ = commands_task_space.at(command_slot_);
                command_slot_++;
            }
            else {
                if (loop == true) {
                    command_slot_ = 0;
                }
                else {
                    xd_ = commands_task_space.at(command_slot_ - 1);
                }
            }
        }

        bool get_plan_type() {
            return task_space_;
        }


    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<command_msgs::planAction> as_;
        command_msgs::planFeedback feedback_;
        command_msgs::planResult result_;
        std::string action_name_; 

    private:
        int command_slot_;
        unsigned int n_joints_;         
        std::vector<float> trajectory;
        KDL::JntArray qd_, qd_dot_, qd_ddot_;
        std::vector<KDL::JntArray> commands_qd, commands_qd_dot, commands_qd_ddot;
        KDL::Frame pose_;
        std::vector<KDL::Frame> commands_task_space;
        bool loop = false;
        bool task_space_ = true;
};  

namespace arm_controllers
{
class ComputedVelocityController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
  public:
  
       bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n)
    {
        // ********* 1. Get joint name / gain from the parameter server *********
        // 1.1 Joint Name
        if (!n.getParam("joints", joint_names_))
        {
            ROS_ERROR("Could not find joint name");
            return false;
        }
        n_joints_ = joint_names_.size();

        if (n_joints_ == 0)
        {
            ROS_ERROR("List of joint names is empty.");
            return false;
        }
        else
        {
            ROS_INFO("Found %d joint names", n_joints_);
            for (int i = 0; i < n_joints_; i++)
            {
                ROS_INFO("%s", joint_names_[i].c_str());
            }
        }

        // 1.2 Gain
        // 1.2.1 Joint Controller
        J_.resize(kdl_chain_.getNrOfJoints());
        J_inv_.resize(kdl_chain_.getNrOfJoints());
        J_trans_.resize(kdl_chain_.getNrOfJoints());
        J_temp_.resize(kdl_chain_.getNrOfJoints());
        Kp_.resize(n_joints_);
        Kd_.resize(n_joints_);
        Ki_.resize(n_joints_);

        std::vector<double> Kp(n_joints_), Ki(n_joints_), Kd(n_joints_);
        for (size_t i = 0; i < n_joints_; i++)
        {
            std::string si = boost::lexical_cast<std::string>(i + 1);
            if (n.getParam("/elfin/computed_velocity_controller/gains/elfin_joint" + si + "/pid/p", Kp[i]))
            {
                Kp_(i) = Kp[i];
            }
            else
            {
                std::cout << "/elfin/computed_velocity_controller/gains/elfin_joint" + si + "/pid/p" << std::endl;
                ROS_ERROR("Cannot find pid/p gain");
                return false;
            }

            if (n.getParam("/elfin/computed_velocity_controller/gains/elfin_joint" + si + "/pid/i", Ki[i]))
            {
                Ki_(i) = Ki[i];
            }
            else
            {
                ROS_ERROR("Cannot find pid/i gain");
                return false;
            }

            if (n.getParam("/elfin/computed_velocity_controller/gains/elfin_joint" + si + "/pid/d", Kd[i]))
            {
                Kd_(i) = Kd[i];
            }
            else
            {
                ROS_ERROR("Cannot find pid/d gain");
                return false;
            }
        }

        bool frame_error;
        if (n.getParam("frame_error", frame_error))
        {
            frame_error_ = frame_error;
        } 

        else {
            ROS_ERROR("Cannot find if the task is wanted to be done neither in joint or taskpace");
            return false;
        }

        // 2. ********* urdf *********
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

        // 3. ********* Get the joint object to use in the realtime loop [Joint Handle, URDF] *********
        for (int i = 0; i < n_joints_; i++)
        {
            try
            {
                joints_.push_back(hw->getHandle(joint_names_[i]));
            }
            catch (const hardware_interface::HardwareInterfaceException &e)
            {
                ROS_ERROR_STREAM("Exception thrown: " << e.what());
                return false;
            }

            urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
            if (!joint_urdf)
            {
                ROS_ERROR("Could not find joint '%s' in urdf", joint_names_[i].c_str());
                return false;
            }
            joint_urdfs_.push_back(joint_urdf);
        }

        // 4. ********* KDL *********
        // 4.1 kdl parser
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
        if (!n.getParam("root_link", root_name))
        {
            ROS_ERROR("Could not find root link name");
            return false;
        }
        if (!n.getParam("tip_link", tip_name))
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


        // 4.3 inverse dynamics solver 초기화
        gravity_ = KDL::Vector::Zero(); // ?
        gravity_(2) = -9.81;            // 0: x-axis 1: y-axis 2: z-axis

        id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));
        inv_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
        fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

        tau_d_.data = Eigen::VectorXd::Zero(n_joints_);

        qd_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_dot_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_ddot_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_old_.data = Eigen::VectorXd::Zero(n_joints_);
        
        q_.data = Eigen::VectorXd::Zero(n_joints_);
        qdot_.data = Eigen::VectorXd::Zero(n_joints_);

        e_.data = Eigen::VectorXd::Zero(n_joints_);
        e_dot_.data = Eigen::VectorXd::Zero(n_joints_);
        e_int_.data = Eigen::VectorXd::Zero(n_joints_);
        Vcmd_jnt_.data = Eigen::VectorXd::Zero(n_joints_);

        // input initialization
        y_.data = Eigen::VectorXd::Zero(n_joints_);
        Vd_jnt_.data = Eigen::VectorXd::Zero(n_joints_);

        // 5.2 Matrix 
        M_.resize(kdl_chain_.getNrOfJoints());
        C_.resize(kdl_chain_.getNrOfJoints());
        G_.resize(kdl_chain_.getNrOfJoints());
        Jd_.resize(kdl_chain_.getNrOfJoints());
        J_.resize(kdl_chain_.getNrOfJoints());
        
        // 6.1 publisher
        pub_qd_ = n.advertise<std_msgs::Float64MultiArray>("qd", 1000);
        pub_q_ = n.advertise<std_msgs::Float64MultiArray>("q", 1000);
        pub_e_ = n.advertise<std_msgs::Float64MultiArray>("e", 1000);

        pub_SaveData_ = n.advertise<std_msgs::Float64MultiArray>("SaveData", 1000); // 뒤에 숫자는?

        // 6.2 action server

        cs_ = new CommandServer("plan/elfin", n_joints_);
        return true;
    }

    void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
    {
        if (msg->data.size() != n_joints_)
        {
            ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
            return;
        }
    }

    void starting(const ros::Time &time)
    {
        t = 0.0;
        ROS_INFO("Starting Computed Velocity Controller");
    }

    void update(const ros::Time &time, const ros::Duration &period)
    {
        // ********* 0. Get states from gazebo *********
        // 0.1 sampling time
        double dt = period.toSec();
        t = t + dt;
        // 0.2 joint state feedback from the model
        for (int i = 0; i < n_joints_; i++)
        {
            q_(i) = joints_[i].getPosition();
            qdot_(i) = joints_[i].getVelocity();
        }

        // ********* 1. Desired Trajectory in Joint Space *********
        task_space = cs_->get_plan_type();
        if (task_space == false) {
            cs_->get_state(qd_, qd_dot_, qd_ddot_);
        }
        else {
            cs_->get_ee_point(xd_);
        }

        // ********* 1. Desired Trajectory in Task space *********
        //if (task_space == false)
        //{
        //    fk_pos_solver_->JntToCart(qd_, xd_); // desired end effector pos
        //    jnt_to_jac_solver_->JntToJac(qd_, Jd_); // jacobian of the desired state
        //    Vd_jnt_.data = Jd_.data * qd_dot_.data;
        //    for (size_t i = 0; i < n_joints_; i++) {
        //        Vd_[i] = Vd_jnt_.data[i]; // populate the twist with the correct elements
        //    }
        //}

        // ********* 2.2 Kinematic controller *********
        e_.data = qd_.data - q_.data; // error for the joint states 

        if (task_space == false)
        {      
            // kinematic control command in joint space
            q_dot_cmd_.data = qd_dot_.data + Kp_.data.cwiseProduct(e_.data);
        }

        else {  
            // kinematic control command in the Task space
            fk_pos_solver_->JntToCart(q_, x_); // end effector poss 
            jnt_to_jac_solver_->JntToJac(q_, J_); // jacobian of the joint
            if (frame_error_ == true) 
            {
                Xerr_ = diff(x_, xd_) / t_; // error from the frame
            }
            else {
                Xerr_.rot = diff(x_.M, xd_.M) / t_;
                Xerr_.vel = diff(x_.p, xd_.p) / t_;
            }

            J_inv_.data = J_.data.inverse(); // inverse of the jacobian 
            J_trans_.data = J_.data.transpose();
            //Vcmd_ = Vd_ + 2.5*Xerr_;
            
            Vcmd_ = 5.0*Xerr_;
            //inv_solver_->CartToJnt(q_, Vcmd_, Vcmd_jnt_);
            for (size_t i = 0; i < n_joints_; i++) {
                Vcmd_jnt_.data[i] = Vcmd_[i];
            }

            MatrixXd I = MatrixXd::Identity(n_joints_, n_joints_);
            
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

        // ********* 2.2 Velocity controller *********

        // given of q_dot_cmd_

        e_dot_.data = q_dot_cmd_.data - qdot_.data; // error for the joint velocity from wanted speed and joint speed

        id_solver_->JntToMass(q_, M_);
        id_solver_->JntToCoriolis(q_, qdot_, C_);
        id_solver_->JntToGravity(q_, G_); 

        //y = M * (feed forward + Kd * error in velocity)
        y_.data = qd_ddot_.data + Kd_.data.cwiseProduct(e_dot_.data);

        aux_d_.data = M_.data * y_.data;
        comp_d_.data = C_.data + G_.data;
        tau_d_.data = aux_d_.data + comp_d_.data;
        
        for (int i = 0; i < n_joints_; i++)
        {
            joints_[i].setCommand(tau_d_(i)); 
        }

        //print_state();
        save_data();
    }

    void stopping(const ros::Time &time)
    {
    }

    void save_data()
    {
        // 1
        // Simulation time (unit: sec)
        SaveData_[0] = t;

        // Desired position in joint space (unit: rad)
        SaveData_[1] = qd_(0);
        SaveData_[2] = qd_(1);
        SaveData_[3] = qd_(2);
        SaveData_[4] = qd_(3);
        SaveData_[5] = qd_(4);
        SaveData_[6] = qd_(5);

        // Desired velocity in joint space (unit: rad/s)
        SaveData_[7] = qd_dot_(0);
        SaveData_[8] = qd_dot_(1);
        SaveData_[9] = qd_dot_(2);
        SaveData_[10] = qd_dot_(3);
        SaveData_[11] = qd_dot_(4);
        SaveData_[12] = qd_dot_(5);

        // Desired acceleration in joint space (unit: rad/s^2)
        SaveData_[13] = qd_ddot_(0);
        SaveData_[14] = qd_ddot_(1);
        SaveData_[15] = qd_ddot_(2);
        SaveData_[16] = qd_ddot_(3);
        SaveData_[17] = qd_ddot_(4);
        SaveData_[18] = qd_ddot_(5);

        // Actual position in joint space (unit: rad)
        SaveData_[19] = q_(0);
        SaveData_[20] = q_(1);
        SaveData_[21] = q_(2);
        SaveData_[22] = q_(3);
        SaveData_[23] = q_(4);
        SaveData_[24] = q_(5);

        // Actual velocity in joint space (unit: rad/s)
        SaveData_[25] = qdot_(0);
        SaveData_[26] = qdot_(1);
        SaveData_[27] = qdot_(2);
        SaveData_[28] = qdot_(3);
        SaveData_[29] = qdot_(4);
        SaveData_[30] = qdot_(5);

        // Error position in joint space (unit: rad)
        SaveData_[31] = e_(0);
        SaveData_[32] = e_(1);
        SaveData_[33] = e_(2);
        SaveData_[34] = e_(3);
        SaveData_[35] = e_(4);
        SaveData_[36] = e_(5);

        // Error velocity in joint space (unit: rad/s)
        SaveData_[37] = e_dot_(0);
        SaveData_[38] = e_dot_(1);
        SaveData_[39] = e_dot_(2);
        SaveData_[40] = e_dot_(3);
        SaveData_[41] = e_dot_(4);
        SaveData_[42] = e_dot_(5);

        // Error intergal value in joint space (unit: rad*sec)
        SaveData_[43] = e_int_(0);
        SaveData_[44] = e_int_(1);
        SaveData_[45] = e_int_(2);
        SaveData_[46] = e_int_(3);
        SaveData_[47] = e_int_(4);
        SaveData_[48] = e_int_(5);

        // error in the task space
        SaveData_[43] = Xerr_(0);
        SaveData_[44] = Xerr_(1);
        SaveData_[45] = Xerr_(2);
        SaveData_[46] = Xerr_(3);
        SaveData_[47] = Xerr_(4);
        SaveData_[48] = Xerr_(5);

        // 2
        msg_qd_.data.clear();
        msg_q_.data.clear();
        msg_e_.data.clear();

        msg_SaveData_.data.clear();

        // 3
        for (int i = 0; i < n_joints_; i++)
        {
            msg_qd_.data.push_back(qd_(i));
            msg_q_.data.push_back(q_(i));
            msg_e_.data.push_back(e_(i));
        }

        for (int i = 0; i < SaveDataMax; i++)
        {
            msg_SaveData_.data.push_back(SaveData_[i]);
        }

        // 4
        pub_qd_.publish(msg_qd_);
        pub_q_.publish(msg_q_);
        pub_e_.publish(msg_e_);

        pub_SaveData_.publish(msg_SaveData_);
    }

    void print_state()
    {
        static int count = 0;
        if (count > 99)
        {
            
            printf("*********************************************************\n\n");
            printf("*** Simulation Time (unit: sec)  ***\n");
            printf("t = %f\n", t);
            printf("\n");

            printf("*** Desired State in Joint Space (unit: deg) ***\n");
            printf("qd_(0): %f, ", qd_(0)*R2D);
            printf("qd_(1): %f, ", qd_(1)*R2D);
            printf("qd_(2): %f, ", qd_(2)*R2D);
            printf("qd_(3): %f, ", qd_(3)*R2D);
            printf("qd_(4): %f, ", qd_(4)*R2D);
            printf("qd_(5): %f\n", qd_(5)*R2D);
            printf("\n");

            printf("*** Actual State in Joint Space (unit: deg) ***\n");
            printf("q_(0): %f, ", q_(0) * R2D);
            printf("q_(1): %f, ", q_(1) * R2D);
            printf("q_(2): %f, ", q_(2) * R2D);
            printf("q_(3): %f, ", q_(3) * R2D);
            printf("q_(4): %f, ", q_(4) * R2D);
            printf("q_(5): %f\n", q_(5) * R2D);
            printf("\n");


            printf("*** Joint Space Error (unit: deg)  ***\n");
            printf("%f, ", R2D * e_(0));
            printf("%f, ", R2D * e_(1));
            printf("%f, ", R2D * e_(2));
            printf("%f, ", R2D * e_(3));
            printf("%f, ", R2D * e_(4));
            printf("%f\n", R2D * e_(5));
            printf("\n");


            count = 0;
        }
        count++;
    }

  private:
    // others
    double t;

    //Joint handles
    unsigned int n_joints_;                               // amount of joints
    std::vector<std::string> joint_names_;                // joint names from the urdf
    std::vector<hardware_interface::JointHandle> joints_; // hw interface for the ros control package which allows the interfacing with the joints
    std::vector<urdf::JointConstSharedPtr> joint_urdfs_;  // Joints from the urdf

    // kdl
    KDL::Tree kdl_tree_;   // kinematic tree based of the model urdf downloaded from the parameter server
    KDL::Chain kdl_chain_; // chain of the kinematic tree

    // kdl M,C,G
    KDL::JntSpaceInertiaMatrix M_; // intertia matrix
    KDL::JntArray C_;              // coriolis
    KDL::JntArray G_;              // gravity torque vector
    KDL::Vector gravity_;
    KDL::Jacobian J_;
    KDL::Jacobian Jd_;
    KDL::Jacobian J_inv_;
    KDL::Jacobian J_trans_;
    KDL::Jacobian J_temp_;

    // kdl solver
    boost::scoped_ptr<KDL::ChainDynParam> id_solver_;                  // Solver To compute the inverse dynamics
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_; // solver for jacobian
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
    boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> inv_solver_;

    // Joint Space State
    KDL::JntArray qd_, qd_dot_, qd_ddot_, q_dot_cmd_;
    KDL::JntArray qd_old_;
    KDL::JntArray q_, qdot_;
    KDL::JntArray e_, e_dot_, e_int_;

    KDL::JntArray Vd_jnt_;

    KDL::JntArray y_;
    KDL::JntArray Vcmd_jnt_;
    // Input
    KDL::JntArray aux_d_;
    KDL::JntArray comp_d_;
    KDL::JntArray tau_d_;

    // frames
    KDL::Frame x_; // end effector frame
    KDL::Frame xd_; // end effector desired frame
    
    // twists
    KDL::Twist Xerr_;
    KDL::Twist Vd_;
    KDL::Twist Vcmd_;

    // gains
    KDL::JntArray Kp_, Ki_, Kd_;

    // save the data
    double SaveData_[SaveDataMax];

    // ros publisher
    ros::Publisher pub_qd_, pub_q_, pub_e_;
    ros::Publisher pub_SaveData_;

    // ros message
    std_msgs::Float64MultiArray msg_qd_, msg_q_, msg_e_;
    std_msgs::Float64MultiArray msg_SaveData_;

    // params for using different controller types and calculations
    bool task_space;
    bool frame_error_;

    CommandServer* cs_;

    float t_ = 1.0;
};
}; // namespace arm_controllers
PLUGINLIB_EXPORT_CLASS(arm_controllers::ComputedVelocityController, controller_interface::ControllerBase)