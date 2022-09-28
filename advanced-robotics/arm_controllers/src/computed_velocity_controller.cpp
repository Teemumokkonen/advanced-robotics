// from ros-control meta packages
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>

#include <urdf/model.h>

// from kdl packages
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>              // inverse dynamics
#include <kdl/chainjnttojacsolver.hpp> 
#include <kdl/chainfksolverpos_recursive.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <Eigen/LU>
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>

#define A 0.1
#define PI 3.141592
#define D2R PI / 180.0
#define R2D 180.0 / PI
#define SaveDataMax 49
#define t_set 1
#define b 2.5
#define f 1

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

        bool joint_space;
        if (n.getParam("Joint_space", joint_space))
        {
            joint_space_ = joint_space;
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

        // 6.2 subsriber

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

        for (size_t i = 0; i < n_joints_; i++)
        {
            qd_ddot_(i) = -M_PI * M_PI / 4 * 45 * KDL::deg2rad * sin(M_PI / 2 * t); // desired acceleration for the controller
            qd_dot_(i) = M_PI / 2 * 45 * KDL::deg2rad * cos(M_PI / 2 * t); // this value is the desired velocity to match with the controller
            qd_(i) = 45 * KDL::deg2rad * sin(M_PI / 2* t); // desired position for each joint
        }

        // ********* 1. Desired Trajectory in Task space *********

        if (joint_space_ == false)
        {
            fk_pos_solver_->JntToCart(qd_, xd_); // desired end effector pos
            jnt_to_jac_solver_->JntToJac(qd_, Jd_); // jacobian of the desired state
            Vd_jnt_.data = Jd_.data * qd_dot_.data;
            for (size_t i = 0; i < n_joints_; i++) {
                Vd_[i] = Vd_jnt_.data[i]; // populate the twist with the correct elements
            }
        }

        // ********* 2.2 Kinematic controller *********

        if (joint_space_ == true)
        {      
            // kinematic control command in joint space
            e_.data = qd_.data - q_.data; // error for the joint states 
            q_dot_cmd_.data = qd_dot_.data + Kp_.data.cwiseProduct(e_.data);
        }
        else {  
            // kinematic control command in the Task space
            fk_pos_solver_->JntToCart(q_, x_); // end effector poss 
            jnt_to_jac_solver_->JntToJac(q_, J_); // jacobian of the joint
            if (frame_error_ == true) 
            {
                Xerr_ = diff(x_, xd_); // error from the frame
            }
            else {
                Xerr_.rot = diff(x_.M, xd_.M);
                Xerr_.vel = diff(x_.p, xd_.p);
            }

    
            J_inv_.data = J_.data.inverse(); // inverse of the jacobian 
            Vcmd_ = Vd_ + 1.0*Xerr_;
            for (size_t i = 0; i < n_joints_; i++) {
                Vcmd_jnt_.data[i] = Vcmd_[i];
            }
            q_dot_cmd_.data = J_inv_.data * Vcmd_jnt_.data;
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

        print_state();
    }

    void stopping(const ros::Time &time)
    {
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

    // kdl solver
    boost::scoped_ptr<KDL::ChainDynParam> id_solver_;                  // Solver To compute the inverse dynamics
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_; // solver for jacobian
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;

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
    bool joint_space_;
    bool frame_error_;
};
}; // namespace arm_controllers
PLUGINLIB_EXPORT_CLASS(arm_controllers::ComputedVelocityController, controller_interface::ControllerBase)