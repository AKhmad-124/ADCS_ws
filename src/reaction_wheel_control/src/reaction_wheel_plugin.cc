
#include <gazebo/gazebo.hh>                       // for accessing all gazebo classes
#include <gazebo/common/common.hh>                // for common fn in gazebo like ModelPlugin, event
#include <gazebo/physics/physics.hh>              // for gazebo physics, to access -- ModelPtr
#include <ignition/math/Vector3.hh>               // to access Vector3d() from ignition math class
#include "rclcpp/rclcpp.hpp"                      // ros clisent library c++   
#include "std_msgs/msg/float32_multi_array.hpp"   // ros 2 msg tpye array of floats

namespace gazebo {
class RecWheelControllerPlugin : public ModelPlugin {

public:
int count = 0;  // Initialize count

private://*variables used



  double vel;
  event::ConnectionPtr updateConnection;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
  bool activate_move_ = false;
  std::thread executor_thread_; // Store executor thread

double ang_vel_xy_ =0.0;
double ang_vel_yz_ =0.0;
double ang_vel_xz_ =0.0;

std::string rec_xy_joint ;  // to store joint name
std::string rec_yz_joint ;  // to store joint name
std::string rec_xz_joint ;  // to store joint name

physics::ModelPtr model;  // Pointer to the model
physics::JointPtr xy_joint;  // pointer to the joint
physics::JointPtr yz_joint;  // pointer to the joint
physics::JointPtr xz_joint;  // pointer to the joint
  
    
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    
    // Store the pointer to the model
    this->model = _model;


    //* getting joints from the world 
    if (_sdf->HasElement("rec_xy_joint")) // check if element existence 
    {this->rec_xy_joint = _sdf->Get<std::string>("rec_xy_joint");}  // use _sdf pointer & Get to find value in <model_vel>
    else//if joint name not provided
    {this->rec_xy_joint = "rec_wheel_xy_joint";//default it to this
    std::cout<< "no joint provided for xy defaulting to 'rec_wheel_xy_joint'" << std::endl;}//say it wasn't provided
    
    if (_sdf->HasElement("rec_yz_joint")) // check if element existence 
    {this->rec_yz_joint = _sdf->Get<std::string>("rec_yz_joint");}  // use _sdf pointer & Get to find value in <model_vel>
    else//if joint name not provided
    {this->rec_yz_joint = "rec_wheel_yz_joint";//default it to this
    std::cout<< "no joint provided for yz defaulting to 'rec_wheel_yz_joint'" << std::endl;}//say it wasn't provided
    
    if (_sdf->HasElement("rec_xz_joint")) // check if element existence 
    {this->rec_xz_joint = _sdf->Get<std::string>("rec_xz_joint");}  // use _sdf pointer & Get to find value in <model_vel>
    else//if joint name not provided
    {this->rec_xz_joint = "rec_wheel_xz_joint";//default it to this
    std::cout<< "no joint provided for xz defaulting to 'rec_wheel_xz_joint'" << std::endl;}//say it wasn't provided
    

      //*assigning the joint names and printing the debugs
    this->xy_joint = _model->GetJoint(this->rec_xy_joint); // by name
    this->yz_joint = _model->GetJoint(this->rec_yz_joint);
    this->xz_joint = _model->GetJoint(this->rec_xz_joint); 
    
    std::cout<< "Joint By Name   =" << this->xy_joint->GetScopedName() << std::endl;
    std::cout<< "Joint By Name   =" << this->yz_joint->GetScopedName() << std::endl;
    std::cout<< "Joint By Name   =" << this->xz_joint->GetScopedName() << std::endl;
    
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&RecWheelControllerPlugin::OnUpdate, this));

    // bind() is use to bind this & OnUpdate i.e this->OnUpdate
    //bind- we don't have define callback fn input parametes its replace by placeholder



    if (!rclcpp::ok()){rclcpp::init(0,nullptr);};//if ros2 isn't init. init it  
    this->node_ = rclcpp::Node::make_shared("ros_reaction_wheel_plugin");//node init
    

    this->sub_ = this->node_->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/rec_wheel_vel",//topic name
      10,//topic queue size
      std::bind(&RecWheelControllerPlugin::ActivateCallback, this, std::placeholders::_1)
      // binds(like a hotkey) the function above to a callback function then when this fucntion is called it is back XD
      //!pay attentoin lil nigga (catch me later)
    );

    //* Start ROS 2 executor in a separate thread
    this->executor_thread_ = std::thread([this]() {
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(this->node_);
      executor.spin();
    });
    
    // Connect to Gazebo update event
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&RecWheelControllerPlugin::OnUpdate, this)//!red thing above's brother
    );



  }


  //keep on updating as simulation iterates 
public:
void ActivateCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  RCLCPP_INFO(this->node_->get_logger(), "Received Message: %d", msg->data);
  this->ang_vel_xy_ = msg->data[0];
  this->ang_vel_yz_ = msg->data[1];
  this->ang_vel_xz_ = msg->data[2];

}


  void OnUpdate() {
    //method 2 
    this->xy_joint->SetParam("fmax", 0, 1.0);//?("property_name",axis_index,value)
    this->xy_joint->SetParam("vel", 0, this->ang_vel_xy_);
    this->yz_joint->SetParam("fmax", 0, 1.0);
    this->yz_joint->SetParam("vel", 0, this->ang_vel_yz_);
    this->xz_joint->SetParam("fmax", 0, 1.0);
    this->xz_joint->SetParam("vel", 0, this->ang_vel_xz_);


    //method 3    
    // if (count == 0)
    // {
    //  this->jointController.reset(new physics::JointController(this->model));  //reset joint controller
    //  this->jointController->AddJoint(this->model->GetJoint("joint_1"));  //add a joint to control
    //  std::string name = this->model->GetJoint("joint_1")->GetScopedName(); //get full joint name
    //  this->jointController->SetVelocityPID(name, common::PID(1, 0, 0)); //set pid for the joint
    //  this->jointController->SetVelocityTarget(name, 0.1);   // set target velocity for the joint

    //  this->jointController->Update();
    // }


    count++;
  }






};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(RecWheelControllerPlugin)
} 