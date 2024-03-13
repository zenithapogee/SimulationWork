#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <thread>
#include <ros2_interfaces/msg/point.hpp>

namespace gazebo
{
  class LaserPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _model;

      // Get the initial pose of the model
      this->initialPose = this->model->WorldPose();

      // Create a ROS 2 node
      this->rosNode = std::make_shared<rclcpp::Node>("laser_node");

      // Create ROS 2 subscribers for roll, pitch, and yaw angles
      this->rollangleSub = this->rosNode->create_subscription<std_msgs::msg::Float64>(
          "laser_plugin/roll_angle_cmd", 10,
          std::bind(&LaserPlugin::OnRollAngleCmd, this, std::placeholders::_1));

      this->pitchangleSub = this->rosNode->create_subscription<std_msgs::msg::Float64>(
          "laser_plugin/pitch_angle_cmd", 10,
          std::bind(&LaserPlugin::OnPitchAngleCmd, this, std::placeholders::_1));

      this->yawangleSub = this->rosNode->create_subscription<std_msgs::msg::Float64>(
          "laser_plugin/yaw_angle_cmd", 10,
          std::bind(&LaserPlugin::OnYawAngleCmd, this, std::placeholders::_1));

      // Create ROS 2 subscriber for fsm_command topic 
      this->fsmCommandSub = this->rosNode->create_subscription<ros2_interfaces::msg::Point>(
          "fsm_command", 10,
          std::bind(&LaserPlugin::OnFSMCommand, this, std::placeholders::_1));

      // Listen to the pose update event
      this->poseUpdateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&LaserPlugin::OnPoseUpdate, this));

      // Start a separate thread for spinning the ROS 2 node
      rosSpinThread = std::thread([this]() {
          rclcpp::spin(this->rosNode);
      });
    }

    // Called when the pose of the model is updated
    void OnPoseUpdate()
    {
      // Calculate the new orientation of the model
      ignition::math::Quaterniond newOrientation =
          ignition::math::Quaterniond(ignition::math::Vector3d(1, 0, 0), rollAngle) *
          ignition::math::Quaterniond(ignition::math::Vector3d(0, 1, 0), pitchAngle) *
          ignition::math::Quaterniond(ignition::math::Vector3d(0, 0, 1), yawAngle);

      // Calculate the new position of the model
      ignition::math::Vector3d newPosition = this->initialPose.Pos();

      // Create a new Pose3d object with the calculated position and orientation
      ignition::math::Pose3d newPose(newPosition, newOrientation);

      // Set the new pose of the model
      this->model->SetWorldPose(newPose);
    }

    // Handle incoming roll angle command
    void OnRollAngleCmd(const std_msgs::msg::Float64::SharedPtr _msg)
    {
      // Set the roll angle from the received message
      rollAngle = _msg->data;

      // Print the received roll angle
      std::cout << "Received New Roll Angle: " << rollAngle << std::endl;
    }

    // Handle incoming pitch angle command
    void OnPitchAngleCmd(const std_msgs::msg::Float64::SharedPtr _msg)
    {
      // Set the pitch angle from the received message
      pitchAngle = _msg->data;

      // Print the received pitch angle
      std::cout << "Received New Pitch Angle: " << pitchAngle << std::endl;
    }

    // Handle incoming yaw angle command
    void OnYawAngleCmd(const std_msgs::msg::Float64::SharedPtr _msg)
    {
      // Set the yaw angle from the received message
      yawAngle = _msg->data;

      // Print the received yaw angle
      std::cout << "Received New Yaw Angle: " << yawAngle << std::endl;
    }

     // Handle incoming fsm command
    void OnFSMCommand(const ros2_interfaces::msg::Point::SharedPtr _msg)
    {
      //Uncomment to have FSM command control the laser orientation in simulation
      // rollAngle = _msg->point[0];
      // pitchAngle = _msg->point[1] /10.0;
      //Pitch 0.1-0.2 range
      // Print the received values
      //std::cout << "Received New X Value: " << rollAngle << std::endl;
      //std::cout << "Received New Y Value: " << pitchAngle << std::endl;
    }
    
  private:
    // Pointer to the model
    physics::ModelPtr model;

    // Initial pose of the model
    ignition::math::Pose3d initialPose;

    // ROS 2 node
    rclcpp::Node::SharedPtr rosNode;

    // ROS 2 subscribers for roll, pitch, and yaw angles and FSM command
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rollangleSub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pitchangleSub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yawangleSub;
    rclcpp::Subscription<ros2_interfaces::msg::Point>::SharedPtr fsmCommandSub;


    // Connection to the pose update event
    event::ConnectionPtr poseUpdateConnection;

    double rollAngle;  
    double pitchAngle;
    double yawAngle;

    // Thread for spinning the ROS 2 node
    std::thread rosSpinThread;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(LaserPlugin)
}  
