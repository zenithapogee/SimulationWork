#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <thread>
#include <ros2_interfaces/msg/point.hpp>



namespace gazebo
{
  class WavePlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _model;

      // Get the initial pose of the model
      this->initialPose = this->model->WorldPose();

      // Set initial values
      this->rollAmplitude = 0.05;  // Default values
      this->pitchAmplitude = 0;
      this->yawAmplitude = 0;

      this->rollFrequency = 2.0;
      this->pitchFrequency = 2.0;
      this->yawFrequency = 0.2;

      // Create a ROS 2 node
      this->rosNode = std::make_shared<rclcpp::Node>("wave_plugin_node");

      // Create ROS 2 subscribers for roll, pitch, and yaw amplitudes and frequencies
      this->rollSub = this->rosNode->create_subscription<std_msgs::msg::Float64>(
          "wave_plugin/roll_amplitude_cmd", 10,
          std::bind(&WavePlugin::OnRollAmplitudeCmd, this, std::placeholders::_1));

      this->pitchSub = this->rosNode->create_subscription<std_msgs::msg::Float64>(
          "wave_plugin/pitch_amplitude_cmd", 10,
          std::bind(&WavePlugin::OnPitchAmplitudeCmd, this, std::placeholders::_1));

      this->yawSub = this->rosNode->create_subscription<std_msgs::msg::Float64>(
          "wave_plugin/yaw_amplitude_cmd", 10,
          std::bind(&WavePlugin::OnYawAmplitudeCmd, this, std::placeholders::_1));

      this->rollfreq = this->rosNode->create_subscription<std_msgs::msg::Float64>(
          "wave_plugin/roll_frequency_cmd", 10,
          std::bind(&WavePlugin::OnRollFrequencyCmd, this, std::placeholders::_1));

      this->pitchfreq = this->rosNode->create_subscription<std_msgs::msg::Float64>(
          "wave_plugin/pitch_frequency_cmd", 10,
          std::bind(&WavePlugin::OnPitchFrequencyCmd, this, std::placeholders::_1));

      this->yawfreq = this->rosNode->create_subscription<std_msgs::msg::Float64>(
          "wave_plugin/yaw_frequency_cmd", 10,
          std::bind(&WavePlugin::OnYawFrequencyCmd, this, std::placeholders::_1));

      // Create ROS 2 publisher to fsm_command
      this->pos_pub = this->rosNode->create_publisher<ros2_interfaces::msg::Point>(
          "fsm_command", 1);

      // Start a separate thread for spinning the ROS 2 node
    rosSpinThread = std::thread([this]() 
    {
        rclcpp::spin(this->rosNode);
    });

      // Listen to the update event. This event is broadcast every simulation iteration
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&WavePlugin::OnUpdate, this));
    }


    // Called by the world update start event
    void OnUpdate()
    {
      // Get the current simulation time
      common::Time simTime = this->model->GetWorld()->SimTime();

      // Calculate the new orientation of the model 
      float rollAngle = rollAmplitude * std::sin(rollFrequency * simTime.Double());
      float pitchAngle = pitchAmplitude * std::sin(pitchFrequency * simTime.Double());
      float yawAngle = yawAmplitude * std::sin(yawFrequency * simTime.Double());

      // Calculate the new orientation of the model
      ignition::math::Quaterniond newOrientation =
          ignition::math::Quaterniond(ignition::math::Vector3d(1, 0, 0), pitchAngle) *
          ignition::math::Quaterniond(ignition::math::Vector3d(0, 1, 0), rollAngle) *
          ignition::math::Quaterniond(ignition::math::Vector3d(0, 0, 1), yawAngle);

      // Calculate the new position of the model
      ignition::math::Vector3d newPosition = this->initialPose.Pos();

      // Create a new Pose3d object with the calculated position and orientation
      ignition::math::Pose3d newPose(newPosition, newOrientation);

      // Set the new pose of the model
      this->model->SetWorldPose(newPose);

      // Call OnFSMCommand to publish the FSM command with the updated angles
      OnFSMCommand(rollAngle,yawAngle);
    }

    // Handle incoming roll amplitude command
    void OnRollAmplitudeCmd(const std_msgs::msg::Float64::SharedPtr _msg)
    {
      // Set the roll amplitude from the received message
      rollAmplitude = _msg->data;

      // Print the received roll amplitude
      std::cout << "Received New Roll Amplitude: " << rollAmplitude << std::endl;
    }

    // Handle incoming pitch amplitude command
    void OnPitchAmplitudeCmd(const std_msgs::msg::Float64::SharedPtr _msg)
    {
      // Set the pitch amplitude from the received message
      pitchAmplitude = _msg->data;

      // Print the received pitch amplitude
      std::cout << "Received New Pitch Amplitude: " << pitchAmplitude << std::endl;
    }

    // Handle incoming yaw amplitude command
    void OnYawAmplitudeCmd(const std_msgs::msg::Float64::SharedPtr _msg)
    {
      // Set the yaw amplitude from the received message
      yawAmplitude = _msg->data;

      // Print the received yaw amplitude
      std::cout << "Received New Yaw Amplitude: " << yawAmplitude << std::endl;
    }

    void OnRollFrequencyCmd(const std_msgs::msg::Float64::SharedPtr _msg)
    {
      // Set the roll frequency from the received message
      rollFrequency = _msg->data;

      // Print the received roll frequency
      std::cout << "Received New Roll Frequency: " << rollFrequency << std::endl;
    }

    void OnPitchFrequencyCmd(const std_msgs::msg::Float64::SharedPtr _msg)
    {
      // Set the pitch freqeuncy from the received message
      pitchFrequency = _msg->data;

      // Print the received pitch frequency
      std::cout << "Received New Pitch Frequency: " << pitchFrequency << std::endl;
    }


    void OnYawFrequencyCmd(const std_msgs::msg::Float64::SharedPtr _msg)
    {
      // Set the pitch freqeuncy from the received message
      yawFrequency = _msg->data;

      // Print the received yaw frequency
      std::cout << "Received New Yaw Frequency: " << yawFrequency << std::endl;
    }


    void OnFSMCommand(float roll, float yaw)    
    {
      ros2_interfaces::msg::Point pub_str;
      pub_str.point[1] = 0;


      if (roll * 20 >= 1) {
      pub_str.point[0] = 1;
      } 
      else if (roll * 20 <= -1) {
          pub_str.point[0] = -1;
      } 
      else {
          pub_str.point[0] = roll * 20;
      }

      if (yaw * 20 >= 1) {
      pub_str.point[1] = 1;
      } 
      else if (yaw * 20 <= -1) {
          pub_str.point[1] = -1;
      } 
      else {
          pub_str.point[1] = yaw * 20;
      }
      
      //comment this line below when laser plugin is subscribing to fsm command
      // pos_pub->publish(pub_str);
    }


  private:
    // Pointer to the model
    physics::ModelPtr model;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    // Initial pose of the model
    ignition::math::Pose3d initialPose;

    // ROS 2 node
    rclcpp::Node::SharedPtr rosNode;

    // ROS 2 subscribers for roll, pitch, and yaw amplitudes
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rollSub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pitchSub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yawSub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rollfreq;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pitchfreq;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yawfreq;
    rclcpp::Publisher<ros2_interfaces::msg::Point>::SharedPtr pos_pub;

    double rollAmplitude; 
    double pitchAmplitude;
    double yawAmplitude;
    double rollFrequency;
    double pitchFrequency;
    double yawFrequency;

    // Roll and pitch angles
    float rollAngle;
    float pitchAngle;

    // Thread for spinning the ROS 2 node
    std::thread rosSpinThread;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(WavePlugin)
}  

