#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <iostream>

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

      // Listen to the update event. This event is broadcast every simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&WavePlugin::OnUpdate, this));
    }

    // Called by the world update start event
void OnUpdate()
{
  // Get the current simulation time
  common::Time simTime = this->model->GetWorld()->SimTime();

  // Calculate the vertical displacement based on a sine wave
//  double amplitudeZ = 0.3;   // Adjust the amplitude of the vertical displacement
//   double frequencyZ = 0.6;   // Adjust the frequency of the vertical displacement
// double displacementZ = amplitudeZ * std::sin(frequencyZ * simTime.Double());

  // Calculate the sideways rocking based on another sine wave (adjust the frequency and amplitude as needed)
  // double rockingAmplitude = 0.2;  // Adjust the amplitude of the rocking motion
  // double rockingFrequency = 0.2;  // Adjust the frequency of the rocking motion
  // double displacementY = rockingAmplitude * std::sin(rockingFrequency * simTime.Double());

  // Random perturbation for added realism
  double randomness = (std::rand() % 1000 - 500) / 1000.0;
  
  // Calculate the new position of the model
  // ignition::math::Vector3d newPosition = this->initialPose.Pos() + ignition::math::Vector3d(0, displacementY, displacementZ);
  // // Calculate the new position of the model along the Z-axis

  // // Create a new Pose3d object with the calculated position and the initial rotation
  // ignition::math::Pose3d newPose(newPosition, this->initialPose.Rot());

  // // Set the new pose of the model
  // this->model->SetWorldPose(newPose);

  // Calculate the pitch and roll angles based on sine waves
  double rollAmplitude = 0.02;  // Adjust xthe amplitude of the pitch motion (0.02)
  double rollFrequency = 0.7;  // Adjust the frequency of the pitch motion
  double rollAngle = rollAmplitude * std::sin(rollFrequency * simTime.Double());
  double yawAmplitude = 0.005;   // Adjust the amplitude of the roll motion
  double yawFrequency = 0.7;   // Adjust the frequency of the roll motion
  double yawAngle = yawAmplitude * std::sin(yawFrequency * simTime.Double());
  double pitchAmplitude = 0.03;   // Adjust the amplitude of the pitch motion
  double pitchFrequency = 0.7;   // Adjust the frequency of the pitch motion
  double pitchAngle = pitchAmplitude * std::sin(pitchFrequency * simTime.Double());
  // double pitchAngle = 0.03,
  //yawAmplitude = 0.005

  // Print the angles
std::cout << "Roll Angle: " << rollAngle << " radians" << std::endl;
std::cout << "Yaw Angle: " << yawAngle << " radians" << std::endl;
std::cout << "Pitch Angle: " << pitchAngle << " radians" << std::endl;
  // Include negative pitch angles by multiplying with a sine function with a different phase
  // double negativePitchAngle = -pitchAmplitude * std::sin(pitchFrequency * simTime.Double() + IGN_PI);

  // // Combine positive and negative pitch angles
  // pitchAngle += negativePitchAngle;

  // Calculate the new orientation of the model (Assume yaw=0)
      // ignition::math::Quaterniond newOrientation =
      // ignition::math::Quaterniond(ignition::math::Vector3d(1, 0, 0), rollAngle) *
      // ignition::math::Quaterniond(ignition::math::Vector3d(0, 1, 0), pitchAngle) *
      // ignition::math::Quaterniond(ignition::math::Vector3d(0, 0, 1), yawAngle);
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

  private:
    // Pointer to the model
    physics::ModelPtr model;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

     // Initial pose of the model
    ignition::math::Pose3d initialPose;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(WavePlugin)
}  // namespace gazebo
