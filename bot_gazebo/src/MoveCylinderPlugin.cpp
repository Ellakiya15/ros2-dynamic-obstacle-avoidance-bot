#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Pose3.hh>

namespace gazebo
{
class MoveCylinderPlugin : public ModelPlugin
{
public:
    void Load(physics::ModelPtr model, sdf::ElementPtr /*sdf*/) override
    {

        // Store the pointer to the model
        this->model = model;
        this->model->GetLink("link")->SetGravityMode(false);
        // Initialize parameters
        this->radius = 1.5;  // Circular path radius
        this->speed = 0.001;   // Movement speed (rad/s)

        // Connect to the world update event (called at every simulation iteration)
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&MoveCylinderPlugin::OnUpdate, this));
    }

    void OnUpdate()
    {
        // Compute the new position based on time
        double currentTime = this->model->GetWorld()->SimTime().Double();
        double x = this->radius * cos(this->speed * currentTime);
        double y = this->radius * sin(this->speed * currentTime);

        // Set the new pose for the cylinder
        ignition::math::Pose3d newPose(x, y, 0.5, 0, 0, 0);  // z = 0.5 to keep it above ground
        this->model->SetWorldPose(newPose);
    }

private:
    physics::ModelPtr model;                 // Pointer to the model
    event::ConnectionPtr updateConnection;   // Pointer to the update event connection
    double radius;                           // Radius of the circular motion
    double speed;                            // Speed of the movement
};

// Register this plugin with Gazebo
GZ_REGISTER_MODEL_PLUGIN(MoveCylinderPlugin)
}
