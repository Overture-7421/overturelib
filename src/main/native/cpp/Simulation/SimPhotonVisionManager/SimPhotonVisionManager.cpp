#include "OvertureLib/Simulation/SimPhotonVisionManager/SimPhotonVisionManager.h"
#include <networktables/NetworkTableInstance.h>

SimPhotonVisionManager::SimPhotonVisionManager() {
    simulatedDriveTrainPoseEntry = nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("/AdvantageKit/RealOutputs/FieldSimulation/RobotPosition").Subscribe({});
    std::cout << "SimPhotonVisionManager initialized!!!" << std::endl;
}

void SimPhotonVisionManager::Init(frc::AprilTagFieldLayout tagLayout) {
    visionSim.AddAprilTags(tagLayout);
}

void SimPhotonVisionManager::Update() {
    if(!simulatedDriveTrainPoseEntry.Exists()) {
        std::cerr << "Simulated drive train pose entry does not exist! Cannot update simulated cameras" << std::endl;
    }
    const frc::Pose2d &pose = simulatedDriveTrainPoseEntry.Get();
    visionSim.Update(pose);
}

void SimPhotonVisionManager::AddSimCamera(photon::PhotonCameraSim* camera, const frc::Transform3d& robotToCamera) {
    visionSim.AddCamera(camera, robotToCamera);
    std::cout << "Added camera " << camera->GetCamera()->GetCameraName() <<" to SimPhotonVisionManager" << std::endl;
}

frc::Pose3d SimPhotonVisionManager::GetRobotPose() {
    return visionSim.GetRobotPose();
}

SimPhotonVisionManager& SimPhotonVisionManager::GetInstance() {
	static SimPhotonVisionManager instance;
	return instance;
}
