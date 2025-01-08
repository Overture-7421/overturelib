#pragma once

#include <photon/PhotonCamera.h>
#include <photon/simulation/VisionSystemSim.h>

class SimPhotonVisionManager {
public:
	SimPhotonVisionManager();

	/***
	 * Ideal tag layout, not the one we measured.
	 */
	void Init(frc::AprilTagFieldLayout tagLayout);
	void Update();

	void AddSimCamera(photon::PhotonCameraSim *cameraSim,
			const frc::Transform3d &robotToCamera);

	static SimPhotonVisionManager& GetInstance();
	SimPhotonVisionManager(SimPhotonVisionManager const&) = delete;
	void operator=(SimPhotonVisionManager const&) = delete;

	frc::Pose3d GetRobotPose();
private:

	photon::VisionSystemSim visionSim { "main" };
	nt::StructSubscriber<frc::Pose2d> simulatedDriveTrainPoseEntry;
};
