#include "OvertureLib/Subsystems/Vision/AprilTags/MovingCameraAprilTags.h"

MovingCameraAprilTags::MovingCameraAprilTags(
		frc::AprilTagFieldLayout *tagLayout, SwerveChassis *chassis,
		std::function<frc::Transform3d()> cameraPoseSupplier, Config config) : AprilTags(
		tagLayout, chassis, std::move(config)), cameraPoseSupplier(
		std::move(cameraPoseSupplier)) {
}

std::optional<photon::EstimatedRobotPose> MovingCameraAprilTags::getEstimatedPose(
		const photon::PhotonPipelineResult &result) {
	poseEstimator->SetRobotToCameraTransform(cameraPoseSupplier());
	return AprilTags::getEstimatedPose(result);
}

