#pragma once

#include "AprilTags.h"

class MovingCameraAprilTags: public AprilTags {
public:
	/// @brief Construct an AprilTag instance that is able to use a moving camera. (e.g a camera in a turret)
	/// @param tagLayout The april tag field layout
	/// @param chassis The swerve chassis to add vision measurements to
	/// @param cameraPoseSupplier Supplies the current camera-to-robot transform
	/// @param config Provides camera name and std dev values for cameras. @ref AprilTags::Config::cameraToRobot "cameraToRobot" is ignored as it should be provided by cameraPoseSupplier
	MovingCameraAprilTags(frc::AprilTagFieldLayout *tagLayout,
			SwerveChassis *chassis,
			std::function<frc::Transform3d()> cameraPoseSupplier,
			Config config);

	std::optional<photon::EstimatedRobotPose> getEstimatedPose(
			const photon::PhotonPipelineResult &result) override;

private:
	std::function<frc::Transform3d()> cameraPoseSupplier;
};
