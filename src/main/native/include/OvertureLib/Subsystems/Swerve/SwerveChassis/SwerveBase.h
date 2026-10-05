#pragma once
#include <frc/filter/SlewRateLimiter.h>
#include <OvertureLib/Sensors/OverPigeon/OverPigeon.h>
#include <OvertureLib/Subsystems/Swerve/SwerveModule/SwerveModule.h>
#include <OvertureLib/Math/ChassisAccels.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/DriverStation.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructTopic.h>
#include <units/length.h>
#include <units/velocity.h>
#include <wpi/array.h>
#include <memory>

// spotless:off
// Path following is not wired here. Whether a robot follows paths with PathPlanner, BLine or
// something else is the robot project's call, and all any of them need is already public:
// getEstimatedPose, resetPose, getCurrentSpeeds and setTargetSpeeds.
class SwerveBase {
public:
	virtual const frc::Pose2d& getEstimatedPose() = 0;
	virtual void resetOdometry(frc::Pose2d initPose) = 0;
	virtual frc::ChassisSpeeds getCurrentSpeeds() = 0;
	virtual void setTargetSpeeds(frc::ChassisSpeeds speeds) = 0;

	virtual units::meters_per_second_t getMaxModuleSpeed() = 0;
	virtual units::meter_t getDriveBaseRadius() = 0;

	virtual frc::Rotation2d getRotation2d() = 0;
	virtual frc::Rotation3d getRotation3d() = 0;

	// Places the robot at a known pose. This is the reset to hand a path follower: it also tells
	// the simulator where the robot was put, on the topic it listens to. resetOdometry only
	// corrects the estimate.
	void resetPose(frc::Pose2d pose) {
		resetOdometryPosePublisher.Set(pose);
		resetOdometry(pose);
	}

protected:
	void configureSwerveBase() {
		configuredChassis = true;

		odometry = std::make_unique<frc::SwerveDrivePoseEstimator<4>>(getKinematics(), frc::Rotation2d{}, modulesPositions, frc::Pose2d{});
	}

	virtual SwerveModule& getFrontLeftModule() = 0;
	virtual SwerveModule& getFrontRightModule() = 0;
	virtual SwerveModule& getBackLeftModule() = 0;
	virtual SwerveModule& getBackRightModule() = 0;

	virtual frc::SwerveDriveKinematics<4>& getKinematics() = 0;

	bool configuredChassis = false;

	std::unique_ptr<frc::SwerveDrivePoseEstimator<4>> odometry;
	wpi::array<frc::SwerveModulePosition, 4U> modulesPositions{
			frc::SwerveModulePosition(), frc::SwerveModulePosition(),
			frc::SwerveModulePosition(), frc::SwerveModulePosition() };
	wpi::array<frc::SwerveModuleState, 4U> modulesStates{
			frc::SwerveModuleState(), frc::SwerveModuleState(),
			frc::SwerveModuleState(), frc::SwerveModuleState() };

private:
	nt::StructPublisher<frc::Pose2d> resetOdometryPosePublisher =
		nt::NetworkTableInstance::GetDefault().GetStructTopic < frc::Pose2d
		>("/PathPlanner/ResetPose").Publish();
};
// spotless:on
