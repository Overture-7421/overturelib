#pragma once
#include <frc/filter/SlewRateLimiter.h>
#include <OvertureLib/Sensors/OverPigeon/OverPigeon.h>
#include <OvertureLib/Subsystems/Swerve/SwerveModule/SwerveModule.h>
#include <OvertureLib/Math/ChassisAccels.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/DriverStation.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <units/length.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
using namespace pathplanner;

// spotless:off
class SwerveBase {
public:
	SwerveBase(frc2::Subsystem* driveSubsystem) : driveSubsystem(driveSubsystem) {

	}

	virtual const frc::Pose2d& getEstimatedPose() = 0;
	virtual void resetOdometry(frc::Pose2d initPose) = 0;
	virtual frc::ChassisSpeeds getCurrentSpeeds() = 0;
	virtual void setTargetSpeeds(frc::ChassisSpeeds speeds) = 0;

	virtual units::meters_per_second_t getMaxModuleSpeed() = 0;
	virtual units::meter_t getDriveBaseRadius() = 0;

	virtual frc::Rotation2d getRotation2d() = 0;
	virtual frc::Rotation3d getRotation3d() = 0;

protected:
	void configureSwerveBase() {

		pathplanner::RobotConfig robotConfig = RobotConfig::fromGUISettings();

		configuredChassis = true;
		AutoBuilder::configure(
			[this]() {return getEstimatedPose();},
			[this](frc::Pose2d pose) {resetOdometry(pose);},
			[this]() {return getCurrentSpeeds();},
			[this](frc::ChassisSpeeds speeds) {setTargetSpeeds(speeds);},
			std::make_shared<PPHolonomicDriveController>(
				PIDConstants(6.5, 0.0, 0.0),
				PIDConstants(6.5, 0.0, 0.0)
			),
			robotConfig,
			[]() {
			auto alliance = frc::DriverStation::GetAlliance();
			if (alliance) {
				return alliance.value() == frc::DriverStation::Alliance::kRed;
			}
			return false;
		},
			driveSubsystem
		);

		odometry = std::make_unique<frc::SwerveDrivePoseEstimator<4>>(getKinematics(), frc::Rotation2d{}, modulesPositions, frc::Pose2d{});
	}

	virtual SwerveModule& getFrontLeftModule() = 0;
	virtual SwerveModule& getFrontRightModule() = 0;
	virtual SwerveModule& getBackLeftModule() = 0;
	virtual SwerveModule& getBackRightModule() = 0;

	virtual frc::SlewRateLimiter<units::meters_per_second>& getVxLimiter() = 0;
	virtual frc::SlewRateLimiter<units::meters_per_second>& getVyLimiter() = 0;
	virtual frc::SlewRateLimiter<units::radians_per_second>& getVwLimiter() = 0;

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
	frc2::Subsystem* driveSubsystem;
};
// spotless:on
