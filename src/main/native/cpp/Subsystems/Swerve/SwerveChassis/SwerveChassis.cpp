// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "OvertureLib/Subsystems/Swerve/SwerveChassis/SwerveChassis.h"

/**
 * @brief Builds an object of swerve chassis
 */
SwerveChassis::SwerveChassis() : SwerveBase(this) {
	frc::SmartDashboard::PutData("Chassis/Odometry", &field2d);

	configureSwerveBase();
	pathplanner::PPHolonomicDriveController::setRotationTargetOverride(
			[this]() {
				return headingTarget;
			});
}

/**
 * @brief Sets the target heading
 *
 * @param rotationTarget The target heading
 */
void SwerveChassis::disableHeadingOverride() {
	headingOverride = false;
}

/**
 * @brief Sets the heading override
 *
 * @param headingOverride The heading override
 */
void SwerveChassis::enableHeadingOverride(frc::Rotation2d headingTarget) {
	headingOverride = true;
	this->headingTarget = headingTarget;
}

/**
 * @brief Returns the robot relative speeds
 *
 * @return ChassisSpeeds object
 */
frc::ChassisSpeeds SwerveChassis::getCurrentSpeeds() {
	return currentSpeeds;
}

/**
 * @brief Returns the robot odometry
 *
 * @return Pose2d object
 */
const frc::Pose2d& SwerveChassis::getEstimatedPose() {
	return latestPose;
}

/**
 * @brief Resets the robot odometry
 *
 * @param initPose Pose2d object
 */
void SwerveChassis::resetOdometry(frc::Pose2d initPose) {
	odometry->ResetPosition(getRotation2d(), modulesPositions, initPose);
}

/**
 * @brief Updates odometry using vision
 */
void SwerveChassis::addVisionMeasurement(frc::Pose2d pose,
		units::second_t timestamp) {
	odometry->AddVisionMeasurement(pose, timestamp);
}

/**
 * @brief Sets the odometry to desired angle
 *
 * @param angle Desired angle
 */
void SwerveChassis::resetHeading(double angle) {
	frc::Pose2d actualOdometry = getEstimatedPose();
	frc::Pose2d newOdometry { actualOdometry.X(), actualOdometry.Y(),
			units::degree_t(angle) };
	resetOdometry(newOdometry);
}

void SwerveChassis::setTargetSpeeds(frc::ChassisSpeeds speeds) {
	desiredSpeeds = speeds;
}

void SwerveChassis::setModuleStates(
		const wpi::array<frc::SwerveModuleState, 4> &desiredStates) {
	getFrontLeftModule().setState(desiredStates[0]);
	getFrontRightModule().setState(desiredStates[1]);
	getBackRightModule().setState(desiredStates[2]);
	getBackLeftModule().setState(desiredStates[3]);

	getFrontLeftModule().setVoltages();
	getFrontRightModule().setVoltages();
	getBackRightModule().setVoltages();
	getBackLeftModule().setVoltages();
}

const wpi::array<frc::SwerveModuleState, 4>& SwerveChassis::getModulesStates() {
	return modulesStates;
}

const wpi::array<frc::SwerveModulePosition, 4>& SwerveChassis::getModulesPositions() {
	return modulesPositions;
}

/**
 * @brief Runs the SysId Quasisstatic command
 */
frc2::CommandPtr SwerveChassis::SysIdQuadstatic(
		frc2::sysid::Direction direction) {
	return frc2::cmd::Sequence(frc2::InstantCommand([this]() {
		sysIdVoltage(0_V);
	}).ToPtr(), frc2::cmd::Wait(0.5_s), m_sysIdRoutine.Quasistatic(direction));
}

/**
 * @brief Runs the SysId Dynamic command
 */
frc2::CommandPtr SwerveChassis::SysIdDinamic(frc2::sysid::Direction direction) {
	return frc2::cmd::Sequence(frc2::InstantCommand([this]() {
		sysIdVoltage(0_V);
	}).ToPtr(), frc2::cmd::Wait(0.5_s), m_sysIdRoutine.Dynamic(direction));
}

/**
 * @brief Sets the voltage for the SysId command
 */
void SwerveChassis::sysIdVoltage(units::volt_t voltage) {
	getFrontLeftModule().setRawVoltageSpeed(voltage);
	getFrontRightModule().setRawVoltageSpeed(voltage);
	getBackRightModule().setRawVoltageSpeed(voltage);
	getBackLeftModule().setRawVoltageSpeed(voltage);
}

/**
 * @brief Updates the robot odometry
 */
void SwerveChassis::updateOdometry() {
	odometry->Update(getRotation2d(), getModulesPositions());
	latestPose = odometry->GetEstimatedPosition();
	currentSpeeds = getKinematics().ToChassisSpeeds(getModulesStates());

	currentAccels = ChassisAccels(currentSpeeds, lastSpeeds);
	lastSpeeds = currentSpeeds;
}

void SwerveChassis::shuffleboardPeriodic() {
	frc::SmartDashboard::PutNumber("Odometry/LinearX",
			desiredSpeeds.vx.value());
	frc::SmartDashboard::PutNumber("Odometry/LinearY",
			desiredSpeeds.vy.value());
	frc::SmartDashboard::PutNumber("Odometry/Angular",
			desiredSpeeds.omega.value());

	frc::SmartDashboard::PutNumber("Odometry/AccelX", currentAccels.ax.value());
	frc::SmartDashboard::PutNumber("Odometry/AccelY", currentAccels.ay.value());
	frc::SmartDashboard::PutNumber("Odometry/AccelOmega",
			currentAccels.omega.value());

	frc::SmartDashboard::PutNumber("Odometry/SpeedX", desiredSpeeds.vx.value());
	frc::SmartDashboard::PutNumber("Odometry/SpeedY", desiredSpeeds.vy.value());
	frc::SmartDashboard::PutNumber("Odometry/SpeedOmega",
			desiredSpeeds.omega.value());

	field2d.SetRobotPose(latestPose);
	getPoseLog().Append(latestPose);
	frc::SmartDashboard::PutNumber("Odometry/X", latestPose.X().value());
	frc::SmartDashboard::PutNumber("Odometry/Y", latestPose.Y().value());

	frc::SmartDashboard::PutNumber("Odometry/HeadingTarget",
			headingTarget.Degrees().value());
	frc::SmartDashboard::PutNumber("Odometry/HeadingError",
			getHeadingController().GetPositionError().value());
}

void SwerveChassis::Periodic() {
	updateOdometry();

	if (headingOverride) {
		double outOmega = getHeadingController().calculateValue(
				headingTarget.Radians(), latestPose.Rotation().Radians());
		if (std::abs(outOmega) < 0.1 && desiredSpeeds.vx == 0_mps
				&& desiredSpeeds.vy == 0_mps) {
			outOmega = 0.0;
		}
		desiredSpeeds.omega = units::radians_per_second_t { outOmega };
	}

	modulesPositions[0] = getFrontLeftModule().getPosition();
	modulesPositions[1] = getFrontRightModule().getPosition();
	modulesPositions[2] = getBackLeftModule().getPosition();
	modulesPositions[3] = getBackRightModule().getPosition();

	modulesStates[0] = getFrontLeftModule().getState();
	modulesStates[1] = getFrontRightModule().getState();
	modulesStates[2] = getBackLeftModule().getState();
	modulesStates[3] = getBackRightModule().getState();

	frc::ChassisSpeeds targetSpeeds = { getVxLimiter().Calculate(
			desiredSpeeds.vx), getVyLimiter().Calculate(desiredSpeeds.vy),
			getVwLimiter().Calculate(desiredSpeeds.omega) };
	wpi::array < frc::SwerveModuleState, 4U > desiredStates =
			getKinematics().ToSwerveModuleStates(targetSpeeds);
	getKinematics().DesaturateWheelSpeeds(&desiredStates, getMaxModuleSpeed());

	setModuleStates (desiredStates);
	shuffleboardPeriodic();
}
