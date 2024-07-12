// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveChassis.h"

/**
* @brief Builds an object of swerve chassis
*/
SwerveChassis::SwerveChassis(units::meters_per_second_t maxModuleSpeed, units::meter_t driveBaseRadius) {

	AutoBuilder::configureHolonomic(
		[this]() { return getOdometry(); },
		[this](frc::Pose2d pose) { resetOdometry(pose); },
		[this]() { return getRobotRelativeSpeeds(); },
		[this](frc::ChassisSpeeds speeds) { driveRobotRelative(speeds); },
		HolonomicPathFollowerConfig(
			PIDConstants(6.5, 0.0, 0.0),
			PIDConstants(6.5, 0.0, 0.0),
			maxModuleSpeed,
			driveBaseRadius,
			ReplanningConfig()
		),
		[]() {

		auto alliance = frc::DriverStation::GetAlliance();
		if (alliance) {
			return alliance.value() == frc::DriverStation::Alliance::kRed;
		}
		return false;
	},
		this // Reference to this subsystem to set requirements
	);

	frc::SmartDashboard::PutData("Chassis/Odometry", &field2d);

	this->maxModuleSpeed = maxModuleSpeed;
	this->driveBaseRadius = driveBaseRadius;

	pathplanner::PPHolonomicDriveController::setRotationTargetOverride([this]() { return getRotationTargetOverride(); });
}

std::optional<frc::Rotation2d> SwerveChassis::getRotationTargetOverride() {
	if (headingOverride) {
		return headingTarget;
	} else {
		return std::nullopt;
	}
}

/**
 * @brief Sets the target heading
 *
 * @param rotationTarget The target heading
 */
void SwerveChassis::setTargetHeading(frc::Rotation2d rotationTarget) {
	headingTarget = rotationTarget;
}

/**
 * @brief Sets the heading override
 *
 * @param headingOverride The heading override
 */
void SwerveChassis::setHeadingOverride(bool headingOverride) {
	this->headingOverride = headingOverride;
	headingController.reset(getOdometry().Rotation().Radians());
}

void SwerveChassis::setVyOverride(bool vyOverride) {
	this->vyOverride = vyOverride;
}

void SwerveChassis::setVyTarget(units::meters_per_second_t vy) {
	this->vyTarget = vy;
}

/**
 * @brief Sets the swerve module positions for the kinematics and odometry.
 *
 * @param positions The positions of the modules relative to the center of the robot.
 */
void SwerveChassis::setModulePositions(std::array<frc::Translation2d, 4>* positions) {
	kinematics = std::make_unique<frc::SwerveDriveKinematics<4>>(*positions);
};

/**
* @brief Sets the swerve modules ratios
*
* @param turnRatio The ratio between the turn motor and the wheel
* @param driveRatio The ratio between the drive motor and the wheel
* @param wheelDiameter The diameter of the wheel
*/
void SwerveChassis::setModulesRatios(double turnRatio, double driveRatio, double wheelDiameter) {
	frontRightModule->setGearRatio(turnRatio, driveRatio);
	frontLeftModule->setGearRatio(turnRatio, driveRatio);
	backRightModule->setGearRatio(turnRatio, driveRatio);
	backLeftModule->setGearRatio(turnRatio, driveRatio);

	frontRightModule->setWheelDiameter(wheelDiameter);
	frontLeftModule->setWheelDiameter(wheelDiameter);
	backRightModule->setWheelDiameter(wheelDiameter);
	backLeftModule->setWheelDiameter(wheelDiameter);
}

/**
 * @brief Sets the swerve modules
 *
 * @param frontLeft The front left module
 * @param frontRight The front right module
 * @param backLeft The back left module
 * @param backRight The back right module
 */
void SwerveChassis::setModules(SwerveModule* frontLeft, SwerveModule* frontRight, SwerveModule* backLeft, SwerveModule* backRight) {
	this->frontLeftModule = frontLeft;
	this->frontRightModule = frontRight;
	this->backLeftModule = backLeft;
	this->backRightModule = backRight;

	pigeon->SetYaw(0_deg);

	odometryPos = { frontLeftModule->getPosition(),
			frontRightModule->getPosition(),
			backLeftModule->getPosition(),
			backRightModule->getPosition() };

	odometry = std::make_unique<frc::SwerveDrivePoseEstimator<4>>(
		*kinematics,
		frc::Rotation2d{},
		odometryPos,
		frc::Pose2d{}
	);

}

/**
 * @brief Sets each module rotator PID values
 *
 * @param kP The proportional value
 * @param kI The integral value
 * @param kD The derivative value
 */
void SwerveChassis::setRotatorPID(double kP, double kI, double kD) {
	backRightModule->setRotatorPIDValues(kP, kI, kD);
	backLeftModule->setRotatorPIDValues(kP, kI, kD);
	frontRightModule->setRotatorPIDValues(kP, kI, kD);
	frontLeftModule->setRotatorPIDValues(kP, kI, kD);
}

/**
 * @brief Sets the modules feedforward values
 *
 * @param kS The static value
 * @param kV The velocity value
 * @param kA The acceleration value
 */
void SwerveChassis::setFeedForward(units::volt_t kS, units::volt_t kV, units::volt_t kA) {
	backRightModule->setFFConstants(kS, kV, kA);
	backLeftModule->setFFConstants(kS, kV, kA);
	frontRightModule->setFFConstants(kS, kV, kA);
	frontLeftModule->setFFConstants(kS, kV, kA);
}

/**
 * @brief Sets the robot target speed in robot relative
 *
 * @param speeds ChassisSpeeds object
 */
void SwerveChassis::driveRobotRelative(frc::ChassisSpeeds speeds) {
	desiredSpeeds = speeds;
}

/**
 * @brief Sets the robot target speed in field relative
 *
 * @param speeds ChassisSpeeds object
 */
void SwerveChassis::driveFieldRelative(frc::ChassisSpeeds speeds) {
	frc::ChassisSpeeds chassisSpeeds = frc::ChassisSpeeds::Discretize(frc::ChassisSpeeds::FromFieldRelativeSpeeds(speeds, getOdometry().Rotation()), RobotConstants::LoopTime);

	driveRobotRelative(chassisSpeeds);
}

/**
 * @brief Returns the robot relative speeds
 *
 * @return ChassisSpeeds object
 */
frc::ChassisSpeeds SwerveChassis::getRobotRelativeSpeeds() {
	return kinematics->ToChassisSpeeds(getModuleStates());
}

frc::ChassisSpeeds SwerveChassis::getFieldRelativeSpeeds() {
	return fieldRelativeSpeed;
}

ChassisAccels SwerveChassis::getFieldRelativeAccels() {
	return fieldRelativeAccel;
}

/**
 * @brief Sets the robot alliance
 */
void SwerveChassis::setAlliance() {
	if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
		alliance = -1;
	} else {
		alliance = 1;
	}
}

/**
 * @brief Limits the robot speed
 *
 * @param speeds ChassisSpeeds object
 * @return ChassisSpeeds object
 */
frc::ChassisSpeeds SwerveChassis::limitSpeeds(frc::ChassisSpeeds speeds, bool isFieldRelative) {
	units::meters_per_second_t xSpeed = xLimiter.Calculate(speeds.vx);
	units::meters_per_second_t ySpeed = yLimiter.Calculate(speeds.vy);
	units::radians_per_second_t omega = rLimiter.Calculate(speeds.omega);

	if (isFieldRelative) {
		return { alliance * xSpeed, alliance * ySpeed, omega };
	} else {
		return { xSpeed, ySpeed, omega };
	}
}

/**
 * @brief Sets the robot target speed
 *
 * @param speeds ChassisSpeeds object
 * @param isFieldRelative Boolean
 * @param isOpenLoop Boolean
 */
void SwerveChassis::setDrive(frc::ChassisSpeeds speeds, bool isFieldRelative, bool isOpenLoop, frc::Rotation2d heading) {
	if (isFieldRelative && isOpenLoop) {
		driveFieldRelative(limitSpeeds(speeds, isFieldRelative));
	} else if (isFieldRelative && !isOpenLoop) {
		headingTarget = heading;
		driveFieldRelative(limitSpeeds(speeds, isFieldRelative));
	} else {
		driveRobotRelative(limitSpeeds(speeds, isFieldRelative));
	}
}

/**
 * @brief Returns the robot odometry
 *
 * @return Pose2d object
 */
const frc::Pose2d& SwerveChassis::getOdometry() {
	return latestPose;
}

/**
 * @brief Resets the robot odometry
 *
 * @param initPose Pose2d object
 */
void SwerveChassis::resetOdometry(frc::Pose2d initPose) {
	odometry->ResetPosition(pigeon->GetRotation2d(), getModulePosition(), initPose);
}

/**
 * @brief Return the robot kinematics
 *
 * @return SwerveDriveKinematics object
 */
const frc::SwerveDriveKinematics<4>& SwerveChassis::getKinematics() {
	return *kinematics;
}

/**
 * @brief Updates odometry using vision
 */
void SwerveChassis::addVisionMeasurement(frc::Pose2d pose, units::second_t timestamp) {
	if (acceptVisionMeasurements) {
		odometry->AddVisionMeasurement(pose, timestamp);
	}
}

void SwerveChassis::setAcceptingVisionMeasurements(bool acceptVisionMeasurements) {
	this->acceptVisionMeasurements = acceptVisionMeasurements;
}

/**
 * @brief Sets the odometry to desired angle
 *
 * @param angle Desired angle
 */
void SwerveChassis::resetAngle(double angle) {
	frc::Pose2d actualOdometry = getOdometry();
	frc::Pose2d newOdometry{ actualOdometry.X(), actualOdometry.Y(), units::degree_t(angle) };
	resetOdometry(newOdometry);
}

/**
 * @brief Resets the robot heading
 *
 * @return CommandPtr
 */
frc2::CommandPtr SwerveChassis::resetHeading() {
	return frc2::cmd::RunOnce([this] {
		double angle = alliance = 1 ? 0 : 180;
		resetAngle(angle);
	});
}

/**
 * @brief Updates module states
 *
 * @param desiredStates SwerveModuleState array
 */
void SwerveChassis::setModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates) {
	frontLeftModule->setState(desiredStates[0]);
	frontRightModule->setState(desiredStates[1]);
	backRightModule->setState(desiredStates[2]);
	backLeftModule->setState(desiredStates[3]);

	backRightModule->setVoltages();
	backLeftModule->setVoltages();
	frontLeftModule->setVoltages();
	frontRightModule->setVoltages();
}

/**
 * @brief Returns the module states
 *
 * @return SwerveModuleState array
 */
wpi::array<frc::SwerveModuleState, 4> SwerveChassis::getModuleStates() {
	wpi::array<frc::SwerveModuleState, 4> modulePositions{
		frontLeftModule->getState(),
			frontRightModule->getState(),
			backLeftModule->getState(),
			backRightModule->getState()
	};
	return modulePositions;
}

/**
 * @brief Returns the module positions
 *
 * @return SwerveModulePosition array
 */
wpi::array<frc::SwerveModulePosition, 4> SwerveChassis::getModulePosition() {
	wpi::array<frc::SwerveModulePosition, 4> modulePositions{
		frontLeftModule->getPosition(),
			frontRightModule->getPosition(),
			backLeftModule->getPosition(),
			backRightModule->getPosition()
	};
	return modulePositions;
}

/**
 * @brief Returns the robot pitch
 *
 * @return double
 */
double SwerveChassis::getPitch() {
	return pigeon->GetPitch().GetValue().value();
}

/**
 * @brief Returns the robot yaw
 *
 * @return double
 */
double SwerveChassis::getYaw() {
	return pigeon->GetYaw().GetValue().value();
}

/**
 * @brief Returns the robot roll
 *
 * @return double
 */
double SwerveChassis::getRoll() {
	return pigeon->GetRoll().GetValue().value();
}

/**
 * @brief Runs the SysId Quasisstatic command
*/
frc2::CommandPtr SwerveChassis::SysIdQuadstatic(frc2::sysid::Direction direction) {
	return frc2::cmd::Sequence(
		frc2::InstantCommand([this]() { sysIdVoltage(0_V); }).ToPtr(),
		frc2::cmd::Wait(0.5_s),
		m_sysIdRoutine.Quasistatic(direction)
	);
}

/**
 * @brief Runs the SysId Dynamic command
*/
frc2::CommandPtr SwerveChassis::SysIdDinamic(frc2::sysid::Direction direction) {
	return frc2::cmd::Sequence(
		frc2::InstantCommand([this]() { sysIdVoltage(0_V); }).ToPtr(),
		frc2::cmd::Wait(0.5_s),
		m_sysIdRoutine.Dynamic(direction)
	);
}

/**
 * @brief Sets the voltage for the SysId command
*/
void SwerveChassis::sysIdVoltage(units::volt_t voltage) {
	frontLeftModule->setRawVoltageSpeed(voltage);
	frontRightModule->setRawVoltageSpeed(voltage);
	backLeftModule->setRawVoltageSpeed(voltage);
	backRightModule->setRawVoltageSpeed(voltage);
}


/**
 * @brief Updates the robot odometry
 */
void SwerveChassis::updateOdometry() {
	odometry->Update(pigeon->GetRotation2d(), getModulePosition());
	latestPose = odometry->GetEstimatedPosition();
	frc::ChassisSpeeds robotRelativeSpeed = getRobotRelativeSpeeds();
	frc::Rotation2d robotHeading = latestPose.Rotation();

	fieldRelativeSpeed = frc::ChassisSpeeds::FromRobotRelativeSpeeds(robotRelativeSpeed, robotHeading);
	fieldRelativeAccel = ChassisAccels(fieldRelativeSpeed, lastFieldRelativeSpeed);

	lastFieldRelativeSpeed = fieldRelativeSpeed;
}

void SwerveChassis::shuffleboardPeriodic() {
	// frc::SmartDashboard::PutNumber("Odometry/LinearX", desiredSpeeds.vx.value());
	// frc::SmartDashboard::PutNumber("Odometry/LinearY", desiredSpeeds.vy.value());
	// frc::SmartDashboard::PutNumber("Odometry/Angular", desiredSpeeds.omega.value());

	// frc::SmartDashboard::PutNumber("Odometry/AccelX", fieldRelativeAccel.ax.value());
	// frc::SmartDashboard::PutNumber("Odometry/AccelY", fieldRelativeAccel.ay.value());
	// frc::SmartDashboard::PutNumber("Odometry/AccelOmega", fieldRelativeAccel.omega.value());


	// frc::SmartDashboard::PutNumber("Odometry/SpeedX", fieldRelativeSpeed.vx.value());
	// frc::SmartDashboard::PutNumber("Odometry/SpeedY", fieldRelativeSpeed.vy.value());
	// frc::SmartDashboard::PutNumber("Odometry/SpeedOmega", fieldRelativeSpeed.omega.value());

	field2d.SetRobotPose(latestPose);
	poseLog.Append(latestPose);
	// frc::SmartDashboard::PutNumber("Odometry/X", estimatedPos.X().value());
	// frc::SmartDashboard::PutNumber("Odometry/Y", estimatedPos.Y().value());
}

void SwerveChassis::Periodic() {
	updateOdometry();

	if (headingOverride) {
		double outOmega = headingController.calculate(headingTarget.Radians(), latestPose.Rotation().Radians());
		if (std::abs(outOmega) < 0.1 && desiredSpeeds.vx == 0_mps && desiredSpeeds.vy == 0_mps) {
			outOmega = 0.0;
		}
		desiredSpeeds.omega = units::radians_per_second_t{ outOmega };
		// frc::SmartDashboard::PutNumber("Odometry/HeadingTarget", headingTarget.Degrees().value());
		// frc::SmartDashboard::PutNumber("Odometry/HeadingError", headingController.GetPositionError().value());
	}

	if (vyOverride) {
		desiredSpeeds.vy = vyTarget;
	}

	wpi::array<frc::SwerveModuleState, 4U> desiredStates = kinematics->ToSwerveModuleStates(desiredSpeeds);

	kinematics->DesaturateWheelSpeeds(&desiredStates, maxModuleSpeed);

	setModuleStates(desiredStates);
	shuffleboardPeriodic();
}