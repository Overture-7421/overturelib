// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveChassis.h"

/**
* @brief Construye un nuevo objeto de swerve chassis
*/
SwerveChassis::SwerveChassis() {
    navx.Calibrate();

    std::this_thread::sleep_for(std::chrono::seconds(1));
    double startTime = frc::Timer::GetFPGATimestamp().value();
    while (navx.IsCalibrating()) {
        double timePassed = frc::Timer::GetFPGATimestamp().value() - startTime;
        if (timePassed > 10) {
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    navx.ZeroYaw();
}

void SwerveChassis::setModulePositions(std::array<frc::Translation2d, 4>* positions) {
    this->modulePos = positions;

    kinematics = new frc::SwerveDriveKinematics<4>{ *modulePos };
};

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

void SwerveChassis::setModules(SwerveModule* frontLeft, SwerveModule* frontRight, SwerveModule* backLeft, SwerveModule* backRight) {
    this->frontLeftModule = frontLeft;
    this->frontRightModule = frontRight;
    this->backLeftModule = backLeft;
    this->backRightModule = backRight;

    odometryPos = new std::array<frc::SwerveModulePosition, 4>{
        frontLeftModule->getPosition(),
            frontRightModule->getPosition(),
            backLeftModule->getPosition(),
            backRightModule->getPosition()
    };

    odometry = new frc::SwerveDrivePoseEstimator<4>{
        *kinematics,
            frc::Rotation2d{},
            *odometryPos,
            frc::Pose2d{}
    };

    resetNavxButton.OnTrue(frc2::InstantCommand{ [this]() {resetNavx();} }.ToPtr());
}

/**
 * @brief Pendiente descripcion
 */
void SwerveChassis::setRotatorPID(double kP, double kI, double kD) {
    backRightModule->setRotatorPIDValues(kP, kI, kD);
    backLeftModule->setRotatorPIDValues(kP, kI, kD);
    frontRightModule->setRotatorPIDValues(kP, kI, kD);
    frontLeftModule->setRotatorPIDValues(kP, kI, kD);
}

/**
 * @brief Pendiente descripcion
 */
void SwerveChassis::setDrivePID(double kP, double kI, double kD) {
    backRightModule->setDrivePIDValues(kP, kI, kD);
    backLeftModule->setDrivePIDValues(kP, kI, kD);
    frontRightModule->setDrivePIDValues(kP, kI, kD);
    frontLeftModule->setDrivePIDValues(kP, kI, kD);
}

/**
 * @brief Pendiente descripcion
 */
void SwerveChassis::setFeedForward(units::volt_t kS, units::volt_t kV, units::volt_t kA) {
    backRightModule->setFFConstants(kS, kV, kA);
    backLeftModule->setFFConstants(kS, kV, kA);
    frontRightModule->setFFConstants(kS, kV, kA);
    frontLeftModule->setFFConstants(kS, kV, kA);
}

/**
 * @brief Establece si se usa voltaje bruto para caracterizacion o teleoperado
 * @param set true para usar voltaje bruto
 */
void SwerveChassis::setUseRawVoltageSpeed(bool set) {
    frontLeftModule->setUseRawVoltageSpeed(set);
    frontRightModule->setUseRawVoltageSpeed(set);
    backLeftModule->setUseRawVoltageSpeed(set);
    backRightModule->setUseRawVoltageSpeed(set);
}

/**
 * @brief Establece el angulo objetivo del robot
 * @param targetAngle angulo objetivo
 */
void SwerveChassis::setTargetAngle(double targetAngle) {
    this->targetAngle = targetAngle;
}

/**
 * @brief Establece la velocidad del robot para teleoperado
 * @param speeds velocidad del robot
 */
void SwerveChassis::setSpeed(frc::ChassisSpeeds speeds) {
    this->linearX = speeds.vx.value();
    this->linearY = speeds.vy.value();
    this->angular = speeds.omega.value();

    wpi::array<frc::SwerveModuleState, 4> desiredStates = kinematics->ToSwerveModuleStates(speeds);

    setModuleStates(desiredStates);
}

/**
 * @brief Establece el voltaje de las ruedas para caracterizacion
 * @param voltage voltaje de las ruedas
 */
void SwerveChassis::setWheelVoltage(double voltage) {
    frontLeftModule->setWheelVoltage(voltage);
    frontRightModule->setWheelVoltage(voltage);
    backLeftModule->setWheelVoltage(voltage);
    backRightModule->setWheelVoltage(voltage);
}

/**
 * @brief Regresa la odometria del robot
 * @return odometria del robot
 */
frc::Pose2d SwerveChassis::getOdometry() {
    return odometry->GetEstimatedPosition();
}

/**
 * @brief resetea la odometria del robot
 * @param pose2d odometria a la que se reseteara
*/
void SwerveChassis::resetOdometry(frc::Pose2d initPose) {
    odometry->ResetPosition(frc::Rotation2d{ units::degree_t{-navx.GetAngle()} }, getModulePosition(), initPose);
}

/**
 * @brief Obtiene el cambio de direccion del robot
 * @return cambio de direccion del robot
*/
double SwerveChassis::getHeadingRate() {
    return -navx.GetRate();
}

/**
 * @brief Obtiene la kinematica del robot
 * @return kinematica del robot
 */
const frc::SwerveDriveKinematics<4>& SwerveChassis::getKinematics() {
    return *kinematics;
}

/**
 * @brief Se actualiza la odometrua con datos de la vision
 */
void SwerveChassis::addVisionMeasurement(frc::Pose2d pose, units::second_t timestamp) {
    odometry->AddVisionMeasurement(pose, timestamp);
}

/**
 * @brief Se actualiza la navx
 * @param angle angulo a actualizar
*/
void SwerveChassis::resetNavx(double angle) {
    frc::Pose2d actualOdometry = getOdometry();
    frc::Pose2d newOdometry{ actualOdometry.X(), actualOdometry.Y(), units::degree_t(angle) };
    resetOdometry(newOdometry);
}

/**
 * @brief Se actualizan las posiciones de los modulos
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
 * @brief Se obtienen los estados de los modulos
 * @return Arreglo de SwerveModuleStates con las posiciones de los modulos
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
 * @brief Se obtienen las posiciones de los modulos
 * @return Arreglo de SwerveModulePosition con las posiciones de los modulos
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
 * @brief Se obtienen el pitch del robot
 * @return pitch del robot
*/
double SwerveChassis::getPitch() {
    return navx.GetPitch();
}

/**
 * @brief Se obtienen el yaw del robot
 * @return yaw del robot
*/
double SwerveChassis::getYaw() {
    return getOdometry().Rotation().Degrees().value();
}

/**
 * @brief Se obtienen el roll del robot
 * @return roll del robot
*/
double SwerveChassis::getRoll() {
    return navx.GetRoll();
}

/**
 * @brief Se actuliza la odometria del robot
*/
void SwerveChassis::updateOdometry() {
    odometry->Update(frc::Rotation2d(units::degree_t(-navx.GetAngle())), getModulePosition());
}

/**
 * @brief Pendiente a describir
*/
void SwerveChassis::shuffleboardPeriodic() {
    frc::SmartDashboard::PutNumber("LinearX", linearX);
    frc::SmartDashboard::PutNumber("LinearY", linearY);
    frc::SmartDashboard::PutNumber("Angular", angular);

    auto estimatedPos = getOdometry();
    frc::SmartDashboard::PutNumber("OdometryX", estimatedPos.X().value());
    frc::SmartDashboard::PutNumber("OdometryY", estimatedPos.Y().value());
    frc::SmartDashboard::PutNumber("AnglenaveX", estimatedPos.Rotation().Degrees().value());
}
