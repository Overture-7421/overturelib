#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <units/angular_acceleration.h>
#include <units/acceleration.h>

struct ChassisAccels {
    units::meters_per_second_squared_t ax;
    units::meters_per_second_squared_t ay;
    units::radians_per_second_squared_t omega;

    ChassisAccels(const frc::ChassisSpeeds& currentSpeed, const frc::ChassisSpeeds& lastSpeed, units::second_t period = 0.02_s){
        ax = (currentSpeed.vx - lastSpeed.vx) / period;
        ay = (currentSpeed.vy - lastSpeed.vy) / period;
        omega = (currentSpeed.omega - lastSpeed.omega) / period;

        if (units::math::abs(ax) > 6.0_mps_sq) {
            ax = 6.0 * units::math::copysign(1_mps_sq, ax);
        }
        if (units::math::abs(ay) > 6.0_mps_sq) {
            ay = 6.0 * units::math::copysign(1_mps_sq, ay);
        }
        if (units::math::abs(omega) > 6.0_rad_per_s_sq) {
            omega = 6.0 * units::math::copysign(1_rad_per_s_sq, omega);
        }
    }

    ChassisAccels(units::meters_per_second_squared_t ax, units::meters_per_second_squared_t ay, units::radians_per_second_squared_t omega) :
        ax(ax), ay(ay), omega(omega){
    }

    ChassisAccels() :
        ax(0_mps_sq), ay(0_mps_sq), omega(0_rad_per_s_sq){
    }
    static ChassisAccels FromRobotRelativeAccels(
        units::meters_per_second_squared_t ax, units::meters_per_second_squared_t ay,
        units::radians_per_second_squared_t omega, const frc::Rotation2d& robotAngle) {
        // CCW rotation out of chassis frame
        auto rotated =
            frc::Translation2d{units::meter_t{ax.value()}, units::meter_t{ay.value()}}
                .RotateBy(robotAngle);
        return {units::meters_per_second_squared_t{rotated.X().value()},
                units::meters_per_second_squared_t{rotated.Y().value()}, omega};
    }

    static ChassisAccels FromRobotRelativeAccels(
        const ChassisAccels& robotRelativeAccels, const frc::Rotation2d& robotAngle) {
        return FromRobotRelativeAccels(robotRelativeAccels.ax,
                                       robotRelativeAccels.ay,
                                       robotRelativeAccels.omega, robotAngle);
    }
};