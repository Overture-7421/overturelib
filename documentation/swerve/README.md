# Swerve and path following

OvertureLib builds the drivetrain. It does not choose how the robot follows paths: that is
wired up in the robot project, so a robot can use PathPlanner, BLine, both or neither
without the library carrying any of them.

## What a path follower plugs into

Every path follower asks for the same handful of things, and `SwerveChassis` already has
them as public methods:

| The follower wants | Give it | Notes |
| ------------------ | ------- | ----- |
| The robot pose | `chassis::getEstimatedPose` | Odometry fused with vision |
| A way to reset the pose | `chassis::resetPose` | Not `resetOdometry`, see below |
| The current speeds | `chassis::getCurrentSpeeds` | Robot relative |
| Somewhere to send speeds | `chassis::setTargetSpeeds` | Robot relative |
| The subsystem it requires | `chassis` | |

**`resetPose` or `resetOdometry`?** Today they do the same thing, but they mean different
things. A path follower resetting the pose says "the robot is here now", while
`resetOdometry` is also what heading resets and vision corrections go through. Hand a
follower `resetPose`, so that anything that should happen on a placement and not on a
correction has one place to live.

Speeds sent to `setTargetSpeeds` go through the same path as the driver's: an active
`SpeedsHelper` still rewrites them, and they are discretized and desaturated before
reaching the modules.

## The examples

| File | What it is |
| ---- | ---------- |
| [Chassis.java](examples/Chassis.java) | Shelby's drivetrain. No path follower in sight |
| [PathPlannerConfig.java](examples/PathPlannerConfig.java) | Hands that chassis to PathPlanner |
| [BLinePaths.java](examples/BLinePaths.java) | Hands that chassis to BLine |

The chassis is the same for both followers. Each follower lives in one small class beside
it, in `frc.robot.Subsystems.Chassis`, and switching is a matter of which one
`RobotContainer` uses.

They were compiled against PathPlannerLib 2026.1.2 and BLine-Lib v0.9.2.

## PathPlanner

1. Install the vendordep in the robot project (WPILib: Manage Vendor Libraries, Install new
   libraries (online)):

    ```text
    https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json
    ```

2. Open the robot project in the PathPlanner app and fill in the robot config. It is saved
   to `src/main/deploy/pathplanner/settings.json`, which is what
   `RobotConfig.fromGUISettings()` reads. Nothing checks it against the chassis, so the
   numbers have to be kept in step by hand. For Shelby:

    | `settings.json` | Value | Same number in `Chassis.java` |
    | --------------- | ----- | ----------------------------- |
    | `robotMass` | 61.235 | None: the chassis does not need the mass. Weigh the robot |
    | `driveGearing` | 7.03 | `kDriveGearRatio` |
    | `driveWheelRadius` | 0.0508 | `kWheelRadiusInches` (2 in) |
    | `maxDriveSpeed` | 4.541 | `getMaxModuleSpeed()` |
    | `flModuleX`, `flModuleY` | 0.282575, 0.276225 | `kTrackXInches`, `kTrackYInches` (11.125 in, 10.875 in) |
    | `driveCurrentLimit` | 60.0 | `SupplyCurrentLimit` |

3. Copy [PathPlannerConfig.java](examples/PathPlannerConfig.java) next to the chassis and
   call it from `RobotContainer`:

    ```java
    public final Chassis chassis = new Chassis();
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Before the chooser, which builds every auto in deploy/pathplanner/autos as it is
        // created. NamedCommands.registerCommand(...) goes before it too, for the same reason.
        PathPlannerConfig.configure(chassis);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
    ```

The path following gains are the two `PIDConstants` at the top of `PathPlannerConfig`.

## BLine

1. Install the vendordep in the robot project:

    ```text
    https://bline-metrics.edan-liahovetsky.workers.dev/vendor/BLine-Lib.json
    ```

    If that URL is down, BLine's README gives
    `https://raw.githubusercontent.com/edanliahovetsky/BLine-Lib/main/BLine-Lib.json` as
    the fallback.

2. Draw the paths in the [BLine editor](https://bline-web.pages.dev/). BLine loads them
   from `src/main/deploy/autos/paths/<name>.json`, and the default constraints from
   `src/main/deploy/autos/config.json`.

3. Copy [BLinePaths.java](examples/BLinePaths.java) next to the chassis and build the
   autos from it in `RobotContainer`:

    ```java
    public final Chassis chassis = new Chassis();
    private final BLinePaths paths = new BLinePaths(chassis);
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        autoChooser.setDefaultOption("None", Commands.none());
        autoChooser.addOption(
                "Left double pass",
                Commands.sequence(
                        paths.resetAndFollow("LeftDoublePass1"),
                        paths.follow("LeftDoublePass2")));
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
    ```

Two things about BLine that the example is built around:

- **The pose reset belongs to the first path only.** BLine resets the pose every time a
  command built with a reset starts. `resetAndFollow` is for the first path of an auto and
  `follow` for the rest; a reset on a later path throws away what odometry and vision
  worked out on the way there.
- **A path file is read when its command is built**, and a missing one throws. Building
  the autos in the `RobotContainer` constructor, as above, makes that a crash at startup
  in the pits instead of at the start of a match.

If a path has event triggers that schedule commands, BLine's README recommends composing
the auto with its own `BLineCommands.sequence` instead of `Commands.sequence`.

The path following gains are the three `PIDController`s in `BLinePaths`. They are BLine's
suggested starting values and have not been tuned on a robot.

## Updating a robot from 2026.4.0-beta-3 or earlier

Up to 2026.4.0-beta-3, `SwerveBase.configureSwerveBase()` configured PathPlanner itself. A
robot written against those versions needs three changes when it updates:

1. **Delete `getTranslationPID()` and `getRotationPID()` from the chassis.** They no longer
   override anything, so the `@Override` on them stops compiling. The gains move to
   `PathPlannerConfig`.
2. **Add `PathPlannerConfig` and call `PathPlannerConfig.configure(chassis)`.** Without it
   `AutoBuilder` is never configured, and PathPlanner refuses to build autos.
3. **Stop relying on `/PathPlanner/ResetPose`.** The library no longer publishes that
   NetworkTables topic. It existed for the old external simulator.

Robots that never ran an auto need only the first.

## Updating a robot from 2026.4.0-beta-4

2026.4.0-beta-4 simulated the drivetrain with maple-sim. That is gone: a simulator that
drives the devices themselves, such as WPI Emulator, runs the unmodified robot code, and a
second simulation inside the library fought it over the same motor and gyro states. A
robot written against beta-4 needs these changes:

1. **Delete `getSimulationConfig()` and `getSimulationPigeon()` from the chassis,** with
   their `org.ironmaple` imports. They no longer override anything, so the `@Override` on
   them stops compiling.
2. **Stop calling `getSimulatedPose()` and `resetSimulatedPose()`.** They are gone with the
   simulation. `resetPose` stays, and is still the reset to hand a path follower.
3. **Remove the maple-sim vendordep,** unless the robot uses maple-sim on its own.

Outside a simulator the drivetrain no longer moves in simulation, because nothing feeds
the motors' simulated sensors.

Nothing else changes for the robot, but the drivetrain reads its sensors differently. The
chassis refreshes all four modules in one batched call per CAN bus at the start of its
`periodic()`, and a module owned by a chassis no longer reads its sensors again in its own
`periodic()`. A module's `getVoltageDrive()` returns the voltage as of that refresh. X mode
brakes through the drive requests rather than by rewriting the neutral mode config.

Telemetry moved, with no values lost. The per-module `/Swerve/Modules/<name>/State`,
`/TargetState` and `/Distance` topics are gone; the same values are in the
`/Swerve/Modules/States`, `/Swerve/Modules/TargetStates` and `/Swerve/Modules/Positions`
arrays, indexed front left, front right, back left, back right. Point any dashboard or
AdvantageScope layout that used the old names at the arrays. `RequestedVoltage` and
`AppliedVoltage` stay per module.

## Updating a robot from 2026.4.0-beta-5

Up to 2026.4.0-beta-5 the module config carried a WPILib `SimpleMotorFeedforward`, in
volts per meter per second, which the library computed every loop and handed to the Talon
on top of its velocity request. The gains now live on the Talon itself, in
`DriveMotorConfig.Slot0`, and the library sends only the speed and its acceleration. A
robot written against beta-5 needs these changes:

1. **Build the module config with ids only:** `new SwerveModuleConfig(driveId, turnId)`.
   The feedforward argument is gone, and so is the `FeedForward` field.
2. **Move the gains into `DriveMotorConfig.Slot0`, in wheel rotations per second.** The
   Talon's units are the mechanism's, because the drive sensor is scaled by
   `DriveGearRatio`. Multiply each meters-per-second gain by the wheel circumference,
   `Math.PI * WheelDiameter`: for a 4 inch wheel that is 0.319, so a kV of 2.0879 V per m/s
   becomes 0.666 V per rotation per second. Converted like this the robot drives exactly as
   before.
3. **Set a kP.** Without one the drive is open loop, as it always was. kP is in volts per
   rotation per second of error; 0.3 is a sane start, and a log of
   `/Swerve/Modules/States` against `/Swerve/Modules/TargetStates` tunes it from there.

kA now does something. The library derives the acceleration of the request from one loop
to the next and sends it with the speed, so the Talon applies kA to it. A request that
steps shows up as a voltage spike for one loop, so leave kA at zero if the drive command
does not limit its own acceleration.

The per-module `/Swerve/Modules/<name>/RequestedVoltage` topic is replaced by
`/FeedForwardVoltage`, the feedforward volts the Talon computed from its gains.
`AppliedVoltage` stays, and the difference between the two is what kP contributed.
