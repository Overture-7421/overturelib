# OvertureLib

Team 7421's vendor library for FRC, built against **WPILib 2026**.

The library ships in both **C++ and Java**. The two implementations are kept feature
equivalent, but Java is the one being actively developed ahead of the 2027 SystemCore
transition, and **builds are Java only by default**. The C++ sources remain in the tree
and still build on demand, with `-PjavaOnly=false`.

## Layout

| Language | Sources | Published artifact |
| -------- | ------- | ------------------ |
| C++ | `src/main/native/cpp`, `src/main/native/include` | `com.overture.lib:OvertureLib-cpp` (only with `-PjavaOnly=false`) |
| Java | `src/main/java` | `com.overture.lib:OvertureLib-java` |

The Java root package is `com.overture.lib`. Package names mirror the C++ include
directories, lowercased:

| C++ header | Java package |
| ---------- | ------------ |
| `OvertureLib/Gamepads/...` | `com.overture.lib.gamepads` |
| `OvertureLib/Math/...` | `com.overture.lib.math` |
| `OvertureLib/MotorControllers/...` | `com.overture.lib.motorcontrollers` |
| `OvertureLib/Robots/...` | `com.overture.lib.robots` |
| `OvertureLib/Sensors/...` | `com.overture.lib.sensors` |
| `OvertureLib/Simulation/...` | `com.overture.lib.simulation` |
| `OvertureLib/Subsystems/LedsManager/...` | `com.overture.lib.subsystems.leds` |
| `OvertureLib/Subsystems/Swerve/...` | `com.overture.lib.subsystems.swerve` |
| `OvertureLib/Subsystems/Vision/...` | `com.overture.lib.subsystems.vision` |
| `OvertureLib/Utils/...` | `com.overture.lib.utils` |

## Conventions in the Java port

- **Units are plain SI doubles**, matching WPILib Java itself: meters, meters per second,
  seconds, radians, volts, amps. Motor-native quantities stay in rotations, which is what
  Phoenix 6's `double` overloads expect.
- **Simulation is selected at runtime, not compile time.** The C++ code switches on the
  `__FRC_ROBORIO__` macro; Java has no preprocessor, so the simulation managers are always
  linked and only driven when `RobotBase.isSimulation()` is true.
- **`SwerveBase` extends `SubsystemBase`.** C++ mixes the two in via multiple inheritance,
  which Java does not have, so the hierarchy is `SubsystemBase` → `SwerveBase` →
  `SwerveChassis` → your drivetrain.
- **Standard deviations are `Matrix<N3, N1>`** (built with `VecBuilder.fill`) rather than
  the C++ `wpi::array<double, 3>`, because that is what the WPILib pose estimator takes.
- `LimelightHelpers.java` is vendored from
  [limelightlib-wpijava](https://github.com/LimelightVision/limelightlib-wpijava) (v1.14).
  Only its `package` declaration and one malformed Javadoc tag were changed.

## Building

```bash
./gradlew build                    # Java only, the default
./gradlew build -PjavaOnly=false   # Java and the whole native toolchain
```

**Java only is the default**, set by `javaOnly=true` in `gradle.properties`, because the
library is being phased over to Java. It skips `config.gradle`, the native component and
the C++ publications, which also makes iteration on the Java sources much faster. Nothing
about the C++ build is removed, and `-PjavaOnly=false` is the way back to it.

The flag is read by value, not by presence. `hasProperty` alone is also true for
`javaOnly=false`, and a property set in `gradle.properties` cannot be unset from the
command line, so there would be no way back to a C++ build short of editing the file. A
bare `-PjavaOnly` still means Java only, which is what both CI workflows pass.

**CI builds Java only** as well, and the C++ jobs are commented out rather than deleted,
so released versions from here on ship the Java artifacts and nothing else.
`OvertureLib.json` declares an empty `cppDependencies` to match, because advertising
binaries that are never published makes every C++ platform 404 rather than just the
unbuilt ones.

To put C++ back: uncomment the `build-docker` and `build-host` jobs in both workflows, set
`javaOnly=false` in `gradle.properties`, and restore the `cppDependencies` block in
`OvertureLib.json` (see git history for the exact block).

By default the build resolves the latest WPILib development build. To build against the
last tagged release, add `-PreleaseMode`.

Formatting is enforced by Spotless (`googleJavaFormat` for Java, `eclipseCdt` for C++):

```bash
./gradlew spotlessApply
```

## Vendor dependencies

Java vendor dependencies are read directly out of `vendordeps/*.json` at configuration
time, so the Java and C++ builds always consume the same vendor versions. Adding or
bumping a vendordep JSON is all that is needed — there is no second list to update.

Currently: WPILib New Commands, CTRE Phoenix 6, PathPlannerLib and PhotonLib.
