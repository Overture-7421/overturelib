# OvertureLib documentation

Guides for using the library from a robot project, with the example code they refer to.

| Guide | What it covers |
| ----- | -------------- |
| [swerve](swerve/README.md) | Building a drivetrain on `SwerveChassis` and following paths with it, using PathPlanner or BLine |

## About the examples

The examples are plain Java files, written the way they would sit in a robot project and
based on Shelby's code ([FRC-Shelby-2026OS](https://github.com/Overture-7421/FRC-Shelby-2026OS)).
Copy them into a robot and change the numbers.

They are not part of the Gradle build. They depend on libraries OvertureLib deliberately
does not, so nothing compiles them when the library changes: if you change an API one of
them uses, update the example with it.
