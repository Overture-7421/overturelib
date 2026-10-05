# Alliance flip

Field positions are written once, in blue alliance coordinates, and `AllianceFlip` turns
them into the matching ones for red. It does the same arithmetic as PathPlanner's and
BLine's `FlippingUtil`, with the same field size, so a target flipped here lands exactly
where a path flipped by either of them does.

It lives in the library so that code which only aims at something does not have to import
a path follower to do it.

## The three things it does

| Call | What it does | Use it for |
| ---- | ------------ | ---------- |
| `AllianceFlip.flip(x)` | Always flips to the other alliance | When you already know you are red |
| `AllianceFlip.flipIfRed(x)` | Flips only if the driver station says red | Almost everything |
| `AllianceFlip.mirrorLeftRight(x)` | Moves to the other side of the field, same alliance | Writing a left target in terms of the right one |

`flip` and `flipIfRed` take a `Translation2d`, a `Rotation2d`, a `Pose2d` or field
relative `ChassisSpeeds`. `mirrorLeftRight` takes the first three.

Blue alliance coordinates are WPILib's: the origin in the corner to the right of the blue
drivers, x growing towards the red wall, y growing to their left.

## Example

Shelby's shooter targets, from `LaunchConstants`:

```java
import com.overture.lib.utils.AllianceFlip;

// Blue alliance coordinates
public static final Translation2d HubPose = new Translation2d(4.626, 4.035);
public static final Translation2d RightPass = new Translation2d(4.0, 2.5);
public static final Translation2d LeftPass = AllianceFlip.mirrorLeftRight(RightPass);

public static final double FieldMidline = AllianceFlip.getFieldWidth() / 2.0;

public static Translation2d getHubPose() {
    return AllianceFlip.flipIfRed(HubPose);
}
```

**Call `flipIfRed` where the value is used, not once at startup.** The alliance is not
known until the driver station connects, and until then it answers for blue. That is why
`HubPose` above stays a blue constant and `getHubPose()` is a method: a
`static final Translation2d Hub = AllianceFlip.flipIfRed(...)` is decided once, when the
class loads, which is usually while the robot boots and before it knows it is red.

Only field relative speeds are flipped. Robot relative speeds mean the same thing on both
alliances.

## The two kinds of field

How red relates to blue changes from season to season, and `flip` follows whichever one is
configured:

| `FieldSymmetry` | The red half is the blue half... | Position | Heading | Field speeds | Seasons |
| --------------- | -------------------------------- | -------- | ------- | ------------ | ------- |
| `ROTATIONAL` | turned 180 degrees about the field center | `(L - x, W - y)` | `θ - 180°` | `(-vx, -vy, ω)` | 2022, 2025, 2026 |
| `MIRRORED` | seen in a mirror across the middle of the field | `(L - x, y)` | `180° - θ` | `(-vx, vy, -ω)` | 2023, 2024 |

`L` is the field length and `W` its width. `mirrorLeftRight` is the same on both:
`(x, W - y)` and `-θ`.

The library ships set up for the current season, 2026: `ROTATIONAL`, 16.54 m by 8.07 m.
To run on another field, say so first thing in the `Robot` constructor:

```java
// The 2024 field
AllianceFlip.configure(AllianceFlip.FieldSymmetry.MIRRORED, 16.541, 8.211);
```

It has to be first because anything that already read the field size keeps the number it
read. `LeftPass` and `FieldMidline` in the example above are computed when
`LaunchConstants` is first used, from the field as it is configured at that moment.

## Coming from `FlippingUtil`

| PathPlanner or BLine | `AllianceFlip` |
| -------------------- | -------------- |
| `flipFieldPosition(p)`, `flipFieldRotation(r)`, `flipFieldPose(p)`, `flipFieldSpeeds(s)` | `flip(...)` |
| `if (isRedAlliance()) x = flipField...(x)` | `flipIfRed(x)` |
| `fieldSizeX`, `fieldSizeY` | `getFieldLength()`, `getFieldWidth()` |
| `symmetryType = kRotational` or `kMirrored` | `configure(FieldSymmetry.ROTATIONAL` or `MIRRORED, ...)` |
| BLine's `mirrorFieldPosition`, `mirrorFieldRotation`, `mirrorFieldPose` | `mirrorLeftRight(...)` |
| `flipFeedforwards`, `flipFeedforwardXs`, `flipFeedforwardYs` | Not provided: they reorder per module forces inside a path follower |

The path follower still flips its own paths with its own copy. PathPlanner 2026.1.2 and
BLine v0.9.2 both ship with the same field as this library, so nothing needs to be kept in
step for 2026. If you call `configure` for another field, the follower has its own field
settings that need changing too.
