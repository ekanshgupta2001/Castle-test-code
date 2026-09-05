# 4. Pedro paths

Pedro Pathing drives the robot along smooth curves to a target pose while correcting for error. We
use version 2.1.2.

> **Nothing in this lesson works on our robot yet.** `pedroPathing/Constants.java` is still
> default-constructed — no drivetrain, no localizer, no tuned gains. The code compiles and the
> structure is complete, but the follower cannot move a real robot until Constants is filled in and
> the tuning procedure has been run. Read this now; it becomes real the day that is done.

## Coordinates

- Origin at a **field corner**, 144 inches square
- **X** increases right, **Y** increases up
- Headings in **radians**, counter-clockwise from +X, and Pedro hands them back in `[0, 2π)`
- A `Pose` is `(x, y, heading)`

The Limelight reports position relative to the **field centre**, so converting means adding 72 to
both axes. `Limelight.getBotposeAsPedroPose()` does that for you.

Two seams to watch:

- `Math.atan2` returns `(-π, π]` while Pedro uses `[0, 2π)`. Mixing them breaks naive heading
  comparisons. Use `Angles.normalizeAngle()`.
- For "how far do I need to turn?", use `Angles.angleError()`, which takes the short way around.
  Plain subtraction says 350° → 10° is a 340° turn.

## The Follower's two modes

One object, two behaviours:

| Mode | Entered by | Behaviour |
|---|---|---|
| Teleop | `startTeleopDrive()` | Consumes stick vectors from `setTeleOpDrive()` |
| Path | `followPath(chain)` | Drives itself; ignores the sticks |

`Drivetrain` wraps both. The important thing is that **switching back is explicit**:

```java
public void cancelPath() {
    follower.breakFollowing();
    follower.startTeleopDrive();
}
```

Pedro's `holdPoint()` clears `isBusy` while still actively station-keeping, so a path that "finished"
can still be holding the robot in place. Without an explicit hand-back the sticks appear dead.

One subtlety: `startTeleopDrive()` internally calls `follower.update()`. Calling it every loop ticks
the follower twice per cycle and corrupts its velocity estimates — which is why `Drivetrain` guards
it behind a `teleopEngaged` flag.

## Building a path

```java
PathChain chain = follower.pathBuilder()
        .addPath(new BezierLine(startPose, endPose))
        .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
        .build();
```

Curve types: `BezierLine` (two points, straight), `BezierCurve` (three or more, curved),
`BezierPoint` (hold a position).

Heading interpolation is separate from the curve — the robot can rotate independently of where it
travels:

| Call | Effect |
|---|---|
| `setConstantHeadingInterpolation(h)` | Face one direction the whole way |
| `setLinearHeadingInterpolation(a, b)` | Rotate smoothly from a to b |
| `setTangentHeadingInterpolation()` | Face along the path |

## Running one

Use the command wrappers rather than calling the follower directly:

```java
drivetrain.followPathCommand(chain, /* holdEnd = */ true)
```

`holdEnd` decides what happens when the path finishes on its own. Pedro clears `isBusy` the
moment the end-of-path hold begins, so the *command* completes then either way. With `holdEnd =
true` the follower is **left station-keeping** at the target; the hold is released by the next
path, by `cancelPath()`, or, in teleop, the moment driver control resumes. With `holdEnd = false`
the wrapper calls `cancelPath()` and control goes straight back to the driver. Use `true` when
something might push the robot (a scoring pose in auto), `false` when you want the sticks back
immediately.

An earlier version called `cancelPath()` on *every* end, which silently made `holdEnd` a no-op: the
robot was free-wheeling at the score pose while the intake ejected. `Drivetrain.followLazyCommand`
now checks the `EndCondition` and only cancels on interruption or when not asked to hold.

These wrappers do two things a raw `followPath` does not: they `requiring(drivetrain)`, so driver
control is suspended for the duration, and their `setEnd` hands control back whenever the command
is interrupted, so a cancelled command actually stops the robot.

**Two wrappers, and the difference matters:**

| Wrapper | Path is built | Use for |
|---|---|---|
| `followPathCommand(chain, holdEnd)` | once, before scheduling | a fixed route you can draw on paper |
| `followLazyCommand(supplier, holdEnd)` | at command start, every run | anything whose target moves — vision, or a path from wherever the robot currently is |

Reach for the lazy one whenever the endpoints are not known at build time. The next section explains
why a stored chain cannot be reused.

## Paths whose target is not known yet

A `PathChain` **resolves its endpoints once and caches them**. `BezierLine.initialize()` begins with
`if (initialized) return;`. Build a chain, store it, follow it twice, and the second run goes to the
first run's target.

For anything vision-driven, build a fresh path each time:

```java
drivetrain.followLazyCommand(this::buildPollenPath, false)
```

The supplier runs at command start. Returning `null` — no valid target — yields a command that
finishes immediately rather than driving somewhere arbitrary.

(Pedro also offers `FuturePose`, a functional interface accepted by the Bezier constructors, which
defers resolution to first use. It has the same caching caveat, so `followLazyCommand` is the safer
habit.)

## Doing something part-way along

Waiting until a path ends before starting a mechanism wastes the whole drive. Callbacks fire mid-path:

```java
follower.pathBuilder()
        .addPath(new BezierLine(from, to))
        .setLinearHeadingInterpolation(from.getHeading(), to.getHeading())
        .addParametricCallback(0.6, intake::intake)   // 60% of the way there
        .build();
```

`MainAuto` uses exactly this to spin the intake up before arrival. Also available:
`addTemporalCallback(ms, action)` and `addPoseCallback(pose, action, guess)`.

Overlapping mechanism motion with driving is where autonomous cycle time actually comes from.

## Autonomous structure

`MainAuto` builds the whole routine as one `sequential` in `start()`, once the alliance is known:

```java
return sequential(
        leg("staging", staging, true, LEG_TIMEOUT_MS, intake::intake),

        skipIfAnyLegMissed(sequential(
                leg("score", score, true, LEG_TIMEOUT_MS),
                intake.runForMs(Intake.OUTTAKE_TICKS_PER_SEC, SCORE_EJECT_MS),
                instant(intake::markEmpty)
        )),

        leg("park", park, false, PARK_TIMEOUT_MS),
        instant(intake::stop)
);
```

`MainAuto` writes no loop at all — `MatchOpMode` owns it (lesson 1). All sequencing lives in the
command tree, so there is no hand-written state machine to get out of sync.

### What a "leg" actually is

`leg()` is not just `followPathCommand`. Three things are wrapped around every path:

```java
sequential(
    instant(() -> currentLeg = name),
    race(
        drivetrain.followLazyCommand(() -> pathToward(target, onApproach), holdEnd),
        waitMs(budgetMs)                 // 1. bounded
    ),
    instant(() -> finishLeg(name, target))   // 2. checked
);
```

1. **Bounded.** `follower.isBusy()` stays true forever when the robot is pinned against a wall or a
   defending robot. Without the `race`, the enclosing `sequential` blocks for the rest of the period
   and nothing on screen says why. This is the same rule `Macros` follows — it just took longer to
   reach autonomous.
2. **Checked.** `finishLeg` asks `follower.atPose(target, tol, tol)` whether the robot actually got
   there, and records `MISSED` if not. A leg that timed out is not assumed to have worked.
3. **Planned from the present.** `pathToward` builds the path from `drivetrain.getPose()` at command
   start, not from where the previous leg was *supposed* to end. After a missed leg, that is the
   difference between recovering and driving a path beginning somewhere the robot never reached.

Missing a leg skips the scoring block and falls through to park. A parked robot that scored nothing
beats a robot stuck against a wall, and both beat a routine that silently stopped executing.

### Legs cannot finish on tick one

`followLazyCommand` waits `Drivetrain.MIN_PATH_MS` before it will believe `!isBusy()`. The follower
does not guarantee `isBusy()` is true on the same tick `followPath()` was called, and without the
guard a leg can complete instantly — which looks exactly like a path that ran perfectly in zero time.
`PositionalMotor.MIN_MOVE_MS` exists for the identical reason.

A supplier returning `null` still finishes immediately; that path is deliberately exempt.

## Alliance mirroring

Write every pose for **blue**, once, and mirror it:

```java
Pose start = FieldConstants.forAlliance(FieldConstants.startPose(position), alliance);
```

Mirroring flips the heading as well as the position (`π − h` across the vertical centre line).
Mirroring position alone produces an autonomous that works perfectly on one alliance and drives
backwards on the other. The math is unit-tested in `FieldConstantsTest`; the coordinates are
placeholders you must measure.

## Tuning order

Constants must be tuned in this order — each step depends on the previous:

1. Localization (forward / lateral / turn multipliers)
2. Forward velocity → `MecanumConstants.xVelocity`
3. Lateral velocity → `yVelocity`
4. Forward zero-power acceleration
5. Lateral zero-power acceleration
6. Heading PIDF
7. Translational PIDF
8. Drive PIDF
9. Centripetal correction

Run the `Tuning` OpMode; it walks through each with instructions on screen.

---

Next: [5. Vision](05-vision.md)
