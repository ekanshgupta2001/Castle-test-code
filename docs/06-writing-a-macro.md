# 6. Writing a macro

A macro is one button that does a job the drivers would otherwise do by hand. They live in
`commands/Macros.java`.

The goal is that a driver presses a button and the robot handles the rest — which means a macro has
to be **safe when things go wrong**, not just correct when they go right. A macro that works in
practice and hangs in a match is worse than no macro.

## The four rules

Every macro in this file follows all four. They are not style preferences; each one is a bug we hit.

### 1. Bounded

Every wait is wrapped in a timeout:

```java
race(
    waitUntil(robot.limelight::hasStablePollen),
    waitMs(SEARCH_TIMEOUT_MS)
)
```

An unbounded `waitUntil` whose condition never becomes true holds its subsystems **forever**. The
default commands stay suspended, the intake stops responding, and nothing on the screen says why. The
original `collectPollen` did exactly this whenever no game piece was visible.

### 2. Reports an outcome

```java
finish(Outcome.SUCCESS, Outcome.TIMED_OUT, this::capturedSomethingNew)
```

`getStatus()` shows up in telemetry as `collectPollen : TIMED_OUT`. A macro that fails silently is
worse than one that does nothing, because the drivers keep trusting it.

Be careful what "success" means. `collectPollen` originally succeeded when the *path* finished — so a
run that drove perfectly and picked up nothing reported success. It now requires a **new** capture:

```java
private boolean capturedSomethingNew() {
    return robot.intake.hasPollen() && !hadPollenAtStart;
}
```

Without the snapshot, pressing the button while already holding a piece reports instant success and
does nothing.

### 3. Declares its resources

Requirements come from the subsystem commands you compose. Because `followLazyCommand` requires the
drivetrain, scheduling the macro suspends driver control automatically, and ending it restores
control automatically.

A macro holds the drivetrain for its **whole** duration, including the search phase before the robot
moves. That is deliberate — it keeps arbitration simple — and it is safe because the driver can abort
by touching a stick.

### 4. Builds vision paths lazily

```java
robot.drivetrain.followLazyCommand(this::buildPollenPath, false)
```

A `PathChain` caches its endpoints on first use, so a stored chain drives to a stale target. The
supplier runs at command start; returning `null` finishes the command immediately rather than driving
somewhere arbitrary.

## A worked example

`collectPollen()`, annotated:

```java
return sequential(
    begin("collectPollen"),                                  // set status for telemetry
    instant(() -> hadPollenAtStart = robot.intake.hasPollen()),  // snapshot for rule 2
    instant(robot.limelight::activatePollenPipeline),
    waitMs(PIPELINE_WARMUP_MS),                              // the camera needs a few frames

    race(                                                    // rule 1: bounded search
        waitUntil(robot.limelight::hasStablePollen),         // stable, not just "seen"
        waitMs(SEARCH_TIMEOUT_MS)
    ),

    race(                                                    // rule 1 again
        deadline(
            robot.drivetrain.followLazyCommand(this::buildPollenPath, false),  // rule 4
            robot.intake.captureAndHoldCommand()             // runs alongside the drive
        ),
        waitUntil(this::capturedSomethingNew),               // stop early on a real capture
        waitMs(APPROACH_TIMEOUT_MS)
    ),

    instant(robot.limelight::activateAprilTagPipeline),      // leave the camera usable
    finish(Outcome.SUCCESS, Outcome.TIMED_OUT, this::capturedSomethingNew)   // rule 2
);
```

The `deadline` is the interesting part: the path is the timekeeper, the intake runs alongside, and
when the path ends the intake command is cancelled and its `setEnd` stops the motor.

## Two ways to aim, and when to use each

| Macro | How it works | Use when |
|---|---|---|
| `alignToPollen()` | Plans a path to a computed heading | You want the follower's tuned motion profile |
| `servoAlignToPollen()` | Closed loop on raw `tx` | You need accuracy the pose estimate cannot give |

The path version is only as good as your localization and mount calibration. The servo version
watches the actual camera error and converges regardless of either — the right choice for the last
few degrees. It uses Pedro's `PIDFController` rather than a hand-rolled loop, so it behaves like the
rest of the robot and is tuned the same way. See lesson 7.

## Aborting

Two ways out, both wired in `Teleop`:

```java
if (macroRunning() && (gamepad1.backWasPressed() || driverWantsControl())) {
    abortMacro();
}
```

`driverWantsControl()` is any gp1 stick past 0.25 — grabbing the controls cancels, which is what a
driver does instinctively when something looks wrong.

Cancelling releases the Ivy resource, but **the Pedro follower drives itself** once handed a path. So
the abort also calls `robot.abortMacro()`, which calls `cancelPath()`. Without that the robot keeps
driving to its target while the sticks do nothing.

## Adding your own

1. Add a method to `Macros` returning a `Command`.
2. Start with `begin("name")`, end with `finish(...)`.
3. Wrap every wait in `race(..., waitMs(...))`.
4. Build vision paths with `followLazyCommand`.
5. Leave the camera on the AprilTag pipeline when you finish.
6. Bind it in `Teleop.handleDriverInput()` inside the `if (!macroRunning())` block.
7. Test the failure case first: run it with **no target visible** and confirm it times out and says so.

That last step is the one people skip.

## Keeping it game-agnostic

Nothing in `Macros` mentions a field coordinate. Everything is relative to what the camera currently
sees, so it works regardless of where the robot is standing.

Scoring routines that depend on knowing *where things are* belong in a separate class built on
`FieldConstants`, once the field is measured. Keeping that split means the macro engine stays useful
even when the game changes.

---

Next: [7. Control theory](07-control-theory.md)
