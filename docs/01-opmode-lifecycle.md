# 1. The OpMode lifecycle

An OpMode is the program the Driver Station runs. Ours extend `OpMode` (the *iterative* kind), which
means the SDK calls your methods repeatedly rather than you writing one long `while` loop.

## The five methods

```
  [ INIT pressed ]
        |
     init()            once
        |
    init_loop()        repeatedly, until START
        |
  [ START pressed ]
        |
     start()           once
        |
      loop()           repeatedly, ~50 times per second, until STOP
        |
  [ STOP pressed ]
        |
      stop()           once
```

| Method | Runs | Use it for |
|---|---|---|
| `init()` | once, on INIT | Build subsystems, schedule default commands |
| `init_loop()` | until START | Menus, AprilTag localization, showing warnings |
| `start()` | once, on START | Resetting edge detection, starting timers |
| `loop()` | ~50 Hz | Everything during the match |
| `stop()` | once | Closing files, stopping the camera |

Look at `opmodes/MatchOpMode.java` — it implements all five, marks them `final`, and exposes
hooks (`onInit`, `onInitLoop`, `onStart`, `onDecide`, `onTelemetry`, `onStop`) for `Teleop` and
`MainAuto` to fill in. Neither of those files overrides a lifecycle method directly.

## The loop order is load-bearing

Here is the match loop, in full. It lives in `opmodes/MatchOpMode.java`, and it is `final`:

```java
robot.readSensors();         // 1. observe
robot.updateLocalization();  //    blend any AprilTag fix into the pose estimate
onDecide();                  //    <- your OpMode: read buttons, schedule commands
Scheduler.execute();         // 2. decide
robot.writeActuators();      // 3. act
onAfterAct();                //    <- your OpMode: haptics, pose handoff
```

**Observe, decide, act — in that order.** It looks obvious written down, and it is the single easiest
thing to get wrong.

Which is why it is not left to you. `Teleop` and `Auto` do not write a loop at all; they fill in the
hooks. There is no ordering for a new OpMode to choose, and therefore none to get wrong.

This code used to call `Scheduler.execute()` *before* updating the sensors. Everything still ran, and
the robot still drove. But every command was deciding using sensor readings from the previous loop —
about 20 ms stale. At 40 in/s that is nearly an inch of error on every decision, and the bug is
invisible because nothing crashes.

That is why `Robot` has two methods instead of one:

- `readSensors()` — clears the hub's bulk cache, refreshes the Limelight and colour sensor, ticks the
  match clock, and samples the battery
- `writeActuators()` — pushes the intake and lift, then ticks the Pedro follower

There is deliberately no `update()` that does both. A single method would let a caller put the whole
observe-and-act pair on one side of the scheduler, which is exactly the bug this split prevents.

## Why sensors are read once per loop

`readSensors()` starts with:

```java
for (LynxModule hub : hubs) hub.clearBulkCache();
```

Every encoder or current reading is normally a separate USB round-trip to the hub. With bulk caching
in `MANUAL` mode, the first read of a loop fetches *everything* in one transaction and the rest come
from memory until you clear the cache again.

That matters more than it sounds. The intake's current is read three times per loop — once by the
anti-jam logic, once by telemetry, once by the match logger. Without caching that is three round
trips for one number.

It also means every part of the loop sees the *same* snapshot. Two pieces of code asking "where is
the robot?" in the same loop get the same answer.

## Edge detection, and the trap in it

To act once per button press rather than 50 times per second, use the SDK's one-shots:

```java
if (gamepad1.aWasPressed()) { ... }     // fires exactly once per physical press
```

Never `if (gamepad1.a)`, which is true for every loop the button is held.

**The trap:** `aWasPressed()` *latches*. It sets a flag on the press and only clears it when you read
it. Our `init_loop()` reads none of the buttons — so anything a driver leans on during the two
minutes of init stays latched, and every one of them fires simultaneously on the first loop after
START. Including `A`, which launches a vision macro.

The fix is one line in `start()`:

```java
gamepad1.resetEdgeDetection();
gamepad2.resetEdgeDetection();
```

## `stop()` cannot move motors

The SDK forbids iterative OpModes from commanding actuators in `stop()`. Attempts are ignored and
logged as `CANCELLED_FOR_SAFETY`. It zeroes the motors for you anyway.

So `Robot.stop()` only releases things the SDK does not handle — it stops the Limelight streaming and
turns off the colour sensor's LED. `SelfTest` needs its motors genuinely stopped mid-OpMode, so it
calls `intake.stop()` followed by `writeActuators()` itself.

## Try it

Open `Concept: Commands` on the Driver Station. Press and hold `A` during init, then press START, and
watch what happens on the first loop.

---

Next: [2. Subsystems](02-subsystems.md)
