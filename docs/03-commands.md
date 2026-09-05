# 3. Commands

A command is a small object with four parts: what to do when it starts, what to do each loop, how to
tell when it is finished, and how to clean up. The **Scheduler** runs all the active ones, once per
loop, and sorts out who gets which subsystem.

We use [Ivy](https://github.com/Pedro-Pathing), which ships with Pedro Pathing.

> Run `Concept: Commands` on the Driver Station while reading this. One button per idea.

## Why not just write `if` statements

You can drive one motor with an `if`. Trouble starts when two things want the same motor, or when an
action takes several seconds and the robot has to keep driving meanwhile.

The usual answer is a state machine with an `enum` and a `switch`, and it works — until there are
nine states and someone forgets a transition. Commands let you write the *sequence* directly and get
the arbitration for free.

## Building one

```java
Command spin = Command.build()
        .setStart(() -> motor.setPower(1))     // once, when it begins
        .setExecute(() -> telemetry.log())     // every loop while running
        .setDone(() -> timer.seconds() > 2)    // true = finished
        .setEnd(ec -> motor.setPower(0))       // always runs, however it ends
        .requiring(intakeSubsystem);           // what it needs exclusive use of
```

Every part is optional. Two rules worth internalising:

- **`setDone(() -> false)` never finishes on its own.** It runs until something interrupts it. That
  is right for "hold this until told otherwise", and a bug everywhere else.
- **`setEnd` runs on interruption too.** Put cleanup there, not after `setDone` returns true — a
  command that gets preempted never reaches that point.

## Groups

| Group | Ends when | Use for |
|---|---|---|
| `sequential(a, b, c)` | all done, in order | step-by-step routines |
| `parallel(a, b)` | **all** finish | independent things at once |
| `race(a, b)` | **first** finishes | timeouts |
| `deadline(timekeeper, a, b)` | the **first argument** finishes | "do X while Y happens" |

`race` is how every timeout in this codebase is built:

```java
race(
    waitUntil(limelight::hasStablePollen),   // the thing we want
    waitMs(SEARCH_TIMEOUT_MS)                // the thing that saves us
)
```

**Every wait in this repo has a timeout.** An unbounded `waitUntil` whose condition never comes true
holds its subsystems forever, which silently disables the default commands for the rest of the match.
`collectPollen` had exactly that bug: if no game piece was ever visible, the intake was locked out
permanently and nothing said why.

`deadline` is subtler. The *first* argument is the timekeeper; the rest are cancelled the moment it
finishes:

```java
deadline(
    drivetrain.followLazyCommand(this::buildPollenPath, false),  // decides when we're done
    intake.captureAndHoldCommand()                               // runs alongside, then cancelled
)
```

Drive to the piece while intaking; when the path ends, the intake command is cancelled and its
`setEnd` stops the motor.

## Requirements: how conflicts get resolved

`requiring(this)` declares that a command needs exclusive use of a subsystem. Two commands requiring
the same subsystem cannot run at once — the Scheduler decides which wins, using priority.

This is the mechanism behind **default commands**:

```java
public Command defaultIdleCommand() {
    return Command.build()
            .setExecute(() -> { if (hasPollen) hold(); else stop(); })
            .setDone(() -> false)
            .setPriority(-1)                                     // lower than anything else
            .setInterruptedBehavior(InterruptedBehavior.SUSPEND) // park, don't die
            .setBlockedBehavior(BlockedBehavior.QUEUE)
            .requiring(this);
}
```

Schedule it once in `init()` and it runs whenever nothing else wants the intake. Any normal command
(priority 0) preempts it; when that command ends, the idle command resumes on its own.

Three details worth knowing, each of which was a real bug here:

- **`SUSPEND`, not `END`.** Suspended commands are parked and resumed. An ended one is gone for good.
- **The logic is in `setExecute`, not `setStart`.** The Scheduler's resume path re-adds the command
  *without* calling `start()` again, so start-only logic would never run after the first suspension.
- **`BlockedBehavior.QUEUE`.** At priority `-1` the default (`CANCEL`) means that if anything already
  holds the intake when this is scheduled, it is dropped silently for the whole match. It worked only
  because `Teleop` happened to schedule it first.

## Driving is a command too

`Drivetrain.driverControlCommand()` is a default command exactly like the one above — it reads the
sticks in `setExecute` and requires the drivetrain.

That is what makes macros cancellable for free. Scheduling `collectPollen()` — which requires the
drivetrain — suspends driver control. When the macro ends or is cancelled, driver control resumes by
itself. There is no "am I in a macro?" flag for the loop to get wrong.

**The one thing Ivy cannot do for you:** stop the Pedro follower. Once handed a path, the follower
drives itself; cancelling the command only releases the *resource*. That is why every path command
carries `setEnd(ec -> cancelPath())`. Forgetting it means a cancelled macro keeps driving into a wall
while the driver's sticks do nothing.

## The `unless()` trap

Ivy gives every command two decorators that look like the obvious way to skip work:

```java
command.until(() -> condition)    // fine
command.unless(() -> condition)   // do NOT use this inside a sequential
```

`unless` is implemented as `conditional(condition, Command.NOOP, this)`. And `Command.NOOP` is
`Command.build()` — a command with **no `setDone`**, so it inherits Ivy's default done-supplier,
which is `() -> false`.

A command whose `done()` is always false never finishes. So on the skip path, `unless` hands the
scheduler a command that runs forever, and the `sequential` containing it blocks for the rest of the
match. The skip you asked for becomes a hang.

Skip with an explicit no-op instead:

```java
conditional(() -> shouldRun, realCommand, instant(() -> { }))
```

`instant()` sets `done` to `true`, so it completes on its first tick. `MainAuto.skipIfAnyLegMissed`
is the real use of this.

The general lesson is worth more than the specific bug: **`setDone` is not optional.** Any command
built without one runs until something interrupts it.

## Scheduler API

```java
Scheduler.reset();               // clear everything — call in init()
command.schedule();              // or Scheduler.schedule(command)
Scheduler.execute();             // once per loop
Scheduler.cancel(command);
Scheduler.isScheduled(command);  // running, queued, or suspended
```

`Scheduler` is **static**, so its state outlives an OpMode. Always `reset()` in `init()` or you
inherit whatever the last OpMode left running. Note `reset()` clears the queues *without* calling
`end()` on running commands — so if cleanup matters, cancel individually.

## Building a command that depends on runtime state

A command is built once and may run much later. Anything decided at build time is frozen:

```java
// WRONG — target is captured when the command is BUILT
drivetrain.followPathCommand(pathTo(limelight.getTarget()), false)

// RIGHT — the supplier runs when the command STARTS
drivetrain.followLazyCommand(() -> pathTo(limelight.getTarget()), false)
```

This bites hardest with vision. See lesson 5.

---

Next: [4. Pedro paths](04-pedro-paths.md)
