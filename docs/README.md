# Team Code Guide

This is the guide to *our* robot code. The 130 KB `README.md` at the repo root is the stock FTC SDK
release notes — useful occasionally, not what you want now.

## If you have 5 minutes

1. Open `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/MatchOpMode.java`.
2. Find `loop()`. Everything the robot does during a match goes through there.
3. Notice it is five lines — read sensors, decide, run the scheduler, write actuators — and that it
   is `final`, so no OpMode can reorder them.

That ordering is the single most important idea in this codebase. Lesson 1 explains why.

Then open `Teleop.java` and notice what is *not* in it: no loop, no logger, no lifecycle. Only the
things that make it teleop.

## Read these in order

| # | Lesson | What you'll be able to do |
|---|---|---|
| 1 | [OpMode lifecycle](01-opmode-lifecycle.md) | Know when your code runs, and why loop order matters |
| 2 | [Subsystems](02-subsystems.md) | Add a new mechanism to the robot |
| 3 | [Commands](03-commands.md) | Make several mechanisms cooperate without a state machine |
| 4 | [Pedro paths](04-pedro-paths.md) | Make the robot drive itself to a point on the field |
| 5 | [Vision](05-vision.md) | Turn "the camera sees something" into "the robot drives to it" |
| 6 | [Writing a macro](06-writing-a-macro.md) | Build a new one-button action, safely |
| 7 | [Control theory](07-control-theory.md) | Understand the feedback loops, and when to reach for one |
| 8 | [Diagnostics](08-diagnostics.md) | Find out what the robot actually did, instead of guessing |

Each lesson points at real files in this repo. None of the code in them is invented for the lesson.

## Running things

```bash
./gradlew :TeamCode:assembleDebug     # build the app
./gradlew :TeamCode:test              # run unit tests — no robot needed

adb pull /sdcard/FIRST/data ./logs    # get match logs off the robot
python3 tools/analyze_log.py logs/teleop_*.csv          # summarise a run
python3 tools/analyze_log.py --plot logs/teleop_*.csv   # and plot it
```

If Gradle fails with **"Unsupported class file major version"**, your default Java is too new
for Gradle 8.9 (it runs on JDK 17 to 22). Use the one Android Studio ships with, either per command:

```bash
JAVA_HOME="/Applications/Android Studio.app/Contents/jbr/Contents/Home" ./gradlew :TeamCode:test
```

or once, so plain `./gradlew` works from then on, by adding this line to
`~/.gradle/gradle.properties` (your user file, not the one in the repo):

```properties
org.gradle.java.home=/Applications/Android Studio.app/Contents/jbr/Contents/Home
```

A Java toolchain block in the build would not help here: toolchains pick the JDK that compiles
the code, not the JDK Gradle itself runs on. CI pins Temurin 17 for the same reason.

Deploying to the robot is the green Run button in Android Studio.

## OpModes on the Driver Station

| Name | Group | Purpose |
|---|---|---|
| `Teleop` | Main | The match OpMode |
| `Auto` | Main | The autonomous routine |
| `SelfTest` | Diagnostics | **Run this first at every event.** Reports missing hardware by name |
| `Concept: Commands` | Concept | Teaching OpMode — one button per command concept |
| `Tuning` | Pedro Pathing | The Pedro tuning menu |

## Before this robot can drive itself

Two things are unfinished, and everything path-related depends on them:

1. **`pedroPathing/Constants.java` is default-constructed.** No drivetrain, no localizer, no tuned
   gains. Pedro cannot follow a path until this is filled in and the tuning procedure is run.
2. **`util/field/FieldConstants.java` holds placeholder coordinates.** The alliance-mirroring math is real
   and tested; the numbers are guesses. Measure the field and replace them.

Teleop driving, the intake, vision detection, and `SelfTest` all work without either.

## House rules

- **Hardware lookups go through `util/hardware/Hardware`.** A missing device must degrade one subsystem, not
  kill the OpMode. A vision failure should never cost you the driver-controlled period.
- **Every wait gets a timeout.** An unbounded `waitUntil` holds its subsystem forever when its
  condition never becomes true, silently disabling default commands for the rest of the match.
- **Math goes in `util/`, not in a subsystem.** Anything that touches hardware cannot be unit tested.
  That is the only reason `VisionMath` and `ColorMath` are separate classes — and it is why the trig
  bug in the old vision code went unnoticed for so long.
- **Name states with enums, not strings.** A typo should be a compile error, not a robot that does
  nothing during a match.
- **Hardware names live in `util/hardware/HardwareNames`.** One place to look, one line to change when a
  device gets renamed in the configuration app the morning of a competition.
- **Nothing runs every loop unless it has to.** Telemetry transmission, dashboard drawing and the
  battery sample are all rate-limited. The control loop should not pay full price for a display.
- **One idea, one home.** Field size lives in `FieldConstants`, config names in `HardwareNames`,
  bindings in `Controls`, the match lifecycle in `MatchOpMode`. If you are about to write something
  that already exists somewhere else, wire the two together instead.
