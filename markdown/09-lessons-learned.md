# 9. Lessons learned

Every rule in this codebase exists because something went wrong. The lessons before this one keep
the *rule*; this page keeps the *story*, so the code comments can stay short and present-tense
and the reasons do not rot when nobody remembers the old version.

Read this when a rule looks like ceremony. It probably is not.

| What went wrong | The rule it left behind | Where it lives now |
|---|---|---|
| The scheduler ran before the sensors were read, so every command decided on one-loop-stale data. Nothing crashed. | Observe, decide, act, in that order. | `MatchOpMode.loop()` is `final` (lesson 1) |
| `Teleop` and `Auto` each hand-wrote the same twelve lifecycle steps. They had already drifted: only one set the telemetry transmission interval. | One lifecycle, filled in through hooks. | `MatchOpMode` |
| The gamepad bindings were written twice, once in the input handler and once as help-card text. A re-bound button left the card lying to the drivers. | Button, label and description are one object; the card renders from it. | `Controls` |
| The sticks were read straight from `gamepad1` fields in `Teleop`, with the SDK's stick-up-is-negative sign applied inline each time. | Analog inputs are enum entries too; the sign convention is applied in exactly one place. | `Controls.axis()` |
| One misnamed device in the configuration made `hardwareMap.get()` throw from `Robot`'s constructor and kill the whole OpMode. A broken colour sensor cost the driver-controlled period. | A missing device degrades one subsystem, never the OpMode. | `util/hardware/Hardware`, `isAvailable()` on every subsystem (lesson 2) |
| Configuration names were string literals in four files. Renaming a device at a competition meant finding all four. | Every config name in one file. | `HardwareNames` |
| `Teleop` and `SelfTest` each walked `hardwareMap.voltageSensor` every loop, and voltage reads are not bulk-cached. | Resolve the sensors once; sample them on a timer. | `Robot.sampleBattery()` |
| Angle wrap arithmetic lived in `VisionMath`, so heading hold and field mirroring imported a "vision" class to do arithmetic about angles. | Name a class for what it contains. | `util/math/Angles` |
| `Drawing` was a package-private class inside the Pedro tuning menu, so only tuning OpModes could draw on the dashboard. | Shared tools live in `util/`. | `util/diagnostics/Drawing` |
| The match log was a `BufferedWriter` that only reached disk when its 8 KB buffer filled. An abnormal exit lost the tail of the match, which is the interesting part. | Flush every 50 rows. | `MatchLogger` |
| The anti-jam logic lived inside `Intake`, next to a `DcMotorEx`, so none of it could be tested. When it was extracted, the first test run failed: `stallStartMs = 0` doubled as "not stalling", so a stall beginning at t = 0 restarted its own dwell forever. It never showed on a robot because `System.currentTimeMillis()` is never zero. | Pure logic goes in `util/`; a sentinel must not overlap the valid range. | `util/control/JamDetector`, explicit `stalling` / `unjamming` flags |
| Six places read `System.currentTimeMillis()` on their own. Nothing that read the clock could be tested without real sleeps, and wall-clock time is not monotonic. | One injected, monotonic time source. | `util/time/Clock` (lesson 8) |
| `collectPollen` reported SUCCESS when the *path* finished, so a perfect drive that picked up nothing was a success. Pressing it while already carrying a piece was an instant success that did nothing. | Success means the thing you actually wanted, measured after the fact, against a snapshot taken at the start. | `Macros.capturedSomethingNew()` (lesson 6) |
| An unbounded `waitUntil` for a target that never appeared held the drivetrain and intake for the rest of the match, silently. | Every wait is `race(work, waitMs(limit))`. | `Macros` (lesson 3) |
| `SelfTest` slept *before* `writeActuators()`, so it sampled a motor that had never been told to move and reported a healthy intake as broken. | Commands set intent; `update()` writes hardware; sample after the write. | `SelfTest.checkIntake()` (lesson 2) |
| The Limelight blob list was copied and sorted every loop to find the largest, for an ordering nothing else read. | Pick the largest in one pass; no per-loop allocation. | `Limelight.update()` |
| **Sept 2026 review.** `holdEnd = true` never held: Pedro clears `isBusy` as the hold begins, the command finished, and its `setEnd` cancelled the path. The robot free-wheeled at the score pose while ejecting. | A natural end while asked to hold leaves the follower holding; only interruption cancels. | `Drivetrain.followLazyCommand`, `DrivetrainCommandTest` (lesson 4) |
| **Sept 2026 review.** `resetHeading()` rewrote the pose but left the heading hold's setpoint, so the PID spun the robot back by its old heading. `relocalize()` did the same underneath driver control. | Any pose write releases the heading hold; relocalize holds the drivetrain. | `Drivetrain.setPose()`, `Macros.relocalize()` |
| **Sept 2026 review.** Three `Concept: Commands` demos called the intake directly from an `instant` that did not require it. Ivy starts a scheduled command immediately, then the idle command's `execute()` re-asserted stop before the write. The motor never moved. The left bumper was also bound twice, so one demo could never fire. | Go through a command that requires the subsystem, or make the group own it explicitly. | `Intake` command factories, `AutoRoutine.build()`, demo 9 (lesson 3) |
| **Sept 2026 review.** `cancelPath()` called `startTeleopDrive()` directly and never set the teleop flag, so driver control called it again next tick and the follower was ticked three times in one loop. | State flags are set where the state changes, and commands change no state at build time. | `Drivetrain.cancelPath()`, `turnToCommand()` |
| **Sept 2026 review.** `docs/` was in `.gitignore`. Nobody who cloned the repo could read a single lesson, and forty files of refactor sat uncommitted. | Documentation is code: tracked, reviewed, built by CI. | `.github/workflows/teamcode.yml` |

## The pattern in the pattern

Almost every row is one of three things:

1. **Something hardware-shaped was untestable**, so the bug hid until a test forced it out.
   Answer: split the logic from the hardware (`util/`, injected `Clock`, `PathFollower`).
2. **The same fact lived in two places** and they drifted. Answer: one home per idea (`Controls`,
   `HardwareNames`, `MatchOpMode`, `game/`).
3. **A library did something reasonable that the code assumed it did not** (`isBusy` false while
   holding, `start()` at schedule time, `NOOP` never finishing). Answer: read the source, then write
   the test that pins the behaviour so the next upgrade cannot silently change it.

When you find the next one, add a row here, fix the rule at its one home, and write the test.

---

Back to the [index](README.md).
