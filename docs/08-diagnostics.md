# 8. Diagnostics

The first seven lessons are about making the robot do something. This one is about knowing what it
actually did — which is the difference between fixing a problem and guessing at it.

> Everything here is in `util/diagnostics/`, plus `util/MatchClock`. All of it is unit tested,
> because none of it touches hardware.

## The problem with watching telemetry

Telemetry scrolls past at 50 Hz and nobody is reading it during a match — the drivers are driving.
So anything that only exists on screen effectively does not exist.

That gives three separate jobs, and they need three different tools:

| Question | Tool |
|---|---|
| What do the drivers need *right now*? | Match telemetry — four lines |
| What is this subsystem doing? | Debug telemetry — behind a toggle |
| What happened 40 seconds ago? | The CSV log |

`Teleop` splits the first two with `DEBUG_TELEMETRY`, toggled from gamepad 2. Match mode shows time,
possession, macro outcome, drive frame — and faults, which render *only when present*, so their
presence is itself the signal. No scanning a wall of numbers for the one that changed.

## Knowing what time it is

`MatchClock` answers "how long is left". It sounds trivial and the robot had no concept of it at all
until recently, which meant nothing could act on time — no endgame warning, no "is there room for
another cycle?".

```java
clock.getPhase();        // NOT_STARTED / RUNNING / ENDGAME / EXPIRED
clock.getRemainingMs();  // clamped at 0, never counts past the buzzer
clock.hasTimeFor(4000);  // the hook for a time-aware fallback
```

**It never reads the clock itself.** Timestamps are passed in, and on the robot they come from
`util/time/Clock`, which every subsystem shares. `Clock.system()` is monotonic (`System.nanoTime`),
not wall-clock time, so a time sync on the Control Hub cannot make a 200 ms dwell take an hour or
fire early. Tests inject `FakeClock` and advance it by hand:

```java
clock.start(System.currentTimeMillis());   // in start()
clock.update(nowMs);                       // every loop
```

That is what makes it testable — `MatchClockTest` runs a whole two-minute period in microseconds.
It is the same reason `VisionMath` and `PoseFusion` are separate from the subsystems that use them
(lesson 2), applied to time.

`hasTimeFor` returns **true** before the clock has started. A routine given no clock behaves exactly
as it did before the clock existed, rather than silently skipping everything.

## Measuring the loop

A robot that "feels laggy" is usually a loop that occasionally takes 200 ms. Printing the current
loop time will never show you that — the spike is one cycle and it is gone.

`LoopTimer` keeps the whole distribution:

```java
loopStats.record(loopMs);
loopStats.getStatus();   // "18.2 now / 19 p95 / 47.3 max / 3 spikes"
```

It counts samples into fixed millisecond buckets, so any percentile over the whole match costs
constant memory and no allocation in the loop. The trade is resolution — a percentile is accurate to
one millisecond — which is the right trade, because whether p95 is 21 or 21.4 ms changes nothing and
whether it is 21 or 210 ms changes everything. `getMax()` stays exact regardless, so a real stall is
never understated.

## Not doing everything every loop

Some work does not deserve 50 Hz. Redrawing the Panels field view builds and sends a network packet;
nobody can see fifty frames a second, and the time comes out of the control loop.

```java
if (drawLimiter.ready(nowMs)) onDraw();    // 10 Hz
```

`RateLimiter` is deliberately dull. Two details matter: the **first call always runs**, so nothing is
delayed at startup waiting for an interval that has not elapsed; and `ready()` **claims the slot**,
so calling it twice in one loop does not run the work twice.

Telemetry uses the SDK's own lever instead — `setMsTransmissionInterval` — because skipping
`addData` calls would blank the screen rather than slow it down. Pick the mechanism that matches what
you are trying to throttle.

## The log is the real record

`MatchLogger` writes one CSV row per loop to `/sdcard/FIRST/data/`. Twenty-five columns, in three
groups that each answer something telemetry cannot:

- **Target and error** (`path_completion`, `trans_error`, `heading_error`) — the difference between
  "the robot was here" and "the robot was tracking well". Pose alone cannot tell you that.
- **Identifiers** (`macro`, `macro_outcome`, `intake_mode`) — what the robot was *trying* to do.
  Without these a log shows motion with no intent behind it.
- **`battery_v` and `loop_ms`** — between them these explain most "it behaved differently that time"
  reports.

Two deliberate choices: it catches `Exception`, not just `IOException`, so a logging fault can never
end a match over a log line; and it flushes every 50 rows, because a `BufferedWriter` that is never
flushed loses the tail of the match, which is the interesting part.

Follower-derived columns write `NaN` rather than `0` when there is no follower — so a run on a robot
whose drivetrain failed to build reads as *missing data*, not as a robot that sat perfectly still
with zero error. Same instinct as `estimatePollenDistanceInches()` returning NaN in lesson 5.

## Reading it back

```bash
adb pull /sdcard/FIRST/data ./logs
python3 tools/analyze_log.py logs/teleop_20260904_191500.csv
python3 tools/analyze_log.py --plot logs/teleop_*.csv     # needs matplotlib
```

The summary answers the four questions worth asking after a run: did the loop keep up, was the
follower tracking, did the battery sag, and what did each macro attempt actually do.

```
Loop timing
  p95       19.0 ms
  max      145.0 ms
  over 100 ms  1 loops (0.11%)
  NOTE: loops over 100 ms mean the robot was blind and unresponsive that long.

Macros
  collectPollen        SUCCESS
  servoAlign           TIMED_OUT
```

## Try it

1. Run `Teleop` on a robot, drive for thirty seconds, stop.
2. Press gamepad 2 `BACK` mid-run and watch the telemetry change shape.
3. Pull the CSV and run the analyzer.
4. Find the loop-time max. If it is over 50 ms, something in the loop is not rate-limited — and now
   you have the evidence rather than a hunch.

---

Back to the [index](README.md).
