# 2. Subsystems

A subsystem owns one piece of hardware and everything about how to use it. `Robot` owns the
subsystems. OpModes own nothing — they translate buttons into requests.

```
        Teleop / Auto / SelfTest        <- buttons and routines
                  |
                Robot                   <- composition, cross-wiring, loop fan-out
                  |
   Drivetrain  Limelight  ColorSensor  Intake    <- one piece of hardware each
```

## The rule that makes subsystems work

**Commands set intent. `update()` writes hardware.**

Look at `Intake.java`. Calling `intake()` does *not* touch the motor:

```java
public void intake() {
    setMode(Mode.INTAKING, INTAKE_TICKS_PER_SEC);   // just sets fields
}
```

The motor is written in exactly one place, once per loop:

```java
public void update() {
    ...
    motor.setVelocity(targetVelocity);   // the only setVelocity in normal operation
}
```

Three things fall out of this:

1. **No fighting.** However many commands ask for different things, the last request before
   `update()` wins, deterministically. Two commands cannot half-apply opposite velocities.
2. **Anti-jam is possible at all.** `update()` sees the final request and can override it on the way
   out. There is no equivalent hook if callers write the motor directly.
3. **The subsystem is testable in principle** — you can drive it through a sequence of requests and
   inspect the resulting state without hardware attached.

The consequence to remember: `intake()` then immediately reading `getVelocityTicksPerSec()` reads the
*previous* command's speed. `SelfTest` had this exact bug — it slept before `writeActuators()` rather
than after, so it sampled a motor that had never been told to move, and reported a healthy intake as
broken.

## Missing hardware must not kill the OpMode

Every lookup goes through `util/hardware/Hardware`:

```java
motor = Hardware.get(hardwareMap, DcMotorEx.class, name);
if (motor == null) return;      // constructor gives up quietly
```

`hardwareMap.get()` throws when a name is wrong. Since subsystems are built in `Robot`'s constructor,
one typo in the robot configuration used to take down the whole OpMode before it ran — meaning a
broken colour sensor cost you the entire driver-controlled period.

Now the name is recorded, `isAvailable()` returns false, every method no-ops, and `SelfTest` and
`Teleop` both print exactly which config entry is wrong.

`Robot`'s constructor calls `Hardware.reset()` first, because the registry is static and would
otherwise accumulate entries across OpMode runs.

The names themselves live in `util/hardware/HardwareNames`:

```java
public Intake(HardwareMap hardwareMap) {
    this(hardwareMap, HardwareNames.INTAKE_MOTOR);
}
```

They used to be literals in each constructor, which meant four files to open to answer "what does
the configuration need to be called?" — and four to edit when someone renamed a device in the
configuration app at a competition. The overloaded constructors taking an explicit name still exist,
so a second robot with a different config is still possible.

## The `util/` rule pays for itself

Lesson 1 said math belongs in `util/` so it can be tested. The intake's anti-jam logic is the case
that proves it.

It used to live inside `Intake`, which holds a `DcMotorEx` — so none of it could be tested without a
robot, a real jam, and a stopwatch. Moving it to `util/JamDetector` (pure state and arithmetic, time
passed in as an argument) made all twelve cases testable, and the first test run immediately failed:

```java
private long stallStartMs = 0;   // 0 also means "not stalling"
```

Zero is a perfectly good timestamp. A stall beginning at `t = 0` restarted its own dwell every loop
and never fired. On a robot that never showed, because `System.currentTimeMillis()` is never zero —
it took a test starting the clock at zero to expose it.

**A sentinel value that overlaps the real range is a bug waiting for the right input.** Use a
separate flag.

## Caching sensor reads

`Limelight` and `ColorSensor` read once in `update()` and store the result. Getters return the cached
value:

```java
public void update() {
    colors = sensor.getNormalizedColors();
    ColorMath.toHsv(colors.red, colors.green, colors.blue, hsv);
}

public float getHue() { return hsv[0]; }   // free, and consistent all loop
```

Getters stay cheap enough to call from telemetry, and two callers in the same loop always agree.

Note `getHsv()` returns a **copy**. It used to hand out the internal array, which `update()`
overwrites in place — so a caller could corrupt the subsystem's state, and anyone holding the
reference silently saw it change underneath them.

## Capability checks

Not every colour sensor has a switchable light; not every one has a distance sensor. Rather than
assuming, ask:

```java
public boolean hasLight()  { return sensor instanceof SwitchableLight; }
public void setLight(boolean on) {
    if (sensor instanceof SwitchableLight) ((SwitchableLight) sensor).enableLight(on);
}
```

The same wrapper then works across REV V1/V2/V3 and AndyMark parts without crashing on the ones that
lack the feature.

## Cross-wiring belongs in `Robot`

`Intake` needs to know when a game piece has been captured. That is a colour-sensor question — but
making `Intake` import `ColorSensor` welds two subsystems together for one boolean.

Instead, `Intake` accepts a supplier and `Robot` provides it:

```java
// Intake
private BooleanSupplier capturedSupplier = () -> false;
public void setCapturedSupplier(BooleanSupplier supplier) { ... }

// Robot's constructor
intake.setCapturedSupplier(this::pollenAtColorSensor);
```

`Intake` stays testable with a fake supplier, and swapping to a distance sensor or a beam break later
is a one-line change in `Robot`.

## Adding a subsystem: checklist

1. Look up hardware with `Hardware.get`, and add `isAvailable()`.
2. Store intent in fields; write hardware only in `update()`.
3. Cache sensor reads in `update()`; return copies of mutable state.
4. Put any real math in a `util/` class so it can be unit tested.
5. Expose `*Command()` factories with `requiring(this)` — see lesson 3.
6. Add it to `Robot` as a `public final` field, and call its `update()` from `readSensors()`
   (sensors) or `writeActuators()` (actuators).
7. Mark tunable constants `public static` and annotate the class `@Configurable`.

## Mechanisms that move between fixed positions

Most mechanisms are a small state machine — a lift is DOWN, LOW or HIGH. `templates/` covers this so
you do not write it again: `PositionalMotor<S>` for an encoder motor, `PositionalServo<S>` for a
servo, both implementing `Mechanism<S>`.

```java
public enum Level { DOWN, LOW, HIGH }

lift = new PositionalMotor<>(hardwareMap, HardwareNames.LIFT_MOTOR, Level.class)
        .preset(Level.DOWN, 0)
        .preset(Level.HIGH, 1200)
        .limits(0, 1250);

lift.goTo(Level.HIGH).schedule();
```

The states are an **enum, not strings**, so a typo fails at compile time instead of silently doing
nothing mid-match.

`templates/ExampleLift.java` combines a motor and a servo into one subsystem, and it is **wired into
`Robot` like any other**. There is no lift on the robot right now, so `Hardware.get` returns null,
`isAvailable()` is false, and every call no-ops — but the wiring is real. Bolt on a lift, add the two
names to `HardwareNames`, and it works.

That is deliberate. A pattern nothing uses is a pattern nobody trusts: you would have had to guess
whether it actually fits the rest of the codebase. Now you can read `Robot`, `Teleop` and
`ExampleLift` together and see the whole path from a button to a mechanism.

**The same trick works for you.** A subsystem guarded by `isAvailable()` can be written, wired and
committed before its hardware exists, which means the software is not the thing holding up the first
test.

---

Next: [3. Commands](03-commands.md)
