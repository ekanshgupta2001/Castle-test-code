# 7. Control theory

Everything here is about one question: **the robot is not where I want it — how hard should I push?**

You do not need to write any of these from scratch. Pedro ships production implementations in
`com.pedropathing.control`, and this codebase uses them. This lesson is about knowing which to reach
for and why.

## Open loop vs closed loop

**Open loop** — command something and hope:

```java
servo.setPosition(0.65);      // no feedback exists; you wait and assume
```

**Closed loop** — measure the error and correct it:

```java
controller.updatePosition(limelight.getFilteredPollenTx());   // where are we?
double turn = -clamp(controller.run(), MAX_TURN);             // how hard do we push?
```

Use open loop when there is no sensor (servos) or the hardware closes the loop for you
(`RUN_TO_POSITION` runs a controller inside the REV hub). Use closed loop when you can measure the
thing you actually care about.

## PID, in the only terms you need

Error is `target − actual`. The output is a sum of three terms:

| Term | Responds to | Effect | Too much causes |
|---|---|---|---|
| **P** | error now | Push proportional to how wrong you are | Oscillation |
| **I** | error accumulated | Grinds out small steady offsets | Slow wobble, overshoot after a stall |
| **D** | how fast error is changing | Damps the approach | Jitter on noisy sensors |
| **F** | — | Constant feed-forward, e.g. gravity on an arm | — |

Tune in that order: P until it responds crisply and oscillates slightly, D until the oscillation
damps, and I only if a steady error remains. Most FTC mechanisms need P and a little D. **I is where
most teams cause themselves problems** — it winds up while a mechanism is stalled or saturated, then
dumps all of it at once.

### Units decide your gains

The vision servo's error is in **degrees** and its output is a turn command in **[-1, 1]**:

```java
public static double ALIGN_P = 0.02;    // 10 degrees of error -> 0.20 turn power
```

`P = 0.02` looks tiny until you notice it maps a 10° error to 20% power. Always sanity-check a gain
by multiplying it by a typical error and asking whether the answer is a sensible output.

### Heading hold: a loop you feel every match

`Drivetrain.applyHeadingHold` is the closed loop drivers notice most. A mecanum robot does not track
straight on its own — uneven friction, a knocked wheel, or contact with another robot all rotate it —
so without correction the driver spends the match nudging the turn stick to stay pointed where they
already were.

```java
double error = Angles.angleError(pose.getHeading(), heldHeading);
headingController.updateError(error);
double turn = clamp(headingController.run(), HEADING_HOLD_MAX_TURN);
```

Two details make it work rather than fight you:

- **The error is fed in directly**, via `updateError`, not as a position. The controller never sees a
  raw angle, so the 0/2π seam cannot make it spin the long way round chasing a 359°-to-1° "error" of
  358°. `angleError` always takes the short way.
- **Any deliberate turn releases it**, and the heading is re-captured when the stick is let go. Hold
  the setpoint through a driver's turn and the robot fights back the moment they stop — which is
  exactly the behaviour that makes people switch assists off and never turn them on again.

Gains are in radians here (`P = 1.5` maps 10° ≈ 0.17 rad to ~0.26 turn power), while the vision servo
below is in degrees. Same technique, different units, so **the numbers are not comparable** — which is
the whole point of the sanity check in the next section.

### Always clamp and always have a tolerance

```java
double turn = -clamp(controller.run(), ALIGN_MAX_TURN);
...
.setDone(() -> Math.abs(limelight.getFilteredPollenTx()) <= ALIGN_TOLERANCE_DEGREES)
```

Without a clamp, a large error commands full power and the robot lurches. Without a tolerance, `done`
never fires because the error never reaches exactly zero — which is why the macro also has a timeout.

## Filtering: three tools, three jobs

| Tool | Job | Where we use it |
|---|---|---|
| **Median** | Throw away outliers | Vision `tx`/`ty` — `MedianFilter` |
| **Low-pass** | Smooth continuous noise | (available: Pedro `LowPassFilter`) |
| **Kalman** | Combine two sources of different quality | Odometry + AprilTags — `PoseFusion` |

The choice matters. Vision fails by being *completely* wrong occasionally, not slightly wrong
constantly — a reflection, a half-occluded blob. An averaging filter blends that outlier into the
answer; a median discards it. `MedianFilterTest` shows the same data through both.

A low-pass filter is the right tool for a genuinely noisy continuous signal, like a current draw
reading. Reach for it there, not for detections.

## Sensor fusion

Two estimates of the same thing, with opposite weaknesses:

| | Strength | Weakness |
|---|---|---|
| Odometry | Smooth, always available, fine short-term | Drifts without bound |
| AprilTags | Absolute, no drift | Noisy, intermittent, **late** |

A Kalman filter tracks how much it trusts its own estimate and weights each new measurement
accordingly. Pedro's takes a motion increment and an absolute measurement:

```java
filterX.update(dx, measuredX);   // dx = odometry motion, measuredX = vision
```

With no vision this loop, feed the prediction back as the measurement so the correction term is zero
and it simply integrates odometry.

Three design decisions in `PoseFusion` worth understanding:

**Position is fused; heading is not.** The Pinpoint's fused IMU heading is much better than a
tag-derived one, and angles wrap at 0/2π — a scalar filter straddling that seam would average 359°
and 1° into 180° and point the robot backwards. Avoiding the problem beats patching it.

**Latency is compensated, not ignored.** A fix describes where the robot was when the shutter opened,
60–120 ms ago. At 40 in/s that is several inches. We keep a short history of odometry samples, look up
where we were at capture time, and apply the vision-minus-history *offset* to the present. There is a
test showing the same fix pulling the estimate backwards without this.

**Bad data is rejected before it is filtered.** Fixes off the field, or further than
`MAX_JUMP_INCHES` away, are dropped entirely. A filter smooths noise; it does not protect you from a
measurement that is simply wrong.

## Motion profiling

Pedro handles this for path following — it plans a velocity curve that accelerates, cruises, and
decelerates rather than commanding a step change. That is why `followPath` produces smooth motion
rather than a lurch.

`PredictiveBrakingController` is also in `com.pedropathing.control` if you need to tune stopping
behaviour specifically.

## Feed-forward

Feedback reacts to error that has already happened. Feed-forward predicts the effort needed and
applies it up front — most usefully gravity compensation on an arm:

```java
double gravity = ARM_KG * Math.cos(armAngleRadians);   // maximum when horizontal
```

Add it to the PID output. The controller then only has to correct the *difference* between prediction
and reality, which is a much smaller job. Pedro's `PIDFController` has an `F` term and
`updateFeedForwardInput()` for this.

## Where to look

| Class | Package | Use |
|---|---|---|
| `PIDFController` | `com.pedropathing.control` | General feedback control |
| `FilteredPIDFController` | same | PID with a filtered derivative — better on noisy sensors |
| `KalmanFilter` | same | Sensor fusion |
| `LowPassFilter` | same | Smoothing a continuous signal |
| `PredictiveBrakingController` | same | Stopping behaviour |
| `MedianFilter` | `teamcode.util.math` | Outlier rejection |
| `Angles` | `teamcode.util.math` | Wrap-safe angle error — see heading hold above |
| `PoseFusion` | `teamcode.util.field` | Odometry + vision, latency-compensated |

## Practice

1. Open the Panels dashboard and run `servoAlignToPollen` with a target in view.
2. Raise `ALIGN_P` until the robot visibly oscillates around the target. That is too much P.
3. Add `ALIGN_D` until it settles cleanly.
4. Drop P until it becomes sluggish, so you have felt both failure modes.

You can do all of that live, without redeploying, because the gains are `@Configurable`.

---

Next: [8. Diagnostics](08-diagnostics.md)
