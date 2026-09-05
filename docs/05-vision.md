# 5. Vision

The Limelight 3A does two jobs, one per pipeline:

| Pipeline | Job | Output |
|---|---|---|
| 0 | AprilTags | Absolute field position (`botpose`) |
| 1 | Colour blobs | Bearing to a game piece (`tx`, `ty`) |

Switching takes several frames, so anything reading immediately after a switch gets the *old*
pipeline's data. That is what the warm-up waits in `Macros` are for.

Thresholds are tuned in the Limelight web UI at `http://limelight.local:5801`, not in code.

## From "the camera sees something" to "where it is"

The camera reports two angles from its crosshair: `tx` (positive right) and `ty` (positive up). Those
are *independent* angles, so the ray to the target in the camera's own frame is proportional to
`(tan tx, tan ty, 1)` along (right, up, forward).

The camera is mounted pitched down by `P`, so that ray gets rotated into the robot frame and then
intersected with the floor:

```
rX = cos P + tan(ty) * sin P      forward
rY = -tan(tx)                     left  (tx>0 is right, hence the sign)
rZ = tan(ty) * cos P - sin P      vertical; must be negative to reach the floor
```

Scale until it drops by the height difference and you have the answer. `util/math/VisionMath` does this.

```
        camera
          |\
          | \   P = pitch below horizontal
   height |  \
          |   \
          |____\________  floor
             forward
```

### The mistake this replaced

It is very tempting to treat `Δh / tan(P − ty)` as the *distance to the target* and split it into
components with `cos(tx)` and `sin(tx)`. It is not a distance — it is **already the forward
component**. Splitting it projects twice:

| tx | old (fwd, left) | correct | range error |
|---|---|---|---|
| 0° | (28.85, 0.00) | (28.85, 0.00) | 0% |
| 10° | (28.41, −5.01) | (28.85, −5.41) | −1.7% |
| 30° | (24.99, −14.43) | (28.85, −17.72) | **−14.8%** |
| 45° | (20.40, −20.40) | (28.85, −30.70) | **−31.6%** |

Exact on-axis, badly wrong at the edge of frame — which is precisely where a target is first spotted.
The bug survived a long time because pointing straight at something makes it disappear.

`VisionMathTest` checks the ray code against an independently derived closed form across a sweep of
`tx` and `ty`, so the two have to agree by geometry rather than by construction.

### The mount constants matter

```java
public static double CAMERA_HEIGHT_INCHES = 12.0;
public static double CAMERA_PITCH_DEGREES = 20.0;   // positive = tilted toward the floor
public static double CAMERA_FORWARD_OFFSET_INCHES = 6.0;
public static double CAMERA_LEFT_OFFSET_INCHES = 0.0;
public static double CAMERA_YAW_OFFSET_DEGREES = 0.0;
```

Measure them on the real robot. They are `@Configurable`, so you can tune them live on the Panels
dashboard while watching the `Blob range` telemetry against a tape measure — no redeploy.

**Sign warning:** our `CAMERA_PITCH_DEGREES` is positive *downward*. The official Limelight docs
define their mount angle positive *upward*. Copy a formula from a tutorial without checking and you
get a confidently wrong answer.

## Guards, and why each exists

`estimateBlobDistanceInches()` returns `NaN` — not `0` — when there is no usable reading, and
`estimateBlobApproachPose()` returns `null` rather than echoing the robot's own pose. Both matter:

- A `0` is indistinguishable from a legitimately computed zero, and would be driven to.
- Echoing the robot pose produces a zero-length path that silently does nothing.

Rejected cases:

| Case | Why |
|---|---|
| Target at or above the horizon | The ray never meets the floor; the old code returned a **negative** distance and put the target *behind* the robot |
| Near the horizon | Range runs away toward infinity |
| Beyond `MAX_VALID_DISTANCE_INCHES` | Not a real detection |
| Off the field | Arithmetic produced something impossible |

## One frame is not enough

A reflection, a half-occluded blob, a frame caught mid-exposure — any of these can produce a
confident, completely wrong reading. So detections are filtered before anything drives:

- Blobs are sorted **largest-area-first**, so "primary" is decided here rather than by an invisible
  sort setting in the web UI.
- `tx` and `ty` go through `MedianFilter`s.
- `hasStableBlob()` requires a **full window** whose **spread** is under a few degrees.

A median is used rather than an average on purpose: an average blends an outlier into the answer, a
median discards it. `MedianFilterTest` demonstrates both.

Gate motion on `hasStableBlob()`, not `seesBlob()`.

## AprilTags: the bug worth remembering

`getBotpose()` **never returns null and never returns NaN.** The SDK's JSON reader hands back
`new double[6]` when the key is absent, and a `Pose3D` is always built from it.

So on any valid *non-AprilTag* frame — a colour blob, for instance — the raw botpose is all zeros.
Add the 72-inch origin shift and you get `Pose(72, 72, 0)`: dead field centre, indistinguishable from
a real reading. The old null/NaN checks were dead code, and `init_loop()` ran this every iteration.

The real guard is asking how many tags contributed:

```java
if (currentPipeline != APRILTAG_PIPELINE_INDEX) return null;
if (latestResult.getBotposeTagCount() < 1) return null;
```

Plus a field-bounds check. `getStaleness()` catches the related failure where the camera drops offline
and `getLatestResult()` keeps returning the last frame forever.

## Fusing vision with odometry

Odometry is smooth but drifts. Vision is absolute but noisy and late. Writing a vision pose straight
into the follower teleports the robot's belief, and one bad frame sends the next path somewhere
arbitrary.

`util/field/PoseFusion` blends them with Pedro's `KalmanFilter`, applying three ideas:

1. **Gating.** Fixes outside the field, or further than `MAX_JUMP_INCHES` from the current estimate,
   are rejected outright.
2. **Latency compensation.** A fix describes where the robot *was* when the shutter opened, typically
   60–120 ms ago — several inches at speed. A short history of odometry samples lets us look up where
   we were at capture time and apply the *offset* to the present.
3. **Position only.** Heading passes through from odometry untouched, because the Pinpoint's fused
   IMU heading beats a tag-derived one, and a scalar filter straddling the 0/2π seam would average
   359° and 1° into 180° and point the robot backwards. Avoiding the seam beats patching it.

There is a contract: **the caller must write the returned pose back to the follower.** The per-loop
delta is measured against the pose the filter last handed out, so ignoring the return value causes
each correction to be re-counted as robot motion. `Robot.updateLocalization()` honours it.

Watch the `Localization` telemetry: `ACCEPTED`, `ODOMETRY_ONLY`, `REJECTED_JUMP`, `REJECTED_OFF_FIELD`.

---

Next: [6. Writing a macro](06-writing-a-macro.md)
