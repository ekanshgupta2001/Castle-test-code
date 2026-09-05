package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.field.FieldConstants;
import org.firstinspires.ftc.teamcode.util.hardware.Hardware;
import org.firstinspires.ftc.teamcode.util.hardware.HardwareNames;
import org.firstinspires.ftc.teamcode.util.math.MedianFilter;
import org.firstinspires.ftc.teamcode.util.math.Angles;
import org.firstinspires.ftc.teamcode.util.math.VisionMath;

import java.util.Collections;
import java.util.List;

/**
 * Wraps the Limelight 3A. Two jobs, one per pipeline: AprilTag localisation, and a colour-blob
 * detector for whatever the season's game piece is.
 *
 * <p>Game-agnostic on purpose. This class talks about a "blob" and a "target height"; what the
 * blob is, how tall it is, and what the drivers call it live in {@code game/Pollen}, and
 * {@code Robot} pushes the height in every loop via {@link #setTargetHeightInches(double)}.
 *
 * <p>The botpose is reported relative to the FIELD CENTRE; Pedro's origin is a field CORNER, so
 * {@link #getBotposeAsPedroPose()} adds half the field to both axes. That half-field figure comes
 * from {@code FieldConstants} so there is exactly one place that knows how big the field is.
 */
@Configurable
public class Limelight {
    /** Added to the tag-derived yaw, for a camera whose forward axis is not the robot's. */
    public static double BOTPOSE_HEADING_OFFSET_RAD = 0.0;

    public static int APRILTAG_PIPELINE_INDEX = 0;
    public static int BLOB_PIPELINE_INDEX = 1;

    // Camera mount geometry - measure these on the real robot before trusting any pose estimate.
    public static double CAMERA_HEIGHT_INCHES = 12.0;
    public static double CAMERA_PITCH_DEGREES = 20.0;          // positive = tilted toward floor
    public static double CAMERA_FORWARD_OFFSET_INCHES = 6.0;   // camera ahead of robot center
    public static double CAMERA_LEFT_OFFSET_INCHES = 0.0;      // camera left of centerline
    public static double CAMERA_YAW_OFFSET_DEGREES = 0.0;      // positive turns the camera left

    /** Stop this far short of the piece, so the path ends with it at the intake, not under us. */
    public static double PICKUP_STANDOFF_INCHES = 8.0;
    /** Any estimate beyond this is rejected rather than driven to. */
    public static double MAX_VALID_DISTANCE_INCHES = 120.0;
    /** A result older than this means the camera has stopped delivering frames. */
    public static long MAX_STALENESS_MS = 250;

    /** Frames of agreement required before a detection is trusted enough to drive at. */
    public static int DETECTION_WINDOW = 5;
    /** Reject the window if the blob is jumping around by more than this many degrees. */
    public static double MAX_DETECTION_SPREAD_DEGREES = 6.0;

    private final Limelight3A limelight;
    /** Height of the blob's centre above the floor. Set by {@code Robot} from the game piece. */
    private double targetHeightInches = 0.0;
    private LLResult latestResult;
    private int currentPipeline;
    private List<LLResultTypes.ColorResult> blobDetections = Collections.emptyList();
    private LLResultTypes.ColorResult primaryBlob = null;

    private final MedianFilter txFilter = new MedianFilter(DETECTION_WINDOW);
    private final MedianFilter tyFilter = new MedianFilter(DETECTION_WINDOW);

    public Limelight(HardwareMap hardwareMap) {
        this(hardwareMap, HardwareNames.LIMELIGHT, APRILTAG_PIPELINE_INDEX);
    }

    public Limelight(HardwareMap hardwareMap, String name, int pipeline) {
        limelight = Hardware.get(hardwareMap, Limelight3A.class, name);
        currentPipeline = pipeline;
        if (limelight == null) return;
        // pipelineSwitch() and start() are synchronous HTTP calls to the camera. If it is unpowered
        // or unplugged these block on socket timeouts, and this constructor runs inside OpMode
        // init() - so an unplugged camera would stall the whole OpMode. Fail soft instead.
        try {
            limelight.pipelineSwitch(pipeline);
            limelight.start();
        } catch (RuntimeException e) {
            Hardware.recordFailure(name, "did not respond during init: " + e.getMessage());
        }
    }

    /** False when the camera is missing from the robot configuration. All reads then return empty. */
    public boolean isAvailable() {
        return limelight != null;
    }

    /**
     * Height of the blob target's centre above the floor, in inches. The ray-to-floor geometry
     * needs it; the season's value lives in {@code game/Pollen} and {@code Robot} pushes it here.
     */
    public void setTargetHeightInches(double inches) {
        targetHeightInches = inches;
    }

    public void update() {
        if (limelight == null) return;
        latestResult = limelight.getLatestResult();
        if (latestResult == null || !latestResult.isValid()) {
            blobDetections = Collections.emptyList();
            primaryBlob = null;
            txFilter.reset();
            tyFilter.reset();
            return;
        }

        List<LLResultTypes.ColorResult> colors = latestResult.getColorResults();
        if (colors == null || colors.isEmpty()) {
            blobDetections = Collections.emptyList();
            primaryBlob = null;
            txFilter.reset();
            tyFilter.reset();
            return;
        }

        // Pick the largest blob in a single pass. "Primary" is decided by this code rather than by
        // whatever sort order happens to be set in the Limelight web UI, which is invisible from
        // here — and a one-pass max costs no allocation, where copying and sorting the list every
        // loop did, for an ordering nothing else ever reads.
        blobDetections = colors;
        LLResultTypes.ColorResult largest = colors.get(0);
        for (int i = 1; i < colors.size(); i++) {
            if (colors.get(i).getTargetArea() > largest.getTargetArea()) largest = colors.get(i);
        }
        primaryBlob = largest;

        txFilter.add(largest.getTargetXDegrees());
        tyFilter.add(largest.getTargetYDegrees());
    }

    public LLResult getLatestResult() {
        return latestResult;
    }

    public LLStatus getStatus() {
        return limelight == null ? null : limelight.getStatus();
    }

    /**
     * True when the last frame carried a target of any kind.
     *
     * <p>Note this is pipeline-agnostic: it is true for a colour blob as readily as for an AprilTag.
     * It is not sufficient on its own to trust {@link #getBotposeAsPedroPose()} - see that method.
     */
    public boolean hasTarget() {
        return latestResult != null && latestResult.isValid() && !isStale();
    }

    /** True when the camera has stopped delivering fresh frames but is still returning the last one. */
    public boolean isStale() {
        return latestResult != null && latestResult.getStaleness() > MAX_STALENESS_MS;
    }

    /**
     * Raw horizontal offset of the current target in degrees, or {@link Double#NaN} with no
     * target. NaN rather than 0 for the same reason {@link #estimateBlobDistanceInches()} uses
     * it: 0 degrees is a real, on-axis reading and must not be confused with "nothing seen".
     */
    public double getTx() {
        return hasTarget() ? latestResult.getTx() : Double.NaN;
    }

    /** Raw vertical offset in degrees, or {@link Double#NaN} with no target. */
    public double getTy() {
        return hasTarget() ? latestResult.getTy() : Double.NaN;
    }

    /** Target area as a fraction of the frame, or {@link Double#NaN} with no target. */
    public double getTa() {
        return hasTarget() ? latestResult.getTa() : Double.NaN;
    }

    /**
     * Switches pipelines, returning whether the camera acknowledged it.
     *
     * <p>The cached index is only updated on success. Updating it unconditionally would make the
     * early-return below short-circuit every future call after a single failed switch, leaving the
     * camera permanently on the wrong pipeline with no retry.
     */
    private boolean switchPipeline(int pipeline) {
        if (limelight == null) return false;
        if (pipeline == currentPipeline) return true;
        boolean ok;
        try {
            ok = limelight.pipelineSwitch(pipeline);
        } catch (RuntimeException e) {
            return false;
        }
        if (ok) {
            currentPipeline = pipeline;
            // The camera needs several frames to produce results from the new pipeline. Drop the
            // caches so nothing reads the old pipeline's data as if it were the new pipeline's.
            latestResult = null;
            blobDetections = Collections.emptyList();
            primaryBlob = null;
        }
        return ok;
    }

    /** Which pipeline the camera is on, named rather than numbered, for telemetry. */
    public String getPipelineName() {
        if (currentPipeline == APRILTAG_PIPELINE_INDEX) return "apriltag";
        if (currentPipeline == BLOB_PIPELINE_INDEX) return "blob";
        return "pipeline " + currentPipeline;
    }

    public int getPipelineIndex() {
        return currentPipeline;
    }

    public void start() {
        if (limelight != null) limelight.start();
    }

    public void stop() {
        if (limelight != null) limelight.stop();
    }

    public Pose3D getBotpose() {
        return hasTarget() ? latestResult.getBotpose() : null;
    }

    /**
     * The AprilTag-derived field pose, converted to Pedro's corner-origin frame, or {@code null}.
     *
     * <p><b>Why the tag-count check is the load-bearing guard.</b> {@code getBotpose()} never returns
     * null and never returns NaN: the SDK's JSON reader hands back {@code new double[6]} when the key
     * is absent, and a Pose3D is always constructed from it. So on any valid non-AprilTag frame - a
     * colour blob, for instance - the raw botpose is all zeros, and the +72 origin shift turns that
     * into Pose(72, 72, 0): dead field centre, indistinguishable from a real reading.
     *
     * <p>Requiring at least one tag, and requiring that we are actually on the AprilTag pipeline,
     * is what separates "the camera localised us" from "the camera returned a default array".
     */
    public Pose getBotposeAsPedroPose() {
        if (!hasTarget()) return null;
        if (currentPipeline != APRILTAG_PIPELINE_INDEX) return null;
        if (latestResult.getBotposeTagCount() < 1) return null;

        Pose3D bp = latestResult.getBotpose();
        if (bp == null) return null;
        Position pos = bp.getPosition().toUnit(DistanceUnit.INCH);
        if (Double.isNaN(pos.x) || Double.isNaN(pos.y)) return null;

        double x = pos.x + FieldConstants.FIELD_CENTER_INCHES;
        double y = pos.y + FieldConstants.FIELD_CENTER_INCHES;
        if (!FieldConstants.isInsideField(x, y)) return null;

        YawPitchRollAngles ori = bp.getOrientation();
        double yaw = ori.getYaw(AngleUnit.RADIANS) + BOTPOSE_HEADING_OFFSET_RAD;
        return new Pose(x, y, yaw);
    }

    /** How many AprilTags contributed to the current botpose. 0 means the pose is meaningless. */
    public int getBotposeTagCount() {
        return hasTarget() ? latestResult.getBotposeTagCount() : 0;
    }

    /**
     * Age of the current result in milliseconds: how long ago the shutter opened.
     *
     * <p>A pose fix describes where the robot <em>was</em>, not where it is. At 40 in/s a 100 ms
     * pipeline is 4 inches of error, so anything fusing vision with odometry has to back-date it.
     */
    public long getVisionLatencyMs() {
        if (latestResult == null) return 0;
        return (long) (latestResult.getCaptureLatency() + latestResult.getTargetingLatency());
    }


    // ---- Colour-blob detection ----

    public boolean activateBlobPipeline() {
        return switchPipeline(BLOB_PIPELINE_INDEX);
    }

    public boolean activateAprilTagPipeline() {
        return switchPipeline(APRILTAG_PIPELINE_INDEX);
    }

    public boolean seesBlob() {
        return !blobDetections.isEmpty();
    }

    private double getBlobTx() {
        return primaryBlob == null ? 0 : primaryBlob.getTargetXDegrees();
    }

    private double getBlobTy() {
        return primaryBlob == null ? 0 : primaryBlob.getTargetYDegrees();
    }

    /** The camera's physical mounting, assembled from the tunable constants above. */
    private static VisionMath.Mount mount() {
        return new VisionMath.Mount(CAMERA_HEIGHT_INCHES, CAMERA_PITCH_DEGREES,
                CAMERA_FORWARD_OFFSET_INCHES, CAMERA_LEFT_OFFSET_INCHES, CAMERA_YAW_OFFSET_DEGREES);
    }

    /**
     * True when several consecutive frames agree on where the blob is.
     *
     * <p>A single frame is not enough to commit the robot to a path — one reflection or half-occluded
     * blob is all it takes to aim at a wall. This requires a full window <em>and</em> that the window
     * be tight; a blob jittering by more than {@link #MAX_DETECTION_SPREAD_DEGREES} is not a lock.
     */
    public boolean hasStableBlob() {
        return seesBlob()
                && txFilter.isReady()
                && tyFilter.isReady()
                && txFilter.spread() <= MAX_DETECTION_SPREAD_DEGREES
                && tyFilter.spread() <= MAX_DETECTION_SPREAD_DEGREES;
    }

    /** Median tx over the detection window — use this, not the raw frame, for aiming. */
    public double getFilteredBlobTx() {
        return txFilter.median();
    }

    public double getFilteredBlobTy() {
        return tyFilter.median();
    }

    public double getBlobSpreadDegrees() {
        return Math.max(txFilter.spread(), tyFilter.spread());
    }

    /**
     * Ground distance from the camera to the blob along the robot's forward axis.
     *
     * <p>Returns {@link Double#NaN} when there is no usable detection, rather than 0 — a zero here
     * would be indistinguishable from a legitimately computed zero and would be driven to.
     */
    public double estimateBlobDistanceInches() {
        if (blobDetections.isEmpty()) return Double.NaN;
        double ty = tyFilter.getCount() > 0 ? tyFilter.median() : getBlobTy();
        double d = VisionMath.forwardDistanceInches(
                ty, CAMERA_PITCH_DEGREES, CAMERA_HEIGHT_INCHES - targetHeightInches);
        if (Double.isNaN(d) || d > MAX_VALID_DISTANCE_INCHES) return Double.NaN;
        return d;
    }

    /**
     * Where the blob sits relative to the robot, as {@code {forward, left}} inches, or {@code null}.
     */
    public double[] estimateBlobInRobotFrame() {
        if (blobDetections.isEmpty()) return null;
        double tx = txFilter.getCount() > 0 ? txFilter.median() : getBlobTx();
        double ty = tyFilter.getCount() > 0 ? tyFilter.median() : getBlobTy();
        return VisionMath.targetInRobotFrame(
                tx, ty, mount(), targetHeightInches, MAX_VALID_DISTANCE_INCHES);
    }

    /**
     * Pose to drive to in order to pick up the blob, or {@code null} when there is no usable
     * detection. Includes {@link #PICKUP_STANDOFF_INCHES}, so this is an approach pose, not the
     * piece's own location.
     *
     * <p>Returning null rather than the robot's own pose matters: the caller builds a path to this
     * value, and echoing the current pose yields a zero-length path that silently does nothing.
     */
    public Pose estimateBlobApproachPose(Pose robotPose) {
        if (robotPose == null) return null;
        double[] robotFrame = estimateBlobInRobotFrame();
        if (robotFrame == null) return null;

        // Face the piece itself, but stop short of it.
        double heading = Angles.headingToward(
                robotPose.getHeading(), robotFrame[0], robotFrame[1]);

        double[] approach = VisionMath.applyStandoff(robotFrame, PICKUP_STANDOFF_INCHES);
        double[] field = VisionMath.toFieldFrame(robotPose.getX(), robotPose.getY(),
                robotPose.getHeading(), approach[0], approach[1]);

        if (!FieldConstants.isInsideField(field[0], field[1])) return null;
        return new Pose(field[0], field[1], heading);
    }
}
