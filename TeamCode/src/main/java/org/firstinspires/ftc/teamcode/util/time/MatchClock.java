package org.firstinspires.ftc.teamcode.util.time;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.util.field.PoseFusion;
import org.firstinspires.ftc.teamcode.util.math.VisionMath;

/**
 * Tracks how much of the match period is left.
 *
 * <h2>Why this exists</h2>
 * Without a clock the robot cannot make any decision that depends on time — it cannot warn the
 * drivers that endgame has started, cannot decide that there is no longer room for a full scoring
 * cycle, and cannot record in the log which phase a problem happened in. Every one of those needs
 * the same three numbers, so they live here once.
 *
 * <h2>Time is passed in, not read</h2>
 * This class never calls {@code System.currentTimeMillis()} or touches an {@code ElapsedTime}. The
 * caller supplies the timestamp to {@link #start(long)} and {@link #update(long)}. That is what
 * makes it unit-testable off-robot — the same reason {@link VisionMath} and {@link PoseFusion} are
 * separate from the subsystems that use them. Tests can advance time by an hour instantly.
 *
 * <pre>
 *   // in an OpMode
 *   clock = MatchClock.forTeleop();
 *   clock.start(System.currentTimeMillis());   // in start()
 *   clock.update(System.currentTimeMillis());  // top of every loop
 * </pre>
 *
 * <p>Durations are {@code @Configurable} because off-season scrimmages and practice sessions
 * routinely run non-standard periods, and a wrong endgame warning is worse than none.
 */
@Configurable
public class MatchClock {
    /** Standard FTC autonomous period. */
    public static long AUTONOMOUS_MS = 30_000;
    /** Standard FTC driver-controlled period, inclusive of endgame. */
    public static long TELEOP_MS = 120_000;
    /** Trailing slice of teleop treated as endgame. */
    public static long ENDGAME_MS = 30_000;

    /** Which match period this clock is counting. */
    public enum Period { AUTONOMOUS, TELEOP }

    /**
     * Where we are in the period.
     *
     * <p>{@link #ENDGAME} only ever occurs on a {@link Period#TELEOP} clock; an autonomous clock
     * goes straight from {@link #RUNNING} to {@link #EXPIRED}.
     */
    public enum Phase { NOT_STARTED, RUNNING, ENDGAME, EXPIRED }

    private final Period period;
    private final long durationMs;
    private final long endgameMs;

    private long startMs = 0;
    private long nowMs = 0;
    private boolean started = false;

    private MatchClock(Period period, long durationMs, long endgameMs) {
        this.period = period;
        this.durationMs = Math.max(0, durationMs);
        this.endgameMs = Math.max(0, Math.min(endgameMs, this.durationMs));
    }

    /** A clock for the autonomous period. Has no endgame. */
    public static MatchClock forAutonomous() {
        return new MatchClock(Period.AUTONOMOUS, AUTONOMOUS_MS, 0);
    }

    /** A clock for the driver-controlled period, with the final {@link #ENDGAME_MS} as endgame. */
    public static MatchClock forTeleop() {
        return new MatchClock(Period.TELEOP, TELEOP_MS, ENDGAME_MS);
    }

    /** A clock with explicit durations, for practice periods that are not match length. */
    public static MatchClock of(Period period, long durationMs, long endgameMs) {
        return new MatchClock(period, durationMs, endgameMs);
    }

    /** Marks the start of the period. Call once, from the OpMode's {@code start()}. */
    public void start(long nowMs) {
        this.startMs = nowMs;
        this.nowMs = nowMs;
        this.started = true;
    }

    /** Advances the clock. Call once per loop, before anything that reads it. */
    public void update(long nowMs) {
        this.nowMs = nowMs;
    }

    public boolean isStarted() {
        return started;
    }

    public Period getPeriod() {
        return period;
    }

    /** Milliseconds since {@link #start}, or 0 before it. Never negative. */
    public long getElapsedMs() {
        if (!started) return 0;
        return Math.max(0, nowMs - startMs);
    }

    /** Milliseconds left in the period. Clamped at 0 — never counts past the buzzer. */
    public long getRemainingMs() {
        if (!started) return durationMs;
        return Math.max(0, durationMs - getElapsedMs());
    }

    public double getRemainingSeconds() {
        return getRemainingMs() / 1000.0;
    }

    public Phase getPhase() {
        if (!started) return Phase.NOT_STARTED;
        long remaining = getRemainingMs();
        if (remaining <= 0) return Phase.EXPIRED;
        if (endgameMs > 0 && remaining <= endgameMs) return Phase.ENDGAME;
        return Phase.RUNNING;
    }

    public boolean isEndgame() {
        return getPhase() == Phase.ENDGAME;
    }

    public boolean isExpired() {
        return getPhase() == Phase.EXPIRED;
    }

    /**
     * Whether {@code budgetMs} of work still fits before the buzzer.
     *
     * <p>This is the hook for time-aware fallbacks: ask before committing to a long cycle, and take
     * the short one when the answer is no. Returns {@code true} before the clock has started, so a
     * routine that is never given a clock behaves exactly as it did before this class existed.
     */
    public boolean hasTimeFor(long budgetMs) {
        if (!started) return true;
        return getRemainingMs() >= budgetMs;
    }

    /** Compact status for telemetry, e.g. {@code "ENDGAME 24.6s"}. */
    public String getStatus() {
        if (!started) return "not started";
        return getPhase() + String.format(java.util.Locale.US, " %.1fs", getRemainingSeconds());
    }
}
