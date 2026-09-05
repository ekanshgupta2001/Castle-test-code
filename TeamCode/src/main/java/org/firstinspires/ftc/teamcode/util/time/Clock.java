package org.firstinspires.ftc.teamcode.util.time;

/**
 * The one source of "what time is it" for loop logic.
 *
 * <p>Every timeout, dwell and rate limit on the robot used to read {@code System.currentTimeMillis()}
 * on its own. That had two costs. Nothing that read the clock could be unit tested without real
 * sleeps, and wall-clock time is not monotonic: a network time sync on the Control Hub can move it
 * backwards, which turns a 200 ms stall dwell into a never-fires or an instant trip.
 *
 * <p>Production code uses {@link #system()}, which is monotonic. Tests inject a fake and advance it
 * by hand. Values are milliseconds from an arbitrary origin, so they are only meaningful relative
 * to each other; never compare one with a wall-clock timestamp.
 */
public interface Clock {
    /** Milliseconds since an arbitrary fixed origin. Never decreases. */
    long nowMs();

    /** The monotonic system clock. */
    static Clock system() {
        return SystemClock.INSTANCE;
    }
}
