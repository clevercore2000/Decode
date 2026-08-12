package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class DriveConstants {
    public static double WHEELBASE_METERS = 0.385;
    public static double TRACK_WIDTH_METERS = 0.385;
    public static double WHEEL_DIAMETER_METERS = 0.0762;
    public static final double WHEEL_CIRCUMFERENCE_METERS = Math.PI * WHEEL_DIAMETER_METERS;

    // goBILDA 5202/5203 435 RPM: 384.5 ticks per revolution measured at the OUTPUT shaft,
    // i.e. the 13.7:1 gearbox is already baked in. DRIVE_GEAR_RATIO is kept for reference
    // only — multiplying by it here would count the reduction twice.
    public static double DRIVE_GEAR_RATIO = 13.7;
    public static final double MOTOR_TICKS_PER_REV = 384.5;
    public static final double TICKS_PER_METER = MOTOR_TICKS_PER_REV / WHEEL_CIRCUMFERENCE_METERS;

    public static double DRIVE_FF = 1.0;

    /**
     * Authority of the rotation term relative to translation, once omega has been normalised by
     * the drive base radius. 1.0 means "omega = 1 drives the wheels as hard as vx = 1".
     * <p>
     * The normalisation matters because {@code SwerveDriveKinematics} multiplies omega by each
     * module's distance from centre in <b>metres</b> (~0.272 m here), while vx and vy pass
     * straight through as unitless power. Without correcting for that, a commanded omega
     * reaches the wheels at ~27% of the strength of the same commanded vx — which reads as
     * "the robot has no torque to turn" when it is really a units mismatch. Teleop masks it by
     * scaling sticks by 5.0 and saturating; a path follower, which emits powers below 1, does
     * not.
     */
    public static double ROTATION_GAIN = 1.0;

    /** Distance from robot centre to a module, metres — the radius omega acts through. */
    public static double driveBaseRadiusMeters() {
        return Math.hypot(WHEELBASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0);
    }

    /**
     * Free translational speed of the chassis, metres/second. Used to convert the follower's
     * unitless power output into a velocity estimate for Pedro. Measure this with a straight-line
     * full-power run rather than trusting the nameplate figure.
     */
    public static double MAX_VELOCITY_MPS = 1.6;
}
