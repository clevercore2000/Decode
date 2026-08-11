package org.firstinspires.ftc.teamcode.Constants;

import com.pedropathing.geometry.Pose;

/**
 * Field poses for autonomous, in Pedro's frame: inches, 0–144 on both axes with the origin at
 * the bottom-left corner, heading in radians with 0 along +x and CCW positive.
 * <p>
 * These are placeholders shaped like a plausible routine, not measured field positions — set
 * them against the real field before running. Poses live here rather than inline in the opmode
 * so a path can be re-aimed without touching the state machine.
 */
public class AutoConstants {

    public static final double FACING_FORWARD = 0.0;
    public static final double FACING_LEFT = Math.PI / 2.0;
    public static final double FACING_BACK = Math.PI;
    public static final double FACING_RIGHT = -Math.PI / 2.0;

    /** Where the robot is placed at init. Must match the physical starting position. */
    public static final Pose START = new Pose(9.0, 60.0, FACING_FORWARD);

    /** Scoring position for the outtake. */
    public static final Pose SCORE = new Pose(36.0, 84.0, FACING_LEFT);

    /** Where the intake collects. */
    public static final Pose COLLECT = new Pose(36.0, 24.0, FACING_RIGHT);

    /** Final resting position at the end of the routine. */
    public static final Pose PARK = new Pose(60.0, 60.0, FACING_FORWARD);

    /** Outtake spin-up time before the ramp is fired, milliseconds. */
    public static double SPINUP_MS = 1200;

    /** How long the ramp stays open to shoot, milliseconds. */
    public static double SHOOT_MS = 800;

    /** How long to run the intake at the collect position, milliseconds. */
    public static double COLLECT_MS = 1500;
}
