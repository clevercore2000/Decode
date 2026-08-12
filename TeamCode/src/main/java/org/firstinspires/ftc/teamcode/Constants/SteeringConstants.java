package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class SteeringConstants {
    public static final double SERVO_TO_WHEEL_RATIO = 2.0;

    // Rev Through Bore encoder: 8192 ticks per encoder shaft revolution
    public static final int ENCODER_TICKS_PER_REV = 8192;
    // Ticks per wheel revolution (encoder on servo shaft, 2:1 to wheel)
    public static final int TICKS_PER_WHEEL_REV = (int) (ENCODER_TICKS_PER_REV * SERVO_TO_WHEEL_RATIO);

    // Encoder motor port names in the hardware map
    // These are the expansion hub motor ports where each Rev Through Bore encoder is plugged in.
    // FL, FR, BR share ports with intake/outtake motors; BL has a dedicated port.
    public static final String FL_ENCODER_NAME = "IM1";
    public static final String FR_ENCODER_NAME = "w2";
    public static final String BL_ENCODER_NAME = "bl_enc";
    public static final String BR_ENCODER_NAME = "w1";

    // Which encoder ports are shared with another motor (intake/outtake)
    public static final boolean FL_ENCODER_SHARED = true;
    public static final boolean FR_ENCODER_SHARED = true;
    public static final boolean BL_ENCODER_SHARED = false;
    public static final boolean BR_ENCODER_SHARED = true;

    // Per-module inversions. These are wiring facts, not tuning values — they must be
    // identical everywhere a module is constructed, otherwise a tick offset measured by
    // SwerveCalibration is interpreted under a different sign convention by SwerveDrive.
    // Values below are the ones SwerveDrive has been running with.
    // Drive inversions were flipped on all four modules so that +vx is physically forward,
    // +vy physically left and +omega physically CCW — the convention kinematics/ documents and
    // the one Pedro commands in. They previously encoded the opposite sense, which teleop
    // absorbed by feeding sticks in raw (FTC's left_stick_y is -1 pushed forward). The teleop
    // opmodes now negate all three axes, so driver-facing behaviour is unchanged.
    public static final boolean FL_DRIVE_INVERTED = true;
    public static final boolean FL_ENCODER_INVERTED = true;
    public static final boolean FL_STEER_INVERTED = false;

    public static final boolean FR_DRIVE_INVERTED = false;
    // Was true while OuttakeHardware reversed w2, which made the SDK negate this encoder for
    // free (setDirection(REVERSE) -> getCurrentPosition() negates). Two negations cancelled, so
    // teleop read +raw. That reversal is gone, so one negation goes with it and this must be
    // false to still read +raw. Net reading is unchanged; FR_TICK_OFFSET stays valid.
    public static final boolean FR_ENCODER_INVERTED = false;
    public static final boolean FR_STEER_INVERTED = true;

    public static final boolean BL_DRIVE_INVERTED = true;
    public static final boolean BL_ENCODER_INVERTED = false;
    public static final boolean BL_STEER_INVERTED = false;

    public static final boolean BR_DRIVE_INVERTED = false;
    public static final boolean BR_ENCODER_INVERTED = true;
    public static final boolean BR_STEER_INVERTED = true;

    // Per-module tick offsets: ticks from limit-switch home to "wheels forward"
    // Set these by running the SwerveCalibration opmode
    public static int FL_TICK_OFFSET = 2501;
    public static int FR_TICK_OFFSET = -306;
    public static int BL_TICK_OFFSET = 320;
    public static int BR_TICK_OFFSET = 2100;

    // Per-module azimuth trim, degrees. This is the fine correction for mechanical slop that a
    // whole-tick calibration cannot express — wheels that are a degree or two off straight make
    // the robot bow away from a straight path even when every module reports 0°.
    //
    // A module's trim is the angle its wheel *actually* sits at when the code believes it is at
    // 0°, positive counter-clockwise. Tune it live: run Translation Test straight forward, and
    // if the robot bows to the left, the modules are aiming left, so raise their trims.
    // Adjust in small steps (0.5°) and change one module at a time.
    //
    // Distinct from *_TICK_OFFSET on purpose: the offset is the coarse zero measured once by
    // SwerveCalibration, the trim is the live-tunable residual on top of it.
    public static double FL_TRIM_DEG = 0.0;
    public static double FR_TRIM_DEG = 0.0;
    public static double BL_TRIM_DEG = 0.0;
    public static double BR_TRIM_DEG = 0.0;

    // Limit switch hardware map names
    public static final String FL_SWITCH_NAME = "flSwitch";
    public static final String FR_SWITCH_NAME = "frSwitch";
    public static final String BL_SWITCH_NAME = "blSwitch";
    public static final String BR_SWITCH_NAME = "brSwitch";

    // Homing — dual-stage (fast approach, back off, slow approach)
    public static double HOMING_FAST_POWER = 0.6;
    public static double HOMING_SLOW_POWER = 0.2;
    public static double HOMING_BACKOFF_POWER = -0.3;
    public static int HOMING_BACKOFF_MS = 300;
    public static int HOMING_TIMEOUT_MS = 10000;
    // Limit switch polarity: false = pressed for active-low, true = pressed for active-high
    public static boolean LIMIT_SWITCH_ACTIVE_STATE = false;

    // Pre-run alignment: aim every module forward and wait for it to settle before a path
    // starts, so the follower is not re-aiming the wheels while it is already applying drive
    // power. Tolerance is checked against the post-optimize steering error.
    public static double ALIGN_TOLERANCE_RADIANS = Math.toRadians(2.0);
    public static int ALIGN_SETTLE_MS = 150;
    public static int ALIGN_TIMEOUT_MS = 4000;
    /**
     * Minimum time alignment always spends actively driving the modules to 0°, even once they
     * read in-tolerance. Off the ground a module reaches 0° almost immediately, so the settle
     * check alone lets alignment finish long before it would on carpet, where friction and
     * load make the same move slower. This floor makes the two cases behave the same.
     */
    public static int ALIGN_HOLD_MS = 1000;

    // Steering PD
    public static double STEER_P = 0.8;
    public static double STEER_D = 0.03;

    public static double STEERING_DEADBAND_RADIANS = 0.01;
    public static double MIN_SERVO_POWER = 0.05;
    public static double STATIC_FRICTION_COMPENSATION = 0.0;
}
