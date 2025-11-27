package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Constants.SteeringConstants;
import org.firstinspires.ftc.teamcode.Hardware.SwerveHardware;

/**
 * Swerve Drive Coordinator
 *
 * Coordinates 4 swerve modules using custom inverse kinematics.
 * Based on PowerPlay's proven approach - simpler than FTCLib.
 *
 * Kinematics Formula (Robot-Centric):
 * For chassis velocity (vx, vy, omega) and module at position (x, y):
 *   moduleSpeed = √(vx_module² + vy_module²)
 *   moduleAngle = atan2(vy_module, vx_module)
 *
 * Where:
 *   a = vx - omega × (WHEELBASE / R)
 *   b = vx + omega × (WHEELBASE / R)
 *   c = vy - omega × (TRACK_WIDTH / R)
 *   d = vy + omega × (TRACK_WIDTH / R)
 *   R = distance from robot center to module
 *
 * Module mapping:
 *   FL: (b, c)  →  speed=√(b²+c²), angle=atan2(c, b)
 *   FR: (b, d)  →  speed=√(b²+d²), angle=atan2(d, b)
 *   BL: (a, c)  →  speed=√(a²+c²), angle=atan2(c, a)
 *   BR: (a, d)  →  speed=√(a²+d²), angle=atan2(d, a)
 */
public class SwerveDrive {

    // Swerve modules (FL, FR, BL, BR standard order)
    private final SwerveModule flModule;
    private final SwerveModule frModule;
    private final SwerveModule blModule;
    private final SwerveModule brModule;

    // Kinematics constant: distance from robot center to module
    private final double R;

    /**
     * Create swerve drive system
     *
     * @param hardware Hardware abstraction containing motors, servos, encoders
     */
    public SwerveDrive(SwerveHardware hardware) {
        // Calculate R = distance from center to module
        // For square robot: R = √(wheelbase² + trackWidth²) / 2
        R = Math.sqrt(
            Math.pow(DriveConstants.WHEELBASE_METERS, 2) +
            Math.pow(DriveConstants.TRACK_WIDTH_METERS, 2)
        ) / 2;

        // Initialize all 4 modules (order: FL, FR, BL, BR - standard!)
        flModule = new SwerveModule(
            hardware.flDrive,
            hardware.flSteer,
            hardware.flEncoder,
            SteeringConstants.FL_ANGLE_OFFSET,
            false,  // drive not inverted
            false,  // steer not inverted
            "FL"
        );

        frModule = new SwerveModule(
            hardware.frDrive,
            hardware.frSteer,
            hardware.frEncoder,
            SteeringConstants.FR_ANGLE_OFFSET,
            true,   // drive inverted (right side)
            false,  // steer not inverted
            "FR"
        );

        blModule = new SwerveModule(
            hardware.blDrive,
            hardware.blSteer,
            hardware.blEncoder,
            SteeringConstants.BL_ANGLE_OFFSET,
            false,  // drive not inverted
            false,  // steer not inverted
            "BL"
        );

        brModule = new SwerveModule(
            hardware.brDrive,
            hardware.brSteer,
            hardware.brEncoder,
            SteeringConstants.BR_ANGLE_OFFSET,
            true,   // drive inverted (right side)
            false,  // steer not inverted
            "BR"
        );
    }

    /**
     * Drive the robot (robot-centric)
     *
     * Coordinate system (WPILib standard):
     *   +vx = forward (toward front of robot)
     *   +vy = left (toward left side of robot) ← NOT right!
     *   +omega = counter-clockwise rotation
     *
     * @param vx     Forward velocity in m/s (positive = forward)
     * @param vy     Strafe velocity in m/s (positive = left)
     * @param omega  Rotation velocity in rad/s (positive = counter-clockwise)
     */
    public void drive(double vx, double vy, double omega) {
        // Custom inverse kinematics (PowerPlay approach)
        double a = vx - omega * (DriveConstants.WHEELBASE_METERS / R);
        double b = vx + omega * (DriveConstants.WHEELBASE_METERS / R);
        double c = vy - omega * (DriveConstants.TRACK_WIDTH_METERS / R);
        double d = vy + omega * (DriveConstants.TRACK_WIDTH_METERS / R);

        // Calculate module speeds and angles
        double[] speeds = new double[4];
        double[] angles = new double[4];

        speeds[0] = Math.hypot(b, c);  // FL
        angles[0] = Math.atan2(c, b);

        speeds[1] = Math.hypot(b, d);  // FR
        angles[1] = Math.atan2(d, b);

        speeds[2] = Math.hypot(a, c);  // BL
        angles[2] = Math.atan2(c, a);

        speeds[3] = Math.hypot(a, d);  // BR
        angles[3] = Math.atan2(d, a);

        // Normalize speeds if any exceed 1.0 (preserves motion direction)
        double maxSpeed = Math.max(
            Math.max(speeds[0], speeds[1]),
            Math.max(speeds[2], speeds[3])
        );

        if (maxSpeed > 1.0) {
            for (int i = 0; i < 4; i++) {
                speeds[i] /= maxSpeed;
            }
        }

        // Apply states to modules
        flModule.setDesiredState(speeds[0], angles[0]);
        frModule.setDesiredState(speeds[1], angles[1]);
        blModule.setDesiredState(speeds[2], angles[2]);
        brModule.setDesiredState(speeds[3], angles[3]);
    }

    /**
     * Stop all modules
     *
     * CRITICAL: Uses SwerveModule.stop() which maintains encoder power
     */
    public void stop() {
        flModule.stop();
        frModule.stop();
        blModule.stop();
        brModule.stop();
    }

    /**
     * Get FL module (for telemetry/debugging)
     */
    public SwerveModule getFlModule() {
        return flModule;
    }

    /**
     * Get FR module (for telemetry/debugging)
     */
    public SwerveModule getFrModule() {
        return frModule;
    }

    /**
     * Get BL module (for telemetry/debugging)
     */
    public SwerveModule getBlModule() {
        return blModule;
    }

    /**
     * Get BR module (for telemetry/debugging)
     */
    public SwerveModule getBrModule() {
        return brModule;
    }
}
