package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.Drivetrain;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveModule;
import org.firstinspires.ftc.teamcode.kinematics.ChassisSpeeds;

import java.util.Locale;

/**
 * Adapts the robot's {@link SwerveDrive} to Pedro Pathing's {@link Drivetrain} contract.
 * <p>
 * Pedro 2.0.4 ships only a mecanum drivetrain, so this is wired in via
 * {@code FollowerBuilder.setDrivetrain(...)}. It deliberately owns no hardware of its own
 * beyond the voltage sensor — steering, homing and the per-module PD loop all stay in
 * {@link SwerveDrive} and {@link SwerveModule}, which are already working and calibrated.
 * <p>
 * <b>Units.</b> Pedro hands the drivetrain unitless <i>power</i> vectors (magnitude clamped to
 * {@code maxPowerScaling} ≤ 1), not velocities, so Pedro's inches-based field geometry never
 * reaches this class. {@link SwerveModule} multiplies commanded speed straight into motor
 * power via {@code DRIVE_FF}, so powers pass through unconverted. Only
 * {@link #xVelocity()}/{@link #yVelocity()} carry real units, and Pedro wants those in
 * inches/second.
 * <p>
 * <b>Frames.</b> {@code correctivePower} and {@code pathingPower} arrive in the <i>field</i>
 * frame; {@code SwerveDrive} wants the robot frame, hence the rotation by {@code -robotHeading}.
 * Both frames agree on +x forward, +y left and CCW-positive rotation.
 */
public class PedroSwerveDrivetrain extends Drivetrain {

    private static final double METERS_TO_INCHES = 39.3701;

    private final SwerveDrive drive;
    private final VoltageSensor voltageSensor;

    private double xVelocity;
    private double yVelocity;

    /** Last command applied, kept only so {@link #debugString()} can report it. */
    private double lastVx, lastVy, lastOmega;

    public PedroSwerveDrivetrain(SwerveDrive drive, HardwareMap hardwareMap) {
        this.drive = drive;
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        updateConstants();
    }

    /**
     * Maps Pedro's three power vectors onto a swerve chassis command.
     * <p>
     * A mecanum has to express rotation through the same four wheels it translates with, so
     * Pedro's {@code Mecanum} splits the command into left/right side vectors and folds the
     * heading term in as a differential. A swerve decouples the two outright, so translation
     * and rotation are computed independently here and the left/right split is not needed.
     *
     * @return {@code {vx, vy, omega}} in the robot frame — <b>not</b> the four wheel powers the
     *         base class javadoc describes. Both ends of that array are ours (this method
     *         produces it, {@link #runDrive(double[])} consumes it), so the meaning is a private
     *         contract between the two.
     */
    @Override
    public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        // Work in components throughout: Vector's magnitude/theta form can represent a negative
        // magnitude, and component math is insensitive to how it chooses to normalise that.
        double cx = correctivePower.getXComponent();
        double cy = correctivePower.getYComponent();
        double correctiveMag = Math.hypot(cx, cy);

        double fx, fy;
        if (correctiveMag >= maxPowerScaling) {
            // Corrective power has saturated. Mecanum drops pathing power entirely here so the
            // robot commits to returning to the curve rather than continuing along it; match that.
            fx = cx;
            fy = cy;
        } else {
            fx = cx + pathingPower.getXComponent();
            fy = cy + pathingPower.getYComponent();
        }

        double mag = Math.hypot(fx, fy);
        if (mag > maxPowerScaling && mag > 0) {
            double scale = maxPowerScaling / mag;
            fx *= scale;
            fy *= scale;
        }

        // Field -> robot frame: rotate by -robotHeading.
        double cos = Math.cos(robotHeading);
        double sin = Math.sin(robotHeading);
        double vx = fx * cos + fy * sin;
        double vy = -fx * sin + fy * cos;

        // headingPower points along the robot's current heading with a signed magnitude, so its
        // projection onto the heading unit vector recovers the signed turn command. Pedro's
        // Mecanum applies it as left = corrective - heading, right = corrective + heading; a
        // faster right side is a CCW rotation, so a positive projection means positive omega.
        double omega = PedroConstants.HEADING_POWER_SIGN
                * (headingPower.getXComponent() * cos + headingPower.getYComponent() * sin);

        lastVx = vx;
        lastVy = vy;
        lastOmega = omega;

        return new double[]{vx, vy, omega};
    }

    /** @param drivePowers {@code {vx, vy, omega}} as produced by {@link #calculateDrive}. */
    @Override
    public void runDrive(double[] drivePowers) {
        // setChassisSpeeds rather than drive(): drive() applies a 0.01 stick deadband that would
        // swallow exactly the small corrections keeping the robot on the path.
        drive.setChassisSpeeds(new ChassisSpeeds(drivePowers[0], drivePowers[1], drivePowers[2]));
        drive.update();
    }

    @Override
    public void updateConstants() {
        this.maxPowerScaling = PedroConstants.MAX_POWER;
        this.nominalVoltage = PedroConstants.NOMINAL_VOLTAGE;
        this.voltageCompensation = PedroConstants.VOLTAGE_COMPENSATION;
        double inchesPerSecond = DriveConstants.MAX_VELOCITY_MPS * METERS_TO_INCHES;
        this.xVelocity = inchesPerSecond;
        this.yVelocity = inchesPerSecond;
    }

    @Override
    public void breakFollowing() {
        lastVx = lastVy = lastOmega = 0;
        // hold() parks each module at its current azimuth with zero drive power, which leaves
        // the wheels where they are instead of snapping them back to forward.
        drive.hold();
    }

    @Override
    public void startTeleopDrive() {
        startTeleopDrive(true);
    }

    @Override
    public void startTeleopDrive(boolean brakeMode) {
        // SwerveModule already sets ZeroPowerBehavior.BRAKE on every drive motor at construction
        // and never changes it, so there is no mode to switch into here.
    }

    @Override
    public double xVelocity() {
        return xVelocity;
    }

    @Override
    public double yVelocity() {
        return yVelocity;
    }

    @Override
    public void setXVelocity(double xMovement) {
        this.xVelocity = xMovement;
    }

    @Override
    public void setYVelocity(double yMovement) {
        this.yVelocity = yMovement;
    }

    @Override
    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    @Override
    public String debugString() {
        return String.format(Locale.US,
                "PedroSwerveDrivetrain{vx=%.3f, vy=%.3f, omega=%.3f, maxPower=%.2f, xVel=%.1f in/s, %.2fV}",
                lastVx, lastVy, lastOmega, maxPowerScaling, xVelocity, getVoltage());
    }
}
