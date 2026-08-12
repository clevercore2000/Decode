package org.firstinspires.ftc.teamcode.Opmodes.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.kinematics.ChassisSpeeds;

/**
 * Open-loop chassis command bench: drive translation and rotation together, at a power ceiling
 * you choose, with no follower and no localization in the loop.
 * <p>
 * This is the opmode for answering "how much power does this manoeuvre actually need, and does
 * it still behave when translation and rotation are combined?". Everything it commands goes
 * straight through {@code setChassisSpeeds} to the modules, so what you set is what the wheels
 * get, capped at {@link #MAX_WHEEL_POWER}.
 * <p>
 * Two ways to drive it:
 * <ul>
 *   <li><b>Sticks</b> (default) — left stick translates, right stick X rotates. The natural way
 *       to feel combined motion and find where it starts to bind.</li>
 *   <li><b>Fixed</b> (△ toggles) — applies {@link #CMD_VX}/{@link #CMD_VY}/{@link #CMD_OMEGA}
 *       from the dashboard while <b>X</b> is held. Repeatable, unlike a thumb on a stick, so
 *       use this when comparing one power setting against another.</li>
 * </ul>
 * Note that a commanded omega is normalised against the drive base radius before it reaches the
 * kinematics (see {@link DriveConstants#ROTATION_GAIN}), so {@code CMD_OMEGA = 0.3} and
 * {@code CMD_VX = 0.3} now put the same magnitude of power into the wheels. Before that
 * normalisation existed, rotation arrived roughly 3.7x weaker than the number suggested, which
 * is why rotation appeared to need a command of 1.0 to move the robot at all.
 * <p>
 * Controls: <b>X</b> hold to drive · <b>△</b> toggle sticks/fixed · <b>O</b> zero the sticks
 * deadband trim · <b>▢</b> stop and hold forward.
 */
@Configurable
@TeleOp(name = "Drive Command Test", group = "Tuning")
public class DriveCommandTest extends LinearOpMode {

    /** Ceiling on the fastest module. The knob this opmode exists to explore. */
    public static double MAX_WHEEL_POWER = 0.4;

    /** Fixed-mode command, normalised: +1 forward, +1 left, +1 CCW. */
    public static double CMD_VX = 0.0;
    public static double CMD_VY = 0.0;
    public static double CMD_OMEGA = 0.3;

    /** Stick deadband, applied before any scaling. */
    public static double STICK_DEADBAND = 0.05;

    /** Scales stick input in stick mode. Separate from the power ceiling on purpose. */
    public static double STICK_SCALE = 1.0;

    private boolean fixedMode = false;

    @Override
    public void runOpMode() {
        SwerveDrive drive = new SwerveDrive(hardwareMap);

        telemetry.addLine("Drive Command Test — homing...");
        telemetry.update();

        boolean homed = drive.homeAllModulesDuringInit(this);
        boolean aligned = homed && drive.alignModulesForwardDuringInit(this);

        while (opModeInInit()) {
            drive.holdForward();
            telemetry.addData("Homed", homed ? "OK" : "FAILED — do not run");
            telemetry.addData("Aligned", aligned ? "OK" : "NOT SETTLED");
            telemetry.addData("Max wheel power", "%.2f", MAX_WHEEL_POWER);
            telemetry.addLine("X: hold to drive | △: sticks/fixed | ▢: stop");
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        boolean lastTriangle = false;

        while (opModeIsActive()) {
            boolean triangle = gamepad1.triangle;
            if (triangle && !lastTriangle) fixedMode = !fixedMode;
            lastTriangle = triangle;

            boolean driving = gamepad1.cross && !gamepad1.square;

            double vx, vy, omega;
            if (fixedMode) {
                vx = CMD_VX;
                vy = CMD_VY;
                omega = CMD_OMEGA;
            } else {
                // Same axis negations as the teleop opmodes: the SDK reports left_stick_y as -1
                // pushed forward and both x axes as +1 to the right, while the drive wants
                // +x forward, +y left, +omega CCW.
                vx = deadband(-gamepad1.left_stick_y) * STICK_SCALE;
                vy = deadband(-gamepad1.left_stick_x) * STICK_SCALE;
                omega = deadband(-gamepad1.right_stick_x) * STICK_SCALE;
            }

            if (driving) {
                drive.setChassisSpeeds(new ChassisSpeeds(vx, vy, omega), MAX_WHEEL_POWER);
                drive.update();
            } else {
                drive.holdForward();
            }

            telemetry.addData("Mode", fixedMode ? "FIXED (dashboard)" : "STICKS");
            telemetry.addData("Driving", driving ? "YES — holding X" : "no (hold X)");
            telemetry.addData("Max wheel power", "%.2f", MAX_WHEEL_POWER);
            telemetry.addData("Rotation gain", "%.2f  (base radius %.3f m)",
                    DriveConstants.ROTATION_GAIN, DriveConstants.driveBaseRadiusMeters());
            telemetry.addLine();
            telemetry.addData("Command", "vx %.2f   vy %.2f   omega %.2f", vx, vy, omega);
            telemetry.addLine();
            drive.logDetailed(telemetry);
            telemetry.addLine();
            telemetry.addLine("X: hold to drive | △: sticks/fixed | ▢: stop");
            telemetry.update();
        }
    }

    private double deadband(double value) {
        return Math.abs(value) < STICK_DEADBAND ? 0.0 : value;
    }
}