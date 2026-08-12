package org.firstinspires.ftc.teamcode.Opmodes.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.Utils.MathUtils;
import org.firstinspires.ftc.teamcode.pedro.PedroConstants;
import org.firstinspires.ftc.teamcode.pedro.PedroSwerveDrivetrain;

/**
 * Rotates in place by a set number of degrees, and reports how many degrees it actually turned.
 * <p>
 * Uses {@code Follower.turnDegrees(degrees, isLeft)}, which is a <i>relative</i> turn: Pedro
 * reads the current pose, adds the requested rotation, and holds that pose — same X and Y, new
 * heading. So there is no path involved and nothing degenerate to chase, unlike asking a path
 * follower to travel between two identical points.
 * <p>
 * Each press turns by {@link #TURN_DEGREES} from wherever the robot currently is, so presses
 * accumulate: four 90° presses should bring you back to the start, and the drift you see after
 * those four is a far better measure of turn accuracy than any single one.
 * <p>
 * Keep individual turns to 180° or less. The "Turned" readout wraps to ±180°, so a larger
 * single turn would read as its short way round.
 * <p>
 * Controls: <b>X</b> turn left (CCW) · <b>O</b> turn right (CW) · <b>▢</b> stop and hold
 * forward · <b>△</b> re-zero the pose to 0°, for measuring from a known start.
 */
@Configurable
@TeleOp(name = "Rotate Degrees Test", group = "Tuning")
public class RotateDegreesTest extends LinearOpMode {

    /** Degrees to rotate per press. Keep at or below 180 — see the class notes. */
    public static double TURN_DEGREES = 90.0;

    /** Follower power ceiling for the turn. */
    public static double TURN_MAX_POWER = 0.5;

    private static final Pose ORIGIN = new Pose(0, 0, 0);
    private static final String CONTROLS =
            "X: turn left | O: turn right | ▢: stop | △: re-zero pose";

    private SwerveDrive drive;
    private Follower follower;
    private PedroSwerveDrivetrain drivetrain;

    private boolean running = false;
    private double startHeading = 0.0;
    private double targetHeading = 0.0;
    private double commandedDegrees = 0.0;

    @Override
    public void runOpMode() {
        drive = new SwerveDrive(hardwareMap);
        drivetrain = new PedroSwerveDrivetrain(drive, hardwareMap);
        follower = PedroConstants.createFollower(hardwareMap, drivetrain);

        telemetry.addLine("Rotate Degrees Test — homing...");
        telemetry.update();

        boolean homed = drive.homeAllModulesDuringInit(this);
        boolean aligned = homed && drive.alignModulesForwardDuringInit(this);

        follower.setStartingPose(ORIGIN);

        while (opModeInInit()) {
            drive.holdForward();
            telemetry.addData("Homed", homed ? "OK" : "FAILED — do not run");
            telemetry.addData("Aligned", aligned ? "OK — wheels at 0°" : "NOT SETTLED");
            telemetry.addData("Turn amount", "%.1f°", TURN_DEGREES);
            telemetry.addLine(CONTROLS);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        boolean lastCross = false, lastCircle = false, lastSquare = false, lastTriangle = false;

        while (opModeIsActive()) {
            boolean cross = gamepad1.cross, circle = gamepad1.circle;
            boolean square = gamepad1.square, triangle = gamepad1.triangle;

            if (cross && !lastCross) startTurn(true);
            if (circle && !lastCircle) startTurn(false);
            if (square && !lastSquare) {
                follower.breakFollowing();
                running = false;
            }
            if (triangle && !lastTriangle) {
                follower.breakFollowing();
                running = false;
                follower.setPose(ORIGIN);
            }

            lastCross = cross;
            lastCircle = circle;
            lastSquare = square;
            lastTriangle = triangle;

            follower.update();

            if (!running) {
                // Not turning: park the wheels forward rather than leaving them wherever the
                // last command left them. Runs after follower.update() so this write wins.
                drive.holdForward();
            }

            double heading = follower.getPose().getHeading();
            double turned = Math.toDegrees(MathUtils.wrapRad(heading - startHeading));
            double error = Math.toDegrees(MathUtils.wrapRad(targetHeading - heading));

            telemetry.addData("Commanded", "%.1f°", commandedDegrees);
            telemetry.addData("Turned", "%.1f°", turned);
            telemetry.addData("Remaining", "%.1f°", error);
            telemetry.addLine();
            telemetry.addData("Heading", "%.1f°  (target %.1f°)",
                    Math.toDegrees(heading), Math.toDegrees(targetHeading));
            telemetry.addData("Turning", follower.isTurning() ? "YES" : "settled");
            telemetry.addData("Tolerance", "%.1f°", PedroConstants.TURN_TOLERANCE_DEG);
            telemetry.addLine();
            telemetry.addLine(drivetrain.debugString());
            telemetry.addLine();
            telemetry.addLine(CONTROLS);
            telemetry.update();
        }
    }

    /**
     * Aligns the modules, then asks the follower to rotate by {@link #TURN_DEGREES}.
     *
     * @param left true to turn counter-clockwise, false for clockwise.
     */
    private void startTurn(boolean left) {
        follower.setMaxPower(TURN_MAX_POWER);

        // Settle the wheels before any power goes down, under whatever load the robot is
        // actually sitting on — a turn that begins by dragging misaligned wheels into the
        // rotation pattern loses accuracy it never gets back.
        telemetry.addLine("Aligning before turn...");
        telemetry.update();
        drive.alignModulesForward(this);

        startHeading = follower.getPose().getHeading();
        commandedDegrees = left ? TURN_DEGREES : -TURN_DEGREES;
        targetHeading = MathUtils.wrapRad(startHeading + Math.toRadians(commandedDegrees));

        follower.turnDegrees(TURN_DEGREES, left);
        running = true;
    }
}
