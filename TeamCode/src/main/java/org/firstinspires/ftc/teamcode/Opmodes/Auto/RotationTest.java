package org.firstinspires.ftc.teamcode.Opmodes.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.kinematics.ChassisSpeeds;

/**
 * Diagnoses why the robot strains but does not turn.
 * <p>
 * Rotation is the first manoeuvre that sends the four modules to four <i>different</i>
 * azimuths. Forward and strafe put every module on a common azimuth with a common speed sign,
 * so they only prove the global sign is right — they cannot catch a module whose position or
 * drive sign is wrong relative to the others. When rotation strains without moving, the wheels
 * are pushing against each other, and there are two very different causes:
 * <ol>
 *   <li>the modules never reach the rotation pattern (steering lacks authority under load), or</li>
 *   <li>they reach it correctly but the drive powers oppose each other.</li>
 * </ol>
 * With drive power applied these look identical, so this opmode separates them.
 * <p>
 * <b>Use ▢ first — it is the whole point of this opmode.</b> It aims the modules into the
 * rotation pattern with <i>zero</i> drive power, so nothing can move and you can simply look at
 * the wheels. For a square chassis the correct pattern is an X, every wheel tangent to a circle
 * about the robot centre:
 * <pre>
 *   FL 135°   FR  45°        \   /
 *   BL -135°  BR -45°        /   \
 * </pre>
 * Check the reported angles converge on those numbers <i>and</i> that the physical wheel you
 * call front-left is the one reading FL. If the pattern never forms, it is cause 1 — steering
 * authority. If it forms cleanly but O still will not turn the robot, it is cause 2, and the
 * per-module drive powers on screen will show which module opposes the others.
 * <p>
 * Controls: <b>▢</b> aim only (no drive power) · <b>X</b> rotate CCW · <b>O</b> rotate CW ·
 * <b>△</b> stop and hold forward.
 */
@Configurable
@TeleOp(name = "Rotation Test", group = "Tuning")
public class RotationTest extends LinearOpMode {

    /** Rotation command magnitude. Keep it small — this is a diagnostic, not a speed run. */
    public static double ROTATION_POWER = 0.3;

    private enum Mode { IDLE, AIM_ONLY, ROTATE_CCW, ROTATE_CW }

    private static final String CONTROLS =
            "▢: aim only | X: rotate CCW | O: rotate CW | △: stop";

    @Override
    public void runOpMode() {
        SwerveDrive drive = new SwerveDrive(hardwareMap);

        telemetry.addLine("Rotation Test — homing...");
        telemetry.update();

        boolean homed = drive.homeAllModulesDuringInit(this);
        boolean aligned = homed && drive.alignModulesForwardDuringInit(this);

        while (opModeInInit()) {
            drive.holdForward();
            telemetry.addData("Homed", homed ? "OK" : "FAILED — do not run");
            telemetry.addData("Aligned", aligned ? "OK" : "NOT SETTLED");
            telemetry.addLine("Start, then press ▢ and LOOK AT THE WHEELS.");
            telemetry.addLine(CONTROLS);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        Mode mode = Mode.IDLE;
        boolean lastSquare = false, lastCross = false, lastCircle = false, lastTriangle = false;

        while (opModeIsActive()) {
            boolean square = gamepad1.square, cross = gamepad1.cross;
            boolean circle = gamepad1.circle, triangle = gamepad1.triangle;

            if (square && !lastSquare) mode = Mode.AIM_ONLY;
            if (cross && !lastCross) mode = Mode.ROTATE_CCW;
            if (circle && !lastCircle) mode = Mode.ROTATE_CW;
            if (triangle && !lastTriangle) mode = Mode.IDLE;

            lastSquare = square;
            lastCross = cross;
            lastCircle = circle;
            lastTriangle = triangle;

            switch (mode) {
                case AIM_ONLY:
                    // Azimuths for a pure rotation, but zero wheel speed: the modules take up
                    // the pattern and nothing can drive, so what you see is the geometry alone.
                    drive.setAzimuthsOnly(new ChassisSpeeds(0, 0, 1));
                    drive.update();
                    break;
                case ROTATE_CCW:
                    drive.setChassisSpeeds(new ChassisSpeeds(0, 0, ROTATION_POWER));
                    drive.update();
                    break;
                case ROTATE_CW:
                    drive.setChassisSpeeds(new ChassisSpeeds(0, 0, -ROTATION_POWER));
                    drive.update();
                    break;
                case IDLE:
                default:
                    drive.holdForward();
                    break;
            }

            telemetry.addData("Mode", mode);
            telemetry.addLine("Expected for pure rotation: FL 135°  FR 45°  BL -135°  BR -45°");
            telemetry.addLine();
            drive.logDetailed(telemetry);
            telemetry.addLine();
            telemetry.addLine(CONTROLS);
            telemetry.update();
        }
    }
}
