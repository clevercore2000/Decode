package org.firstinspires.ftc.teamcode.Opmodes.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.pedro.PedroConstants;
import org.firstinspires.ftc.teamcode.pedro.PedroSwerveDrivetrain;

/**
 * Drives a single straight line from (0, 0) to a target you choose, to check that forward and
 * strafe actually go where they claim to.
 * <p>
 * Heading is held at {@link #TARGET_HEADING_DEG} for the whole path rather than interpolated,
 * so any error you see is translation error and not rotation bleeding into it. With the
 * heading left at 0 the robot faces along +x, so the field axes line up with the robot's own:
 * <ul>
 *   <li>{@code TARGET_X = 24, TARGET_Y = 0} — 24" straight <b>forward</b></li>
 *   <li>{@code TARGET_X = 0, TARGET_Y = 24} — 24" <b>left</b> (strafe)</li>
 *   <li>both non-zero — a diagonal, which is the case that catches a swapped or
 *       sign-flipped axis that the two pure runs can both pass</li>
 * </ul>
 * Targets are {@code @Configurable}, and the path is rebuilt each time you trigger a run, so
 * you can retune from the dashboard without redeploying.
 * <p>
 * Controls (PlayStation labels): <b>X</b> runs to the target, <b>O</b> runs back to the origin,
 * <b>▢</b> cancels and re-zeroes the pose to where the robot currently sits.
 * <p>
 * Modules are homed <i>and settled at 0°</i> during init, and actively held there until a run
 * starts — homing alone only requests 0°, it does not drive the servos there.
 * <p>
 * Run this before {@code Decode Auto} — it is step 6 of the bring-up order in CLAUDE.md, and it
 * only means anything once {@code Pinpoint Localization Test} has confirmed the pose tracks.
 */
@Configurable
@TeleOp(name = "Translation Test", group = "Tuning")
public class TranslationTest extends LinearOpMode {

    /** Target X in inches — positive is forward when heading is 0. */
    public static double TARGET_X = 12.0;

    /** Target Y in inches — positive is left when heading is 0. */
    public static double TARGET_Y = 0.0;

    /**
     * Heading held for the entire path, degrees CCW. Leave at 0 for a pure translation test;
     * a non-zero value makes the robot translate while facing elsewhere, which mixes the
     * heading controller back into what you are trying to measure.
     */
    public static double TARGET_HEADING_DEG = 0.0;

    /** Follower power ceiling for this test. Keep it low until translation is trusted. */
    public static double TEST_MAX_POWER = 0.4;

    private static final Pose ORIGIN = new Pose(0, 0, 0);

    /** PlayStation labels, matching the gamepad the teleop opmodes are written against. */
    private static final String CONTROLS = "X: run to target | O: run to origin | ▢: cancel + re-zero";

    private SwerveDrive drive;
    private Follower follower;
    private PedroSwerveDrivetrain drivetrain;

    private boolean lastA, lastB, lastX;

    /** True while a path is being followed or held, i.e. while the follower owns the drivetrain. */
    private boolean running = false;

    /** Result of the most recent pre-run alignment, surfaced in telemetry. */
    private boolean runAligned = false;

    @Override
    public void runOpMode() {
        drive = new SwerveDrive(hardwareMap);
        drivetrain = new PedroSwerveDrivetrain(drive, hardwareMap);
        follower = PedroConstants.createFollower(hardwareMap, drivetrain);

        telemetry.addLine("Translation Test — homing modules...");
        telemetry.update();

        // Home in init: the follower will command the modules the moment a run starts, and an
        // unhomed module ignores every command it is given.
        boolean homed = drive.homeAllModulesDuringInit(this);

        // Homing leaves the modules at their limit switches with 0° merely *requested*. Settle
        // them there now, so the first path does not spend its opening inches rotating the
        // wheels into line while already driving.
        telemetry.addLine("Aligning modules to 0°...");
        telemetry.update();
        boolean aligned = homed && drive.alignModulesForwardDuringInit(this);

        follower.setStartingPose(ORIGIN);

        while (opModeInInit()) {
            // Keep the steering loop live so they hold 0° rather than drifting while you read
            // telemetry and decide when to start.
            drive.holdForward();

            telemetry.addData("Homed", homed ? "OK" : "FAILED — do not run");
            telemetry.addData("Aligned", aligned ? "OK — wheels at 0°" : "NOT SETTLED — expect start error");
            telemetry.addData("Steer error", "FL %.1f°  FR %.1f°  BL %.1f°  BR %.1f°",
                    Math.toDegrees(drive.fl.getSteerErrorRad()),
                    Math.toDegrees(drive.fr.getSteerErrorRad()),
                    Math.toDegrees(drive.bl.getSteerErrorRad()),
                    Math.toDegrees(drive.br.getSteerErrorRad()));
            telemetry.addData("Target", "%.1f, %.1f @ %.0f°", TARGET_X, TARGET_Y, TARGET_HEADING_DEG);
            telemetry.addLine(CONTROLS);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        follower.setMaxPower(TEST_MAX_POWER);

        while (opModeIsActive()) {
            boolean a = gamepad1.cross, b = gamepad1.circle, x = gamepad1.square;

            if (a && !lastA) runTo(target());
            if (b && !lastB) runTo(ORIGIN);
            if (x && !lastX) {
                follower.breakFollowing();
                running = false;
                // Re-zero so the next run starts from wherever the robot actually is, which
                // lets you chain runs without carrying accumulated error forward.
                follower.setPose(ORIGIN);
            }

            lastA = a;
            lastB = b;
            lastX = x;

            follower.update();

            if (!running) {
                // Nothing is being followed, so park the wheels at 0° rather than leaving them
                // wherever the follower last wrote. Runs after follower.update() so this write
                // is the one that reaches the hardware.
                drive.holdForward();
            }

            Pose pose = follower.getPose();
            Pose goal = target();

            telemetry.addData("Target", "%.1f, %.1f @ %.0f°", TARGET_X, TARGET_Y, TARGET_HEADING_DEG);
            telemetry.addData("Pose", "%.2f, %.2f @ %.1f°",
                    pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
            telemetry.addData("Error to target", "%.2f, %.2f @ %.1f°",
                    goal.getX() - pose.getX(),
                    goal.getY() - pose.getY(),
                    Math.toDegrees(goal.getHeading() - pose.getHeading()));
            telemetry.addData("Following", follower.isBusy());
            telemetry.addData("Pre-run align", running ? (runAligned ? "OK" : "TIMED OUT — steering lacks authority under load") : "idle, holding 0°");
            telemetry.addData("Steer error", "FL %.1f°  FR %.1f°  BL %.1f°  BR %.1f°",
                    Math.toDegrees(drive.fl.getSteerErrorRad()),
                    Math.toDegrees(drive.fr.getSteerErrorRad()),
                    Math.toDegrees(drive.bl.getSteerErrorRad()),
                    Math.toDegrees(drive.br.getSteerErrorRad()));
            telemetry.addLine(drivetrain.debugString());
            telemetry.addLine();
            telemetry.addLine(CONTROLS);
            telemetry.update();
        }
    }

    private Pose target() {
        return new Pose(TARGET_X, TARGET_Y, Math.toRadians(TARGET_HEADING_DEG));
    }

    /**
     * Builds and starts a fresh straight-line path from wherever the robot is now. Rebuilding
     * per run (rather than once at init) is what makes the dashboard-tuned target take effect
     * without a redeploy.
     */
    private void runTo(Pose goal) {
        follower.setMaxPower(TEST_MAX_POWER);

        // Re-align on the spot, under whatever load the robot is actually sitting on, and hold
        // for ALIGN_HOLD_MS before any drive power is applied. The init-time alignment is not
        // enough on its own: if the robot was held off the ground during init, setting it down
        // can knock the modules out of line, and alignment reached in the air says nothing
        // about what the servos can do against carpet.
        telemetry.addLine("Aligning before run...");
        telemetry.update();
        runAligned = drive.alignModulesForward(this);

        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), goal))
                .setConstantHeadingInterpolation(goal.getHeading())
                .build();

        // holdEnd = true: the follower keeps correcting onto the target after arriving, so the
        // number left on screen is real steady-state error rather than wherever it coasted to.
        follower.followPath(path, true);
        running = true;
    }
}
