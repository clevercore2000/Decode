package org.firstinspires.ftc.teamcode.Opmodes.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.AutoConstants;
import org.firstinspires.ftc.teamcode.Constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.pedro.PedroConstants;
import org.firstinspires.ftc.teamcode.pedro.PedroSwerveDrivetrain;

/**
 * Autonomous routine: score the preload, collect, score again, park.
 * <p>
 * Structured as a per-loop state machine in the same shape as the teleop opmodes — one
 * {@code follower.update()} and one {@code telemetry.update()} per iteration, subsystems
 * driven by their own {@code update()} calls — rather than a sequence of blocking waits, so
 * the outtake keeps spinning and the follower keeps correcting while a state waits out a timer.
 * <p>
 * Modules are homed during init: {@link SwerveDrive#homeAllModules(LinearOpMode)} gates on
 * {@code opModeIsActive()} and so cannot be used before start, and homing after start would
 * spend seconds of the 30 second period with the robot stationary.
 */
@Autonomous(name = "Decode Auto", group = "Auto")
public class DecodeAuto extends LinearOpMode {

    private enum State {
        DRIVE_TO_SCORE,
        SPIN_UP,
        SHOOT,
        DRIVE_TO_COLLECT,
        COLLECT,
        DRIVE_TO_SCORE_2,
        SPIN_UP_2,
        SHOOT_2,
        PARK,
        DONE
    }

    private Hardware hardware;
    private SwerveDrive drive;
    private Follower follower;
    private PedroSwerveDrivetrain drivetrain;
    private Outtake outtake;
    private Intake intake;

    private PathChain toScore, toCollect, toScore2, toPark;

    private State state = State.DRIVE_TO_SCORE;
    private final ElapsedTime stateTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        hardware = new Hardware(hardwareMap);
        drive = new SwerveDrive(hardwareMap);
        outtake = new Outtake(hardware);
        intake = new Intake(hardware);
        drivetrain = new PedroSwerveDrivetrain(drive, hardwareMap);
        follower = PedroConstants.createFollower(hardwareMap, drivetrain);

        telemetry.addLine("Homing swerve modules (init)...");
        telemetry.update();

        boolean homed = drive.homeAllModulesDuringInit(this);

        // Settle the wheels at 0° before start. Homing only requests that angle; without this
        // the first path opens by rotating the modules into line while already under power,
        // which costs accuracy exactly where the routine can least afford it.
        telemetry.addLine("Aligning modules to 0°...");
        telemetry.update();
        boolean aligned = homed && drive.alignModulesForwardDuringInit(this);

        follower.setStartingPose(AutoConstants.START);
        buildPaths();

        outtake.resetRamp();

        while (opModeInInit()) {
            drive.holdForward();

            telemetry.addData("Homed", homed ? "OK" : "FAILED — do not run");
            telemetry.addData("Aligned", aligned ? "OK — wheels at 0°" : "NOT SETTLED — expect start error");
            telemetry.addData("Start Pose", "%.1f, %.1f, %.0f°",
                    AutoConstants.START.getX(), AutoConstants.START.getY(),
                    Math.toDegrees(AutoConstants.START.getHeading()));
            telemetry.addLine("Ready.");
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        setState(State.DRIVE_TO_SCORE);
        follower.followPath(toScore);

        while (opModeIsActive()) {
            follower.update();
            advance();
            outtake.update();

            telemetry.addData("State", state);
            telemetry.addData("State Time", "%.1f s", stateTimer.seconds());
            telemetry.addData("Pose", "%.1f, %.1f, %.0f°",
                    follower.getPose().getX(), follower.getPose().getY(),
                    Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("Following", follower.isBusy());
            telemetry.addLine(drivetrain.debugString());
            outtake.log(telemetry);
            intake.log(telemetry);
            telemetry.update();
        }
    }

    private void buildPaths() {
        toScore = follower.pathBuilder()
                .addPath(new BezierLine(AutoConstants.START, AutoConstants.SCORE))
                .setLinearHeadingInterpolation(AutoConstants.START.getHeading(), AutoConstants.SCORE.getHeading())
                .build();

        toCollect = follower.pathBuilder()
                .addPath(new BezierLine(AutoConstants.SCORE, AutoConstants.COLLECT))
                .setLinearHeadingInterpolation(AutoConstants.SCORE.getHeading(), AutoConstants.COLLECT.getHeading())
                .build();

        toScore2 = follower.pathBuilder()
                .addPath(new BezierLine(AutoConstants.COLLECT, AutoConstants.SCORE))
                .setLinearHeadingInterpolation(AutoConstants.COLLECT.getHeading(), AutoConstants.SCORE.getHeading())
                .build();

        toPark = follower.pathBuilder()
                .addPath(new BezierLine(AutoConstants.SCORE, AutoConstants.PARK))
                .setLinearHeadingInterpolation(AutoConstants.SCORE.getHeading(), AutoConstants.PARK.getHeading())
                .build();
    }

    private void setState(State next) {
        state = next;
        stateTimer.reset();
    }

    private void advance() {
        switch (state) {
            case DRIVE_TO_SCORE:
                // Spin the outtake up while still driving so SPIN_UP has less to wait for.
                outtake.setTargetRPM(OuttakeConstants.TARGET_RPM);
                if (!follower.isBusy()) setState(State.SPIN_UP);
                break;

            case SPIN_UP:
                outtake.setTargetRPM(OuttakeConstants.TARGET_RPM);
                // Open loop: the outtake shares its encoder pins with the swerve steering
                // encoders, so there is no velocity feedback to wait on — only a timer.
                if (stateTimer.milliseconds() >= AutoConstants.SPINUP_MS) {
                    outtake.rampShoot(true);
                    setState(State.SHOOT);
                }
                break;

            case SHOOT:
                if (stateTimer.milliseconds() >= AutoConstants.SHOOT_MS) {
                    outtake.rampShoot(false);
                    outtake.resetRamp();
                    outtake.setTargetRPM(0);
                    follower.followPath(toCollect);
                    setState(State.DRIVE_TO_COLLECT);
                }
                break;

            case DRIVE_TO_COLLECT:
                if (!follower.isBusy()) {
                    intake.Start(0.9);
                    setState(State.COLLECT);
                }
                break;

            case COLLECT:
                if (stateTimer.milliseconds() >= AutoConstants.COLLECT_MS) {
                    intake.Stop();
                    follower.followPath(toScore2);
                    setState(State.DRIVE_TO_SCORE_2);
                }
                break;

            case DRIVE_TO_SCORE_2:
                outtake.setTargetRPM(OuttakeConstants.TARGET_RPM);
                if (!follower.isBusy()) setState(State.SPIN_UP_2);
                break;

            case SPIN_UP_2:
                outtake.setTargetRPM(OuttakeConstants.TARGET_RPM);
                if (stateTimer.milliseconds() >= AutoConstants.SPINUP_MS) {
                    outtake.rampShoot(true);
                    setState(State.SHOOT_2);
                }
                break;

            case SHOOT_2:
                if (stateTimer.milliseconds() >= AutoConstants.SHOOT_MS) {
                    outtake.rampShoot(false);
                    outtake.resetRamp();
                    outtake.setTargetRPM(0);
                    follower.followPath(toPark);
                    setState(State.PARK);
                }
                break;

            case PARK:
                if (!follower.isBusy()) {
                    outtake.stop();
                    intake.Stop();
                    setState(State.DONE);
                }
                break;

            case DONE:
                break;
        }
    }
}
