package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.ControlConstants;
import org.firstinspires.ftc.teamcode.Constants.MotifConfig;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.Sorter;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrive;

@TeleOp(name = "Swerve Teleop", group = "Drive")
public class SwerveTeleop extends LinearOpMode {
    private Hardware hardware;
    private SwerveDrive drive;
    private Outtake outtake;
    private Intake intake;
    private Sorter sorter;
    private boolean lastCross = false;
    private boolean lastSquare = false;
    private boolean intakeOn = false;

    @Override
    public void runOpMode() {
        hardware = new Hardware(hardwareMap);
        drive = new SwerveDrive(hardwareMap);
        outtake = new Outtake(hardware);
        intake = new Intake(hardware);
        sorter = new Sorter(hardware, outtake);
        sorter.setMotif(MotifConfig.motifCode);

        telemetry.addLine("Swerve Drive Ready");
        telemetry.addData("Motif", sorter.getMotif().name());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean squarePressed = gamepad2.square && !lastSquare;
            lastSquare = gamepad2.square;
            if (squarePressed) {
                intakeOn = !intakeOn;
                if (intakeOn) intake.Start(0.9);
                else intake.Stop();
            }

            boolean crossPressed = gamepad2.cross && !lastCross;
            lastCross = gamepad2.cross;
            if (crossPressed && sorter.isIdle()) sorter.startShootSequence();
            if (sorter.isSequenceComplete()) sorter.resetSequence();

            if (intakeOn) sorter.checkIntake();
            sorter.update();
            outtake.update();

            if (gamepad2.dpad_up) drive.zero();

            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double rotation = -gamepad1.right_stick_x;
            if (Math.abs(forward) < ControlConstants.DEADBAND) forward = 0;
            if (Math.abs(strafe) < ControlConstants.DEADBAND) strafe = 0;
            if (Math.abs(rotation) < ControlConstants.DEADBAND) rotation = 0;
            drive.drive(forward, strafe, rotation);

            telemetry.addData("Fwd/Str/Rot", "%.1f / %.1f / %.1f", forward, strafe, rotation);
            telemetry.addData("Motif", sorter.getMotif().name());
            telemetry.addData("Slots", sorter.getSlotsString());
            telemetry.addData("Balls", sorter.getBallCount());
            telemetry.addData("Sorter", sorter.getShootState().name());
            telemetry.addData("Sensor", sorter.getSensorString());
            telemetry.addData("Turret RPM", "%.0f / %.0f", outtake.getCurrentRPM(), outtake.getTargetRPM());
            drive.log(telemetry);
            telemetry.update();
        }
    }
}
