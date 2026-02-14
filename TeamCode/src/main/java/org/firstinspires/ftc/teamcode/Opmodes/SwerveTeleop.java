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

    // NEW: Motif switching button states
    private boolean lastDpadLeft = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadRight = false;

    @Override
    public void runOpMode() {
        hardware = new Hardware(hardwareMap);
        drive = new SwerveDrive(hardwareMap);
        outtake = new Outtake(hardware);
        intake = new Intake(hardware);
        sorter = new Sorter(hardware, outtake);

        // Initial setup from config
        sorter.setMotif(MotifConfig.motifCode);

        telemetry.addLine("Swerve Drive Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- MOTIF SWITCHING (Gamepad 2) ---
            if (gamepad2.dpad_left && !lastDpadLeft) sorter.setMotif(21);   // Green 1st
            if (gamepad2.dpad_up && !lastDpadUp) sorter.setMotif(22);     // Green 2nd
            if (gamepad2.dpad_right && !lastDpadRight) sorter.setMotif(23); // Green 3rd

            lastDpadLeft = gamepad2.dpad_left;
            lastDpadUp = gamepad2.dpad_up;
            lastDpadRight = gamepad2.dpad_right;

            // --- INTAKE CONTROL ---
            boolean squarePressed = gamepad2.square && !lastSquare;
            lastSquare = gamepad2.square;
            if (squarePressed) intakeOn = !intakeOn;

            // Auto-stop intake if full
            if (intakeOn && sorter.getBallCount() < 3) {
                intake.Start(0.9);
            } else {
                intake.Stop();
                if (sorter.getBallCount() >= 3) intakeOn = false;
            }

            // --- SHOOTING ---
            boolean crossPressed = gamepad2.cross && !lastCross;
            lastCross = gamepad2.cross;
            if (crossPressed && sorter.isIdle()) sorter.startShootSequence();

            // --- SUBSYSTEM UPDATES ---
            if (intakeOn) sorter.checkIntake();
            sorter.update();
            outtake.update();

            // --- DRIVE CONTROL ---
            if (gamepad1.dpad_up) drive.zero(); // Re-zero on Gamepad 1

            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double rotation = -gamepad1.right_stick_x;

            if (Math.abs(forward) < ControlConstants.DEADBAND) forward = 0;
            if (Math.abs(strafe) < ControlConstants.DEADBAND) strafe = 0;
            if (Math.abs(rotation) < ControlConstants.DEADBAND) rotation = 0;

            drive.drive(forward, strafe, rotation);

            // --- TELEMETRY ---
            telemetry.addLine("=== CONFIG ===");
            // Shows current Motif setup
            String motifDesc = (sorter.getGreenFireIndex() == 0) ? "21 (G 1st)" :
                    (sorter.getGreenFireIndex() == 1) ? "22 (G 2nd)" : "23 (G 3rd)";
            telemetry.addData("ACTIVE MOTIF", motifDesc);

            telemetry.addLine("=== STATS ===");
            telemetry.addData("Slots", sorter.getSlotsString());
            telemetry.addData("Balls", sorter.getBallCount() + (sorter.getBallCount() >= 3 ? " [FULL]" : ""));
            telemetry.addData("Sorter State", sorter.getShootState().name());
            telemetry.addData("Turret RPM", "%.0f / %.0f", outtake.getCurrentRPM(), outtake.getTargetRPM());

            drive.log(telemetry);
            telemetry.update();
        }
    }
}