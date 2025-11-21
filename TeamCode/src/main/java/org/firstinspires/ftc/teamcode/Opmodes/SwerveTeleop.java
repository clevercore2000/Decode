package org.firstinspires.ftc.teamcode.Opmodes;

import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.ControlConstants;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

/**
 * Swerve Drive TeleOp
 *
 * Controls:
 * - Left Stick: Forward/Backward/Strafe
 * - Right Stick X: Rotation
 * - Cross: Outtake (2700 RPM default, +L_Bumper = 1500, +R_Bumper = 4000)
 * - Square: Intake
 */
@TeleOp(name = "Swerve TeleOp")
public class SwerveTeleop extends LinearOpMode {

    private Hardware hardware;

    private Outtake outtake;

    private Intake intake;

    // Uncomment for field-centric drive
    // private boolean fieldCentric = false;
    // private boolean lastFieldCentricButton = false;

    @Override
    public void runOpMode() {
        hardware = new Hardware(hardwareMap);
        outtake = new Outtake(hardware);
        intake = new Intake(hardware);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Mode", "Robot-Centric");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ========== READ INPUTS ==========
            // Inverted Y for natural forward/backward
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotation = gamepad1.right_stick_x;

            forward = applyDeadband(forward, ControlConstants.JOYSTICK_DEADBAND);
            strafe = applyDeadband(strafe, ControlConstants.JOYSTICK_DEADBAND);
            rotation = applyDeadband(rotation, ControlConstants.JOYSTICK_DEADBAND);

            forward *= DriveConstants.MAX_SPEED_METERS_PER_SECOND;
            strafe *= DriveConstants.MAX_SPEED_METERS_PER_SECOND;
            rotation *= DriveConstants.MAX_ANGULAR_VELOCITY;

            // ========== SUBSYSTEM CONTROLS ==========
            //TODO this is only for testing, proper controls need to be decided upon by the driving team
            if (gamepad1.cross) {
                if (gamepad1.left_bumper) {
                    outtake.setTargetRPM(1500);
                } else if (gamepad1.right_bumper) {
                    outtake.setTargetRPM(4000);
                } else {
                    outtake.setTargetRPM(OuttakeConstants.TARGET_RPM);
                }
            } else {
                outtake.stop();
            }

            outtake.update();

            if (gamepad1.square) {
                intake.Start(0.9);
            } else {
                intake.Stop();
            }

            //TODO Uncomment for field-centric drive
            /*
            boolean currentFieldCentricButton = gamepad1.right_bumper;
            if (currentFieldCentricButton && !lastFieldCentricButton) {
                fieldCentric = !fieldCentric;
            }
            lastFieldCentricButton = currentFieldCentricButton;

            if (gamepad1.options) {
                hardware.swerveDrive.zeroHeading();
            }
            */

            // ========== DRIVE CONTROL ==========
           hardware.swerveDrive.drive(forward, strafe, rotation, false);

            // ========== TELEMETRY ==========
            updateTelemetry(forward, strafe, rotation);
        }

        hardware.swerveDrive.stop();
        intake.Stop();
        outtake.stop();
    }

    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        }
        return value;
    }

    private void updateTelemetry(double forward, double strafe, double rotation) {
        // Get module states
        SwerveModuleState[] moduleStates = hardware.swerveDrive.getModuleStates();

        // Display drive mode
        telemetry.addData("Mode", "Robot-Centric");
        telemetry.addLine();

        // Display live drive velocities
        telemetry.addLine("=== DRIVE VELOCITIES ===");
        telemetry.addData("Forward", "%.2f m/s", forward);
        telemetry.addData("Strafe", "%.2f m/s", strafe);
        telemetry.addData("Rotation", "%.2f rad/s", rotation);
        telemetry.addLine();

        // Display outtake status (PID-controlled, 2 motors)
        telemetry.addLine("=== OUTTAKE (PID) ===");

        // Debug: show active buttons
        String buttonsPressed = "";
        if (gamepad1.cross) buttonsPressed += "CROSS ";
        if (gamepad1.left_bumper) buttonsPressed += "L_BUMP ";
        if (gamepad1.right_bumper) buttonsPressed += "R_BUMP ";
        if (buttonsPressed.isEmpty()) buttonsPressed = "NONE";
        telemetry.addData("Buttons", buttonsPressed);
        telemetry.addLine();

        // Determine current speed mode
        String speedMode = "OFF";
        double targetRPM = outtake.getTargetRPM();
        if (targetRPM > 0) {
            if (targetRPM <= 1500) {
                speedMode = "SLOW";
            } else if (targetRPM >= 4000) {
                speedMode = "FAST";
            } else {
                speedMode = "NORMAL";
            }
        }

        // State information
        telemetry.addData("State", outtake.isActive() ? "ACTIVE" : "STOPPED");
        telemetry.addData("Mode", speedMode);
        telemetry.addData("Target RPM", "%.0f", targetRPM);
        telemetry.addLine();

        // Live velocity information for both motors
        telemetry.addData("Motor 1 RPM", "%.0f", outtake.getCurrentRPM1());
        telemetry.addData("M1 Error", "%.0f RPM", outtake.getError1());
        telemetry.addData("M1 Power", "%.2f", outtake.getMotorPower1());
        telemetry.addLine();

        telemetry.addData("Motor 2 RPM", "%.0f", outtake.getCurrentRPM2());
        telemetry.addData("M2 Error", "%.0f RPM", outtake.getError2());
        telemetry.addData("M2 Power", "%.2f", outtake.getMotorPower2());
        telemetry.addLine();

        telemetry.addData("Average RPM", "%.0f", outtake.getAverageRPM());
        telemetry.addData("At Target", outtake.isAtTargetSpeed() ? "YES" : "NO");
        telemetry.addLine();

        // Display module states
        telemetry.addLine("=== SWERVE MODULES ===");
        displayModuleState("FL", moduleStates[0]);
        displayModuleState("FR", moduleStates[1]);
        displayModuleState("BL", moduleStates[2]);
        displayModuleState("BR", moduleStates[3]);
        telemetry.addLine();

        // Display controls
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("Drive:");
        telemetry.addLine("  Left Stick: Forward/Strafe");
        telemetry.addLine("  Right Stick X: Rotation");
        telemetry.addLine();
        telemetry.addLine("Outtake Speed:");
        telemetry.addLine("  Cross ONLY = 2700 RPM");
        telemetry.addLine("  Cross + L Bump = 1500 RPM");
        telemetry.addLine("  Cross + R Bump = 4000 RPM");
        telemetry.addLine();
        telemetry.addLine("Intake:");
        telemetry.addLine("  Square: Run intake");

        telemetry.update();
    }

    private void displayModuleState(String name, SwerveModuleState state) {
        telemetry.addData(
            name,
            "%.2f m/s @ %.0f°",
            state.speedMetersPerSecond,
            state.angle.getDegrees()
        );
    }
}
