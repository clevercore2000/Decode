package org.firstinspires.ftc.teamcode.Opmodes;

import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.ControlConstants;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

/**
 * Simple Swerve Drive Test
 *
 * Minimal OpMode for testing ONLY swerve drive functionality
 * No game mechanisms - pure drive control for debugging
 *
 * Controls:
 * - Left Stick: Forward/Strafe
 * - Right Stick X: Rotation
 */
@TeleOp(name = "Simple Swerve Test", group = "Test")
public class SimpleSwerveTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize hardware
        Hardware hardware = new Hardware(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Info", "Simple swerve test - drive only");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read joystick inputs
            double forward = -gamepad1.left_stick_y;   // Inverted for natural control
            double strafe = gamepad1.left_stick_x;
            double rotation = gamepad1.right_stick_x;

            // Apply deadband
            forward = applyDeadband(forward, ControlConstants.JOYSTICK_DEADBAND);
            strafe = applyDeadband(strafe, ControlConstants.JOYSTICK_DEADBAND);
            rotation = applyDeadband(rotation, ControlConstants.JOYSTICK_DEADBAND);

            // Scale to max speeds
            forward *= DriveConstants.MAX_SPEED_METERS_PER_SECOND;
            strafe *= DriveConstants.MAX_SPEED_METERS_PER_SECOND;
            rotation *= DriveConstants.MAX_ANGULAR_VELOCITY;

            // Drive
            hardware.swerveDrive.drive(forward, strafe, rotation, false);

            // Minimal telemetry
            updateTelemetry(forward, strafe, rotation, hardware);
        }

        hardware.swerveDrive.stop();
    }

    private double applyDeadband(double value, double deadband) {
        return Math.abs(value) < deadband ? 0.0 : value;
    }

    private void updateTelemetry(double forward, double strafe, double rotation, Hardware hardware) {
        SwerveModuleState[] states = hardware.swerveDrive.getModuleStates();

        telemetry.addLine("=== SWERVE TEST ===");
        telemetry.addData("Input", "F:%.2f S:%.2f R:%.2f", forward, strafe, rotation);
        telemetry.addLine();

        telemetry.addLine("Module States:");
        telemetry.addData("FL", "%.1f m/s @ %.0f° [pwr:%.2f]", states[0].speedMetersPerSecond, states[0].angle.getDegrees(), hardware.swerveDrive.getModule(0).getDrivePower());
        telemetry.addData("FR", "%.1f m/s @ %.0f° [pwr:%.2f]", states[1].speedMetersPerSecond, states[1].angle.getDegrees(), hardware.swerveDrive.getModule(1).getDrivePower());
        telemetry.addData("BL", "%.1f m/s @ %.0f° [pwr:%.2f]", states[2].speedMetersPerSecond, states[2].angle.getDegrees(), hardware.swerveDrive.getModule(2).getDrivePower());
        telemetry.addData("BR", "%.1f m/s @ %.0f° [pwr:%.2f]", states[3].speedMetersPerSecond, states[3].angle.getDegrees(), hardware.swerveDrive.getModule(3).getDrivePower());
        telemetry.addLine();

        // Show raw encoder voltages for debugging
        telemetry.addLine("Encoder Voltages:");
        for (int i = 0; i < 4; i++) {
            String[] names = {"FL", "FR", "BL", "BR"};
            telemetry.addData(names[i], "%.2fV", hardware.swerveDrive.getModule(i).getRawEncoderVoltage());
        }

        telemetry.update();
    }
}
