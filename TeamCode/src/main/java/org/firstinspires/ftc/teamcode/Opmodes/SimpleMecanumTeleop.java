package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

/**
 * Simple Mecanum-Style Swerve TeleOp
 *
 * Joystick angle = module angle
 * All 4 modules point where the stick points
 * Robot moves in that direction
 */
@TeleOp(name = "Simple Mecanum Teleop")
public class SimpleMecanumTeleop extends LinearOpMode {

    @Override
    public void runOpMode() {
        Hardware hardware = new Hardware(hardwareMap);

        telemetry.addLine("Simple Mecanum Teleop");
        telemetry.addLine("Move left stick - robot follows!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read joystick (invert for correct coordinate system)
            double x = -gamepad1.left_stick_x;  // Right = positive
            double y = -gamepad1.left_stick_y;  // Up = positive

            // Calculate target angle and speed
            double targetAngle = Math.atan2(y, x);
            double speed = Math.hypot(x, y);

            // Apply deadband and command modules
            if (speed > 0.1) {
                // Scale speed to drive motor range
                speed = Math.min(speed, 1.0);

                // Command ALL modules to point at joystick angle
                hardware.swerveDrive.getFlModule().setDesiredState(speed, targetAngle);
                hardware.swerveDrive.getFrModule().setDesiredState(speed, targetAngle);
                hardware.swerveDrive.getBlModule().setDesiredState(speed, targetAngle);
                hardware.swerveDrive.getBrModule().setDesiredState(speed, targetAngle);
            } else {
                // Joystick neutral - stop driving but hold angle
                hardware.swerveDrive.getFlModule().setDesiredState(0.0, targetAngle);
                hardware.swerveDrive.getFrModule().setDesiredState(0.0, targetAngle);
                hardware.swerveDrive.getBlModule().setDesiredState(0.0, targetAngle);
                hardware.swerveDrive.getBrModule().setDesiredState(0.0, targetAngle);
            }

            // Telemetry - PID DEBUG
            telemetry.addLine("=== JOYSTICK (TARGET) ===");
            telemetry.addData("Target Angle", "%.1f°", Math.toDegrees(targetAngle));
            telemetry.addData("Speed", "%.2f", speed);
            telemetry.addLine();

            // FL module detailed
            double flCurrent = hardware.swerveDrive.getFlModule().getCurrentSteeringAngle();
            double flError = targetAngle - flCurrent;
            // Normalize error to [-π, π]
            while (flError > Math.PI) flError -= 2 * Math.PI;
            while (flError < -Math.PI) flError += 2 * Math.PI;

            telemetry.addLine("=== FL MODULE ===");
            telemetry.addData("Target", "%.1f°", Math.toDegrees(targetAngle));
            telemetry.addData("Current", "%.1f°", Math.toDegrees(flCurrent));
            telemetry.addData("Error", "%.1f°", Math.toDegrees(flError));
            telemetry.addData("Raw Voltage", "%.3fV", hardware.swerveDrive.getFlModule().getRawEncoderVoltage());
            telemetry.addData("Drive Power", "%.2f", hardware.swerveDrive.getFlModule().getDrivePower());
            telemetry.addLine();

            // FR module detailed
            double frCurrent = hardware.swerveDrive.getFrModule().getCurrentSteeringAngle();
            double frError = targetAngle - frCurrent;
            while (frError > Math.PI) frError -= 2 * Math.PI;
            while (frError < -Math.PI) frError += 2 * Math.PI;

            telemetry.addLine("=== FR MODULE ===");
            telemetry.addData("Target", "%.1f°", Math.toDegrees(targetAngle));
            telemetry.addData("Current", "%.1f°", Math.toDegrees(frCurrent));
            telemetry.addData("Error", "%.1f°", Math.toDegrees(frError));
            telemetry.addData("Raw Voltage", "%.3fV", hardware.swerveDrive.getFrModule().getRawEncoderVoltage());
            telemetry.addData("Drive Power", "%.2f", hardware.swerveDrive.getFrModule().getDrivePower());
            telemetry.addLine();

            // BL module detailed
            double blCurrent = hardware.swerveDrive.getBlModule().getCurrentSteeringAngle();
            double blError = targetAngle - blCurrent;
            while (blError > Math.PI) blError -= 2 * Math.PI;
            while (blError < -Math.PI) blError += 2 * Math.PI;

            telemetry.addLine("=== BL MODULE ===");
            telemetry.addData("Target", "%.1f°", Math.toDegrees(targetAngle));
            telemetry.addData("Current", "%.1f°", Math.toDegrees(blCurrent));
            telemetry.addData("Error", "%.1f°", Math.toDegrees(blError));
            telemetry.addData("Raw Voltage", "%.3fV", hardware.swerveDrive.getBlModule().getRawEncoderVoltage());
            telemetry.addData("Drive Power", "%.2f", hardware.swerveDrive.getBlModule().getDrivePower());
            telemetry.addLine();

            // BR module detailed
            double brCurrent = hardware.swerveDrive.getBrModule().getCurrentSteeringAngle();
            double brError = targetAngle - brCurrent;
            while (brError > Math.PI) brError -= 2 * Math.PI;
            while (brError < -Math.PI) brError += 2 * Math.PI;

            telemetry.addLine("=== BR MODULE ===");
            telemetry.addData("Target", "%.1f°", Math.toDegrees(targetAngle));
            telemetry.addData("Current", "%.1f°", Math.toDegrees(brCurrent));
            telemetry.addData("Error", "%.1f°", Math.toDegrees(brError));
            telemetry.addData("Raw Voltage", "%.3fV", hardware.swerveDrive.getBrModule().getRawEncoderVoltage());
            telemetry.addData("Drive Power", "%.2f", hardware.swerveDrive.getBrModule().getDrivePower());

            telemetry.update();
        }

        hardware.swerveDrive.stop();
    }
}
