package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

/**
 * Simple Module Angle Test
 *
 * Press buttons to command all modules to specific angles:
 * - Cross (X): 0° (forward)
 * - Circle (O): 90° (left)
 * - Square: 180° (backward)
 * - Triangle: -90° (right)
 */
@TeleOp(name = "Module Angle Test")
public class SwerveModuleAngleTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Hardware hardware = new Hardware(hardwareMap);

        telemetry.addLine("=========================");
        telemetry.addLine("  MODULE ANGLE TEST");
        telemetry.addLine("=========================");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("Cross (X): 0° (forward)");
        telemetry.addLine("Circle (O): 90° (left)");
        telemetry.addLine("Square: 180° (backward)");
        telemetry.addLine("Triangle: -90° (right)");
        telemetry.addLine();
        telemetry.addLine("Press START when ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double targetAngle = 0.0;
            String command = "NONE";

            // Read button inputs
            if (gamepad1.cross) {
                targetAngle = 0.0;  // 0° = forward
                command = "0° (FORWARD)";
            } else if (gamepad1.circle) {
                targetAngle = Math.PI / 2;  // 90° = left
                command = "90° (LEFT)";
            } else if (gamepad1.square) {
                targetAngle = Math.PI;  // 180° = backward
                command = "180° (BACKWARD)";
            } else if (gamepad1.triangle) {
                targetAngle = -Math.PI / 2;  // -90° = right
                command = "-90° (RIGHT)";
            }

            // If a button is pressed, command all modules to that angle with 0 speed
            if (!command.equals("NONE")) {
                hardware.swerveDrive.getFlModule().setDesiredState(0.0, targetAngle);
                hardware.swerveDrive.getFrModule().setDesiredState(0.0, targetAngle);
                hardware.swerveDrive.getBlModule().setDesiredState(0.0, targetAngle);
                hardware.swerveDrive.getBrModule().setDesiredState(0.0, targetAngle);
            }

            // Display telemetry
            telemetry.addLine("=== COMMAND ===");
            telemetry.addData("Target Angle", command);
            telemetry.addLine();

            telemetry.addLine("=== MODULE ANGLES ===");
            telemetry.addData("FL", "%.1f° (%.3fV)",
                Math.toDegrees(hardware.swerveDrive.getFlModule().getCurrentSteeringAngle()),
                hardware.swerveDrive.getFlModule().getRawEncoderVoltage()
            );
            telemetry.addData("FR", "%.1f° (%.3fV)",
                Math.toDegrees(hardware.swerveDrive.getFrModule().getCurrentSteeringAngle()),
                hardware.swerveDrive.getFrModule().getRawEncoderVoltage()
            );
            telemetry.addData("BL", "%.1f° (%.3fV)",
                Math.toDegrees(hardware.swerveDrive.getBlModule().getCurrentSteeringAngle()),
                hardware.swerveDrive.getBlModule().getRawEncoderVoltage()
            );
            telemetry.addData("BR", "%.1f° (%.3fV)",
                Math.toDegrees(hardware.swerveDrive.getBrModule().getCurrentSteeringAngle()),
                hardware.swerveDrive.getBrModule().getRawEncoderVoltage()
            );
            telemetry.addLine();

            telemetry.addLine("=== DRIVE POWERS ===");
            telemetry.addData("FL", "%.2f", hardware.swerveDrive.getFlModule().getDrivePower());
            telemetry.addData("FR", "%.2f", hardware.swerveDrive.getFrModule().getDrivePower());
            telemetry.addData("BL", "%.2f", hardware.swerveDrive.getBlModule().getDrivePower());
            telemetry.addData("BR", "%.2f", hardware.swerveDrive.getBrModule().getDrivePower());

            telemetry.update();
        }

        hardware.swerveDrive.stop();
    }
}
