package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

/**
 * Swerve Drive Calibration
 *
 * Instructions:
 * 1. Manually rotate all 4 wheels to point STRAIGHT FORWARD (toward front of robot)
 * 2. Press START
 * 3. Copy the displayed offset values to SteeringConstants.java
 * 4. Redeploy code
 */
@TeleOp(name = "Swerve Calibration")
public class SwerveCalibration extends LinearOpMode {

    @Override
    public void runOpMode() {
        Hardware hardware = new Hardware(hardwareMap);

        telemetry.addLine("====================================");
        telemetry.addLine("    SWERVE CALIBRATION");
        telemetry.addLine("====================================");
        telemetry.addLine();
        telemetry.addLine("STEP 1:");
        telemetry.addLine("Manually rotate ALL 4 wheels");
        telemetry.addLine("to point STRAIGHT FORWARD");
        telemetry.addLine("(toward front bumper/intake)");
        telemetry.addLine();
        telemetry.addLine("STEP 2:");
        telemetry.addLine("Press START");
        telemetry.addLine();
        telemetry.addLine("STEP 3:");
        telemetry.addLine("Copy the offset values to");
        telemetry.addLine("SteeringConstants.java");
        telemetry.addLine("====================================");
        telemetry.update();

        waitForStart();

        // Keep servos powered so encoders work (CRITICAL!)
        // Set minimum power to keep electronics active without moving
        hardware.swerveDrive.getFlModule().setDesiredState(0.0, 0.0);
        hardware.swerveDrive.getFrModule().setDesiredState(0.0, 0.0);
        hardware.swerveDrive.getBlModule().setDesiredState(0.0, 0.0);
        hardware.swerveDrive.getBrModule().setDesiredState(0.0, 0.0);

        // Small delay to let encoder readings stabilize
        sleep(500);

        // Read raw encoder voltages
        double flVoltage = hardware.swerveDrive.getFlModule().getRawEncoderVoltage();
        double frVoltage = hardware.swerveDrive.getFrModule().getRawEncoderVoltage();
        double blVoltage = hardware.swerveDrive.getBlModule().getRawEncoderVoltage();
        double brVoltage = hardware.swerveDrive.getBrModule().getRawEncoderVoltage();

        // Calculate offsets (voltage → radians)
        // offset = (voltage / 3.3) × 2π
        double flOffset = (flVoltage / 3.3) * 2 * Math.PI;
        double frOffset = (frVoltage / 3.3) * 2 * Math.PI;
        double blOffset = (blVoltage / 3.3) * 2 * Math.PI;
        double brOffset = (brVoltage / 3.3) * 2 * Math.PI;

        while (opModeIsActive()) {
            telemetry.addLine("====================================");
            telemetry.addLine("    CALIBRATION COMPLETE!");
            telemetry.addLine("====================================");
            telemetry.addLine();
            telemetry.addLine("Copy these values to");
            telemetry.addLine("SteeringConstants.java:");
            telemetry.addLine();
            telemetry.addData("FL_ANGLE_OFFSET", "%.4f", flOffset);
            telemetry.addData("FR_ANGLE_OFFSET", "%.4f", frOffset);
            telemetry.addData("BL_ANGLE_OFFSET", "%.4f", blOffset);
            telemetry.addData("BR_ANGLE_OFFSET", "%.4f", brOffset);
            telemetry.addLine();
            telemetry.addLine("====================================");
            telemetry.addLine("Raw Encoder Voltages:");
            telemetry.addData("FL", "%.3fV", flVoltage);
            telemetry.addData("FR", "%.3fV", frVoltage);
            telemetry.addData("BL", "%.3fV", blVoltage);
            telemetry.addData("BR", "%.3fV", brVoltage);
            telemetry.addLine("====================================");
            telemetry.addLine();
            telemetry.addLine("Press STOP when done copying values");
            telemetry.update();

            sleep(100);
        }

        hardware.swerveDrive.stop();
    }
}
