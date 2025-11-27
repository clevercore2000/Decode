package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.ControlConstants;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

/**
 * Simple Swerve Drive TeleOp
 *
 * Robot-centric control only (no field-centric).
 * Just the basics - drive and game mechanisms.
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

    @Override
    public void runOpMode() {
        // Initialize hardware
        hardware = new Hardware(hardwareMap);
        outtake = new Outtake(hardware);
        intake = new Intake(hardware);

        telemetry.addLine("=========================");
        telemetry.addLine("  SWERVE DRIVE READY");
        telemetry.addLine("=========================");
        telemetry.addLine();
        telemetry.addLine("Mode: Robot-Centric");
        telemetry.addLine();
        telemetry.addLine("Left Stick: Drive");
        telemetry.addLine("Right Stick X: Rotate");
        telemetry.addLine();
        telemetry.addLine("Press START when ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ========== READ DRIVE INPUTS ==========
            // Coordinate system: +X=forward, +Y=left, +rotation=CCW
            // Gamepad: stick up=-Y, stick right=+X, right stick right=+X
            double forward = -gamepad1.left_stick_y;    // Stick up → +X (forward)
            double strafe = -gamepad1.left_stick_x;     // Stick right → -Y (right)
            double rotation = -gamepad1.right_stick_x;  // Stick right → -rotation (CW)

            // Apply deadband (prevents drift)
            forward = applyDeadband(forward, ControlConstants.JOYSTICK_DEADBAND);
            strafe = applyDeadband(strafe, ControlConstants.JOYSTICK_DEADBAND);
            rotation = applyDeadband(rotation, ControlConstants.JOYSTICK_DEADBAND);

            // Scale to max velocities
            forward *= DriveConstants.MAX_SPEED_METERS_PER_SECOND;
            strafe *= DriveConstants.MAX_SPEED_METERS_PER_SECOND;
            rotation *= DriveConstants.MAX_ANGULAR_VELOCITY;

            // ========== GAME MECHANISM CONTROLS ==========
            // Outtake control (Cross button)
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

            // Call rampShoot BEFORE update
            outtake.rampShoot(gamepad1.triangle);
            outtake.update();

            // Intake control (Square button)
            if (gamepad1.square) {
                intake.Start(0.9);
            } else {
                intake.Stop();
            }

            // ========== DRIVE CONTROL ==========
            hardware.swerveDrive.drive(forward, strafe, rotation);

            // ========== TELEMETRY ==========
            updateTelemetry(forward, strafe, rotation);
        }

        // Stop everything when exiting
        hardware.swerveDrive.stop();
        intake.Stop();
        outtake.stop();
    }

    /**
     * Apply deadband to joystick input
     *
     * @param value    Joystick value [-1.0, 1.0]
     * @param deadband Deadband threshold (e.g., 0.05)
     * @return Filtered value (0.0 if within deadband)
     */
    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        }
        return value;
    }

    /**
     * Update telemetry display
     *
     * Shows drive inputs and module states
     */
    private void updateTelemetry(double forward, double strafe, double rotation) {
        telemetry.addLine("=== SWERVE DRIVE ===");
        telemetry.addData("Mode", "Robot-Centric");
        telemetry.addData("Forward", "%.2f m/s", forward);
        telemetry.addData("Strafe", "%.2f m/s", strafe);
        telemetry.addData("Rotation", "%.2f rad/s", rotation);
        telemetry.addLine();

        // Module telemetry
        telemetry.addLine("=== MODULES ===");
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

        // Game mechanisms
        String outtakeStatus = outtake.isActive() ?
            String.format("ON @ %.0f RPM (Avg: %.0f)", outtake.getTargetRPM(), outtake.getAverageRPM()) :
            "OFF";
        telemetry.addData("Outtake", outtakeStatus);
        telemetry.addData("Intake", gamepad1.square ? "RUNNING" : "OFF");

        telemetry.update();
    }
}
