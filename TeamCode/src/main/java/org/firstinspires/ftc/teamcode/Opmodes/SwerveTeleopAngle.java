package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.ControlConstants;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrive;

@TeleOp(name = "Swerve Teleop (Angle)", group = "Drive")
public class SwerveTeleopAngle extends LinearOpMode {
    private SwerveDrive drive;
    private double lastAngleDegrees = 0.0;

    @Override
    public void runOpMode() {
        drive = new SwerveDrive(hardwareMap);

        telemetry.addLine("Robot-Centric Swerve Drive Ready");
        telemetry.addLine("Press START to reset wheels to 0 and begin.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Reset wheels to forward
            // Main teleop loop
            while (opModeIsActive()) {
                // Read joysticks
                double leftStickX = gamepad1.left_stick_x;
                double leftStickY = -gamepad1.left_stick_y; // Y stick is inverted
                double rotationInput = -gamepad1.right_stick_x;

                // Apply deadbands to all axes
                if (Math.abs(leftStickX) < ControlConstants.JOYSTICK_DEADBAND) leftStickX = 0;
                if (Math.abs(leftStickY) < ControlConstants.JOYSTICK_DEADBAND) leftStickY = 0;
                if (Math.abs(rotationInput) < ControlConstants.JOYSTICK_DEADBAND) rotationInput = 0;

                // Calculate magnitude and angle FOR DISPLAY ONLY
                double magnitude = Math.sqrt(leftStickX * leftStickX + leftStickY * leftStickY);

                // Calculate joystick angle (0° = right, 90° = up, 180° = left, 270° = down)
                double joystickAngleRadians = Math.atan2(leftStickY, leftStickX);
                double joystickAngleDegrees = Math.toDegrees(joystickAngleRadians);
                if (joystickAngleDegrees < 0) {
                    joystickAngleDegrees += 360;
                }

                // Calculate robot frame angle (0° = forward, 90° = left, 180° = back, 270° = right)
                // This is 90° rotated from joystick frame
                double robotAngleDegrees = joystickAngleDegrees + 90.0;
                if (robotAngleDegrees >= 360) {
                    robotAngleDegrees -= 360;
                }

                // Only update angle display if joystick is outside deadband
                if (magnitude > ControlConstants.JOYSTICK_DEADBAND) {
                    lastAngleDegrees = robotAngleDegrees;
                }

                // Direct mapping: joystick axes → robot axes (NO polar decomposition!)
                // Joystick: +Y = up, +X = right (after inversion)
                // Robot:    +X = forward, +Y = left
                double forward = leftStickY * ControlConstants.MAX_DRIVE_SPEED;
                double strafe = -leftStickX * ControlConstants.MAX_DRIVE_SPEED;
                double rotation = rotationInput * ControlConstants.MAX_ROTATION_SPEED;

                boolean isRotating = Math.abs(rotation) > 0;
                drive.drive(forward, strafe, rotation, !isRotating);

                // Telemetry
                telemetry.addData("Angle", "%.0f\u00b0", lastAngleDegrees);
                telemetry.addData("Fwd/Str/Rot", "%.1f / %.1f / %.1f", forward, strafe, rotation);
                drive.addTelemetry(telemetry);
                telemetry.update();
            }
        }

       // drive.hold();
    }
}
