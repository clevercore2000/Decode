package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Joystick Angle 360 Test", group = "Swerve")
public class JoystickAngle360Test extends LinearOpMode {

    private double lastAngleDegrees = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        while (opModeIsActive()) {
            double leftStickX = gamepad1.left_stick_x;
            double leftStickY = -gamepad1.left_stick_y; // Y stick is inverted

            // Only update angle if joystick is outside of deadband
            if (Math.sqrt(leftStickX * leftStickX + leftStickY * leftStickY) > 0.1) {
                // Calculate angle in radians from -PI to PI
                double angleRadians = Math.atan2(leftStickX, leftStickY);

                // Convert to degrees from -180 to 180
                double angleDegrees = Math.toDegrees(angleRadians);

                // Convert to 0 to 360 degrees
                if (angleDegrees < 0) {
                    angleDegrees += 360;
                }
                lastAngleDegrees = angleDegrees;
            }

            telemetry.addData("Left Stick X", "%.3f", leftStickX);
            telemetry.addData("Left Stick Y", "%.3f", leftStickY);
            telemetry.addData("Joystick Angle (0-360)", "%.1f", lastAngleDegrees);
            telemetry.update();
        }
    }
}