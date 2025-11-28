package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.ControlConstants;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrive;

@TeleOp(name = "Swerve Teleop", group = "Drive")
public class SwerveTeleop extends LinearOpMode {
    private SwerveDrive drive;

    @Override
    public void runOpMode() {
        drive = new SwerveDrive(hardwareMap);

        telemetry.addLine("Robot-Centric Swerve Drive Ready");
        telemetry.addLine("Left Stick: Forward/Strafe");
        telemetry.addLine("Right Stick X: Rotation");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read joysticks (invert for WPILib coordinate system)
            double forward = -gamepad1.left_stick_y;      // Invert: stick up = +X forward
            double strafe = -gamepad1.left_stick_x;       // Invert: stick right = -Y (since +Y = left)
            double rotation = -gamepad1.right_stick_x;    // Invert: stick right = -rotation (CW)

            // Apply deadband
            if (Math.abs(forward) < ControlConstants.JOYSTICK_DEADBAND) forward = 0;
            if (Math.abs(strafe) < ControlConstants.JOYSTICK_DEADBAND) strafe = 0;
            if (Math.abs(rotation) < ControlConstants.JOYSTICK_DEADBAND) rotation = 0;

            // Scale to max speeds
            forward *= ControlConstants.MAX_DRIVE_SPEED;
            strafe *= ControlConstants.MAX_DRIVE_SPEED;
            rotation *= ControlConstants.MAX_ROTATION_SPEED;

            // Drive (robot-centric)
            drive.drive(forward, strafe, rotation);

            // Telemetry
            telemetry.addLine("=== INPUTS ===");
            telemetry.addData("Forward", "%.2f m/s", forward);
            telemetry.addData("Strafe", "%.2f m/s", strafe);
            telemetry.addData("Rotation", "%.2f rad/s", rotation);
            telemetry.addLine();
            drive.addTelemetry(telemetry);
            telemetry.update();
        }

        drive.stop();
    }
}
