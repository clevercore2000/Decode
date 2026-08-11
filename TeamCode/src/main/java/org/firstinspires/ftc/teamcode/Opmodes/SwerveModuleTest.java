package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrive;

/**
 * Points all four modules at a commanded azimuth so steering direction, homing and tick
 * offsets can be eyeballed.
 * <p>
 * This builds the real {@link SwerveDrive} rather than assembling modules itself, so it
 * always exercises the same inversions and offsets the robot actually drives with.
 */
@Disabled
@TeleOp(name = "Swerve Module Test", group = "Test")
public class SwerveModuleTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        SwerveDrive drive = new SwerveDrive(hardwareMap);

        telemetry.addLine("Swerve Module Test");
        telemetry.addLine("Press START to home and begin");
        telemetry.update();

        waitForStart();

        telemetry.addLine("Homing...");
        telemetry.update();

        if (!drive.homeAllModules(this)) {
            telemetry.addLine("WARNING: Not all modules homed successfully!");
            telemetry.update();
            sleep(2000);
        }

        double targetDegrees = 0;

        while (opModeIsActive()) {
            if (gamepad1.dpad_right) { targetDegrees += 5; sleep(100); }
            if (gamepad1.dpad_left) { targetDegrees -= 5; sleep(100); }
            if (gamepad1.right_bumper) { targetDegrees += 45; sleep(200); }
            if (gamepad1.left_bumper) { targetDegrees -= 45; sleep(200); }
            if (gamepad1.a) { targetDegrees = 0; sleep(200); }

            while (targetDegrees >= 360) targetDegrees -= 360;
            while (targetDegrees < 0) targetDegrees += 360;

            double targetRad = Math.toRadians(targetDegrees);
            drive.fl.setTarget(targetRad, 0);
            drive.fr.setTarget(targetRad, 0);
            drive.bl.setTarget(targetRad, 0);
            drive.br.setTarget(targetRad, 0);

            drive.update();

            telemetry.addLine("Target: " + String.format("%.0f°", targetDegrees));
            telemetry.addLine("DPad L/R: ±5° | Bumpers: ±45° | A: Reset");
            telemetry.addLine();
            drive.log(telemetry);
            telemetry.update();
        }
    }
}
