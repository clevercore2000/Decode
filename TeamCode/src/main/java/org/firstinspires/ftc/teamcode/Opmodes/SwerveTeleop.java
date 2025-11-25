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

            // Call rampShoot BEFORE update so state machine runs before execute()
            outtake.rampShoot(gamepad1.triangle);

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

        // === SWERVE DRIVE STATUS ===
        telemetry.addLine("=== SWERVE DRIVE ===");
        telemetry.addData("Mode", "Robot-Centric");
        telemetry.addData("Input", "F:%.2f S:%.2f R:%.2f", forward, strafe, rotation);

        // Module states (compact 1-line format)
        telemetry.addData("FL", "%.1f m/s @ %.0f°", moduleStates[0].speedMetersPerSecond, moduleStates[0].angle.getDegrees());
        telemetry.addData("FR", "%.1f m/s @ %.0f°", moduleStates[1].speedMetersPerSecond, moduleStates[1].angle.getDegrees());
        telemetry.addData("BL", "%.1f m/s @ %.0f°", moduleStates[2].speedMetersPerSecond, moduleStates[2].angle.getDegrees());
        telemetry.addData("BR", "%.1f m/s @ %.0f°", moduleStates[3].speedMetersPerSecond, moduleStates[3].angle.getDegrees());
        telemetry.addLine();

        // === GAME MECHANISMS (COMPACT) ===
        // Outtake status
        String outtakeStatus = outtake.isActive() ?
            String.format("ON @ %.0f RPM (Avg: %.0f)", outtake.getTargetRPM(), outtake.getAverageRPM()) :
            "OFF";
        telemetry.addData("Outtake", outtakeStatus);

        // Intake status
        telemetry.addData("Intake", gamepad1.square ? "RUNNING" : "OFF");

        telemetry.update();
    }

}
