package org.firstinspires.ftc.teamcode.Opmodes;

import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveModule;
import org.firstinspires.ftc.teamcode.Constants.SteeringConstants;


@TeleOp(name = "Swerve Calibration", group = "Calibration")
public class SwerveCalibration extends LinearOpMode {

    private Hardware hardware;
    private Intake intake;

    private int selectedModule = 0;
    private boolean testing = false;
    private int testAngleIndex = 0;
    private double[] testAngles = {0.0, Math.PI/2, Math.PI, -Math.PI/2};
    private String[] moduleNames = {"Front Left", "Front Right", "Back Left", "Back Right"};

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastXButton = false;
    private boolean lastBButton = false;

    @Override
    public void runOpMode() {
        hardware = new Hardware(hardwareMap);
        intake = new Intake(hardware);

        telemetry.setMsTransmissionInterval(50);

        telemetry.addLine("==================================");
        telemetry.addLine("   SWERVE CALIBRATION TOOL");
        telemetry.addLine("==================================");
        telemetry.addLine();
        telemetry.addLine("1. Manually align ALL wheels forward");
        telemetry.addLine("2. Read the 'Current Angles' below");
        telemetry.addLine("3. Copy values to SteeringConstants.java");
        telemetry.addLine("4. Set as FL/FR/BL/BR_ANGLE_OFFSET");
        telemetry.addLine();
        telemetry.addLine("Press START when ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean currentDpadUp = gamepad1.dpad_up;
            boolean currentDpadDown = gamepad1.dpad_down;

            if (currentDpadUp && !lastDpadUp) {
                selectedModule = (selectedModule + 1) % 4;
            }
            if (currentDpadDown && !lastDpadDown) {
                selectedModule = (selectedModule - 1 + 4) % 4;
            }

            lastDpadUp = currentDpadUp;
            lastDpadDown = currentDpadDown;

            // Axon Mini encoders require servo power - hold position to keep active
            for (int i = 0; i < 4; i++) {
                SwerveModule module = hardware.swerveDrive.getModule(i);
                SwerveModuleState holdState = new SwerveModuleState(
                    0.0,
                    new Rotation2d(module.getCurrentSteeringAngle())
                );
                module.setDesiredState(holdState);
            }

            if (testing) {
                SwerveModule module = hardware.swerveDrive.getModule(selectedModule);
                double targetAngle = testAngles[testAngleIndex];

                SwerveModuleState testState = new SwerveModuleState(
                    0.0,  // Zero velocity
                    new Rotation2d(targetAngle)
                );

                module.setDesiredState(testState);
            }

            telemetry.clear();
            telemetry.addLine("==================================");
            telemetry.addLine("   SWERVE CALIBRATION TOOL");
            telemetry.addLine("==================================");
            telemetry.addLine();

            // Display current angles and voltages
            telemetry.addLine("ANGLE OFFSETS (Copy to Constants):");
            displayModuleAngle(0, "FL");
            displayModuleAngle(1, "FR");
            displayModuleAngle(2, "BL");
            displayModuleAngle(3, "BR");
            telemetry.addLine();

            // Display raw voltages for diagnostics
            telemetry.addLine("Raw Voltages (diagnostic):");
            displayModuleVoltage(0, "FL");
            displayModuleVoltage(1, "FR");
            displayModuleVoltage(2, "BL");
            displayModuleVoltage(3, "BR");
            telemetry.addLine();

            // Encoder status check - warn if any encoders reading 0V
            telemetry.addLine("Encoder Status:");
            int zeroCount = 0;
            for (int i = 0; i < 4; i++) {
                double voltage = hardware.swerveDrive.getModule(i).getRawEncoderVoltage();
                if (voltage < 0.01) {
                    zeroCount++;
                }
            }
            if (zeroCount > 0) {
                telemetry.addLine(String.format("⚠️ WARNING: %d encoder(s) reading 0V!", zeroCount));
                telemetry.addLine("  → Check wiring/connections");
                telemetry.addLine("  → Servos should be powered");
            } else {
                telemetry.addLine("✓ All encoders active (0.3-3.0V)");
            }
            telemetry.addLine();

            // Display module states
            telemetry.addLine("Module States:");
            SwerveModuleState[] states = hardware.swerveDrive.getModuleStates();
            for (int i = 0; i < 4; i++) {
                telemetry.addData(
                    getModuleName(i),
                    "%.2f m/s @ %.0f°",
                    states[i].speedMetersPerSecond,
                    states[i].angle.getDegrees()
                );
            }
            telemetry.addLine();

            // Display selected module and test info
            telemetry.addData("Selected Module", moduleNames[selectedModule]);
            if (testing) {
                telemetry.addData("Testing", "Target: %.0f°", Math.toDegrees(testAngles[testAngleIndex]));
            } else {
                telemetry.addLine("Press X to test selected module");
            }
            telemetry.addLine();

            // Display controls
            telemetry.addLine("Controls:");
            telemetry.addLine("  D-Pad Up/Down: Select module");
            telemetry.addLine("  X: Rotate selected module");
            telemetry.addLine("  B: Stop test");
            telemetry.addLine();

            telemetry.addLine("When all wheels point forward,");
            telemetry.addLine("copy 'CURRENT ANGLES' to code!");

            telemetry.update();
        }

        hardware.swerveDrive.stop();
    }

    /** Values to copy into SteeringConstants.java */
    private void displayModuleAngle(int moduleIndex, String shortName) {
        SwerveModuleState state = hardware.swerveDrive.getModuleStates()[moduleIndex];
        double angleRadians = state.angle.getRadians();
        double angleDegrees = state.angle.getDegrees();

        telemetry.addData(
            String.format("%s_ANGLE_OFFSET", shortName),
            "%.4f  (%.1f°)",
            angleRadians,
            angleDegrees
        );
    }

    private void displayModuleVoltage(int moduleIndex, String shortName) {
        double voltage = hardware.swerveDrive.getModule(moduleIndex).getRawEncoderVoltage();
        telemetry.addData(
            String.format("  %s voltage", shortName),
            "%.3f V",
            voltage
        );
    }

    private String getModuleName(int index) {
        switch (index) {
            case 0: return "FL";
            case 1: return "FR";
            case 2: return "BL";
            case 3: return "BR";
            default: return "??";
        }
    }
}
