package org.firstinspires.ftc.teamcode.Opmodes;

import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;

/**
 * Swerve Module Test OpMode
 *
 * Test individual modules and coordinated swerve drive movements
 *
 * Test Modes:
 * 1. Individual Module Test - Test one module at a time
 * 2. All Modules Test - Command all modules to same state
 * 3. Kinematics Test - Test coordinated movements (forward, strafe, rotate)
 *
 * Controls:
 * Mode 1 (Individual):
 * - D-Pad Up/Down: Select module (FL/FR/BL/BR)
 * - Left Stick: Control selected module (X=angle, Y=speed)
 *
 * Mode 2 (All Modules):
 * - Left Stick: Control all modules (X=angle, Y=speed)
 *
 * Mode 3 (Kinematics):
 * - D-Pad Up: Forward
 * - D-Pad Down: Backward
 * - D-Pad Left: Strafe Left
 * - D-Pad Right: Strafe Right
 * - Left Trigger: Rotate CCW
 * - Right Trigger: Rotate CW
 *
 * Switch Modes:
 * - X Button: Individual Module Test
 * - Y Button: All Modules Test
 * - B Button: Kinematics Test
 * - A Button: Stop all modules
 */
// Enable when needed for testing
@TeleOp(name = "Swerve Module Test", group = "Testing")
public class SwerveModuleTest extends LinearOpMode {

    private Hardware hardware;

    private enum TestMode {
        INDIVIDUAL,
        ALL_MODULES,
        KINEMATICS
    }

    private TestMode currentMode = TestMode.INDIVIDUAL;
    private int selectedModule = 0;  // 0=FL, 1=FR, 2=BL, 3=BR
    private String[] moduleNames = {"Front Left", "Front Right", "Back Left", "Back Right"};

    // Button state tracking
    private boolean lastXButton = false;
    private boolean lastYButton = false;
    private boolean lastBButton = false;
    private boolean lastAButton = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    @Override
    public void runOpMode() {
        hardware = new Hardware(hardwareMap);

        telemetry.addLine("Swerve Module Test");
        telemetry.addLine("Press X/Y/B to select mode");
        telemetry.addLine("Press A to stop");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleModeSelection();
            handleModuleSelection();

            switch (currentMode) {
                case INDIVIDUAL:
                    runIndividualModuleTest();
                    break;
                case ALL_MODULES:
                    runAllModulesTest();
                    break;
                case KINEMATICS:
                    runKinematicsTest();
                    break;
            }

            updateTelemetry();
        }

        hardware.swerveDrive.stop();
    }

    /**
     * Handle mode selection buttons
     */
    private void handleModeSelection() {
        boolean currentXButton = gamepad1.x;
        boolean currentYButton = gamepad1.y;
        boolean currentBButton = gamepad1.b;
        boolean currentAButton = gamepad1.a;

        if (currentXButton && !lastXButton) {
            currentMode = TestMode.INDIVIDUAL;
        }
        if (currentYButton && !lastYButton) {
            currentMode = TestMode.ALL_MODULES;
        }
        if (currentBButton && !lastBButton) {
            currentMode = TestMode.KINEMATICS;
        }
        if (currentAButton && !lastAButton) {
            hardware.swerveDrive.stop();
        }

        lastXButton = currentXButton;
        lastYButton = currentYButton;
        lastBButton = currentBButton;
        lastAButton = currentAButton;
    }

    /**
     * Handle module selection for individual mode
     */
    private void handleModuleSelection() {
        if (currentMode != TestMode.INDIVIDUAL) {
            return;
        }

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
    }

    /**
     * Test individual module control
     */
    private void runIndividualModuleTest() {
        // Get joystick input
        double stickX = gamepad1.left_stick_x;
        double stickY = -gamepad1.left_stick_y;

        // Convert to angle and speed
        double speed = Math.hypot(stickX, stickY) * 2.0;  // Scale to ~2 m/s max
        double angle = Math.atan2(stickY, stickX);

        // Create state
        SwerveModuleState state = new SwerveModuleState(
            speed,
            new Rotation2d(angle)
        );

        // Command selected module, stop others
        for (int i = 0; i < 4; i++) {
            if (i == selectedModule) {
                hardware.swerveDrive.getModule(i).setDesiredState(state);
            } else {
                hardware.swerveDrive.getModule(i).stop();
            }
        }
    }

    /**
     * Test all modules with same command
     */
    private void runAllModulesTest() {
        // Get joystick input
        double stickX = gamepad1.left_stick_x;
        double stickY = -gamepad1.left_stick_y;

        // Convert to angle and speed
        double speed = Math.hypot(stickX, stickY) * 2.0;
        double angle = Math.atan2(stickY, stickX);

        // Create state
        SwerveModuleState state = new SwerveModuleState(
            speed,
            new Rotation2d(angle)
        );

        // Command all modules to same state
        SwerveModuleState[] states = {state, state, state, state};
        hardware.swerveDrive.setModuleStates(states);
    }

    /**
     * Test coordinated kinematics movements
     */
    private void runKinematicsTest() {
        double forward = 0.0;
        double strafe = 0.0;
        double rotation = 0.0;

        // Test speed (m/s and rad/s)
        final double TEST_SPEED = 1.0;
        final double TEST_ROTATION = Math.PI / 4;  // 45 degrees/sec

        // D-Pad for translation
        if (gamepad1.dpad_up) forward = TEST_SPEED;
        if (gamepad1.dpad_down) forward = -TEST_SPEED;
        if (gamepad1.dpad_left) strafe = TEST_SPEED;
        if (gamepad1.dpad_right) strafe = -TEST_SPEED;

        // Triggers for rotation
        if (gamepad1.left_trigger > 0.1) rotation = TEST_ROTATION * gamepad1.left_trigger;
        if (gamepad1.right_trigger > 0.1) rotation = -TEST_ROTATION * gamepad1.right_trigger;

        // Drive (robot-centric)
        hardware.swerveDrive.drive(forward, strafe, rotation, false);
    }

    /**
     * Update telemetry display
     */
    private void updateTelemetry() {
        telemetry.clear();
        telemetry.addLine("===== SWERVE MODULE TEST =====");
        telemetry.addLine();

        // Display mode
        telemetry.addData("Mode", getModeString());
        if (currentMode == TestMode.INDIVIDUAL) {
            telemetry.addData("Selected Module", moduleNames[selectedModule]);
        }
        telemetry.addLine();

        // Display module states
        telemetry.addLine("Module States:");
        SwerveModuleState[] states = hardware.swerveDrive.getModuleStates();
        for (int i = 0; i < 4; i++) {
            String marker = (currentMode == TestMode.INDIVIDUAL && i == selectedModule) ? " <--" : "";
            telemetry.addData(
                String.format("%s%s", getModuleName(i), marker),
                "%.2f m/s @ %.0f°",
                states[i].speedMetersPerSecond,
                states[i].angle.getDegrees()
            );
        }
        telemetry.addLine();

        // Display controls based on mode
        telemetry.addLine("Controls:");
        switch (currentMode) {
            case INDIVIDUAL:
                telemetry.addLine("  D-Pad Up/Down: Select module");
                telemetry.addLine("  Left Stick: Control module");
                break;
            case ALL_MODULES:
                telemetry.addLine("  Left Stick: Control all modules");
                break;
            case KINEMATICS:
                telemetry.addLine("  D-Pad: Translate");
                telemetry.addLine("  Triggers: Rotate");
                break;
        }
        telemetry.addLine("  X: Individual | Y: All | B: Kinematics");
        telemetry.addLine("  A: Stop");

        telemetry.update();
    }

    private String getModeString() {
        switch (currentMode) {
            case INDIVIDUAL: return "Individual Module";
            case ALL_MODULES: return "All Modules";
            case KINEMATICS: return "Kinematics Test";
            default: return "Unknown";
        }
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
