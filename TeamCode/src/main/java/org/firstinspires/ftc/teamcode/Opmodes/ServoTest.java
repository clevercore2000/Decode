package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Servo testing teleop with degree-based incremental control
 * Uses the same degree mapping system as ServoCfg
 *
 * Features:
 * - Array-based servo configuration (easy to add/remove servos)
 * - Degree-based control with configurable ranges per servo
 * - Cycle between multiple servos
 * - Cycle between increment sizes (1°, 5°, 10°)
 * - Safe position clamping to servo-specific ranges
 * - Axon Mini support (0-355° full range)
 *
 * Controls:
 * - Dpad Up: Increase angle
 * - Dpad Down: Decrease angle
 * - X: Cycle to next servo
 * - Y: Cycle increment size
 * - A: Reset to center angle
 */
@TeleOp(name = "Servo Test", group = "Test")
public class ServoTest extends LinearOpMode {

    // ==================== SERVO CONFIGURATION ====================
    /**
     * ServoConfig - stores range configuration for each servo
     * Like ServoCfg, maps degrees to servo position (0.0-1.0)
     */
    private static class ServoConfig {
        String name;
        double minRange;  // Minimum angle in degrees
        double maxRange;  // Maximum angle in degrees

        ServoConfig(String name, double minRange, double maxRange) {
            this.name = name;
            this.minRange = minRange;
            this.maxRange = maxRange;
        }

        /**
         * Maps angle (in degrees) to servo position (0.0-1.0)
         * Same formula as ServoCfg.mapRange()
         */
        double mapRange(double angleDegrees) {
            return ((angleDegrees - minRange) * 1.0) / (maxRange - minRange) + 0.0;
        }

        /**
         * Maps servo position (0.0-1.0) back to angle (degrees)
         */
        double getAngleFromPosition(double position) {
            return (position * (maxRange - minRange)) + minRange;
        }

        double getDefaultAngle() {
            return (minRange + maxRange) / 2.0;
        }
    }

    // Configure your servos here - add/remove as needed
    private static final ServoConfig[] SERVO_CONFIGS = {
        new ServoConfig("rs", 0, 355)  // Axon Mini - full range (0-355°)
        // Add more servos: new ServoConfig("claw", 0, 180)
        // Standard servos: typically 0-180° or 0-270°
        // Axon Mini servos: 0-355° (5° dead zone at 355-360)
    };

    private static final double[] INCREMENT_MODES = {1.0, 5.0, 10.0};  // Degrees

    // ==================== STATE VARIABLES ====================
    private Servo[] servos;
    private double[] servoAngles;  // Current angle in degrees
    private int currentServoIndex = 0;
    private int currentIncrementIndex = 1;  // Start with 5°

    // Debouncing
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastA = false;

    @Override
    public void runOpMode() {
        // Initialize hardware
        telemetry.addData("Status", "Initializing servos...");
        telemetry.update();

        initializeServos();

        telemetry.addData("Status", "Ready! %d servo(s) found", servos.length);
        telemetry.addData("Current Servo", SERVO_CONFIGS[currentServoIndex].name);
        telemetry.update();

        waitForStart();

        // Main control loop
        while (opModeIsActive()) {
            handleInput();
            updateServo();
            displayTelemetry();
        }

        // Cleanup - hold last positions
        telemetry.addData("Status", "Stopped - servos holding position");
        telemetry.update();
    }

    /**
     * Initialize servo array from configuration
     */
    private void initializeServos() {
        servos = new Servo[SERVO_CONFIGS.length];
        servoAngles = new double[SERVO_CONFIGS.length];

        for (int i = 0; i < SERVO_CONFIGS.length; i++) {
            try {
                servos[i] = hardwareMap.get(Servo.class, SERVO_CONFIGS[i].name);
                servoAngles[i] = SERVO_CONFIGS[i].getDefaultAngle();
                servos[i].setPosition(SERVO_CONFIGS[i].mapRange(servoAngles[i]));
                telemetry.addData("✓ Found", "%s (%.0f-%.0f°)",
                    SERVO_CONFIGS[i].name,
                    SERVO_CONFIGS[i].minRange,
                    SERVO_CONFIGS[i].maxRange);
            } catch (Exception e) {
                telemetry.addData("✗ Missing", SERVO_CONFIGS[i].name + " - " + e.getMessage());
                servos[i] = null;
            }
        }
    }

    /**
     * Handle gamepad input with debouncing
     */
    private void handleInput() {
        // Get current servo
        Servo currentServo = servos[currentServoIndex];
        if (currentServo == null) return;

        ServoConfig config = SERVO_CONFIGS[currentServoIndex];
        double increment = INCREMENT_MODES[currentIncrementIndex];

        // Angle control - Dpad Up/Down
        if (gamepad1.dpad_up && !lastDpadUp) {
            servoAngles[currentServoIndex] = clampAngle(
                servoAngles[currentServoIndex] + increment,
                config
            );
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            servoAngles[currentServoIndex] = clampAngle(
                servoAngles[currentServoIndex] - increment,
                config
            );
        }

        // Cycle servo - X button
        if (gamepad1.x && !lastX) {
            currentServoIndex = (currentServoIndex + 1) % servos.length;
            // Skip null servos
            int attempts = 0;
            while (servos[currentServoIndex] == null && attempts < servos.length) {
                currentServoIndex = (currentServoIndex + 1) % servos.length;
                attempts++;
            }
        }

        // Cycle increment mode - Y button
        if (gamepad1.y && !lastY) {
            currentIncrementIndex = (currentIncrementIndex + 1) % INCREMENT_MODES.length;
        }

        // Reset to center - A button
        if (gamepad1.a && !lastA) {
            servoAngles[currentServoIndex] = config.getDefaultAngle();
        }

        // Update debounce state
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastX = gamepad1.x;
        lastY = gamepad1.y;
        lastA = gamepad1.a;
    }

    /**
     * Update current servo position
     * Converts angle (degrees) to servo position (0.0-1.0) using mapRange
     */
    private void updateServo() {
        Servo currentServo = servos[currentServoIndex];
        if (currentServo != null) {
            ServoConfig config = SERVO_CONFIGS[currentServoIndex];
            double servoPosition = config.mapRange(servoAngles[currentServoIndex]);
            currentServo.setPosition(servoPosition);
        }
    }

    /**
     * Clamp angle to servo-specific valid range
     */
    private double clampAngle(double angle, ServoConfig config) {
        return Math.max(config.minRange, Math.min(config.maxRange, angle));
    }

    /**
     * Display comprehensive telemetry
     */
    private void displayTelemetry() {
        ServoConfig currentConfig = SERVO_CONFIGS[currentServoIndex];

        telemetry.addData("=== SERVO TEST ===", "");
        telemetry.addData("", "");

        // Current servo info
        if (servos[currentServoIndex] != null) {
            telemetry.addData("Current Servo", "[%d/%d] %s",
                currentServoIndex + 1,
                servos.length,
                currentConfig.name);
            telemetry.addData("Angle", "%.1f°", servoAngles[currentServoIndex]);
            telemetry.addData("Range", "%.0f° - %.0f°",
                currentConfig.minRange,
                currentConfig.maxRange);
            telemetry.addData("Servo Position", "%.3f",
                currentConfig.mapRange(servoAngles[currentServoIndex]));
        } else {
            telemetry.addData("Current Servo", "[%d/%d] %s (NOT FOUND)",
                currentServoIndex + 1,
                servos.length,
                currentConfig.name);
        }

        telemetry.addData("Increment", "%.0f°", INCREMENT_MODES[currentIncrementIndex]);
        telemetry.addData("", "");

        // All servo positions
        telemetry.addData("=== ALL SERVOS ===", "");
        for (int i = 0; i < servos.length; i++) {
            String indicator = (i == currentServoIndex) ? "→ " : "  ";
            String status;
            if (servos[i] != null) {
                status = String.format("%.1f° (%.0f-%.0f°)",
                    servoAngles[i],
                    SERVO_CONFIGS[i].minRange,
                    SERVO_CONFIGS[i].maxRange);
            } else {
                status = "NOT FOUND";
            }
            telemetry.addData(indicator + SERVO_CONFIGS[i].name, status);
        }
        telemetry.addData("", "");

        // Controls
        telemetry.addData("=== CONTROLS ===", "");
        telemetry.addData("Dpad Up/Down", "Adjust angle");
        telemetry.addData("X", "Next servo");
        telemetry.addData("Y", "Change increment (1°/5°/10°)");
        telemetry.addData("A", "Reset to center");

        telemetry.update();
    }
}
