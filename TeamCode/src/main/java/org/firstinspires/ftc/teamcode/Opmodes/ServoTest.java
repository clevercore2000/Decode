package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Servo Test", group = "Test")
public class ServoTest extends LinearOpMode {

    private static class ServoConfig {
        String name;
        double minRange;
        double maxRange;

        ServoConfig(String name, double minRange, double maxRange) {
            this.name = name;
            this.minRange = minRange;
            this.maxRange = maxRange;
        }

        double mapRange(double angleDegrees) {
            return ((angleDegrees - minRange) * 1.0) / (maxRange - minRange) + 0.0;
        }

        double getAngleFromPosition(double position) {
            return (position * (maxRange - minRange)) + minRange;
        }

        double getDefaultAngle() {
            return (minRange + maxRange) / 2.0;
        }
    }

    // Add servos here: new ServoConfig("name", minDegrees, maxDegrees)
    private static final ServoConfig[] SERVO_CONFIGS = {
        new ServoConfig("rs", 0, 355)
    };

    private static final double[] INCREMENT_MODES = {1.0, 5.0, 10.0};

    private Servo[] servos;
    private double[] servoAngles;
    private int currentServoIndex = 0;
    private int currentIncrementIndex = 1;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastA = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing servos...");
        telemetry.update();

        initializeServos();

        telemetry.addData("Status", "Ready! %d servo(s) found", servos.length);
        telemetry.addData("Current Servo", SERVO_CONFIGS[currentServoIndex].name);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleInput();
            updateServo();
            displayTelemetry();
        }

        telemetry.addData("Status", "Stopped - servos holding position");
        telemetry.update();
    }

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

    private void handleInput() {
        Servo currentServo = servos[currentServoIndex];
        if (currentServo == null) return;

        ServoConfig config = SERVO_CONFIGS[currentServoIndex];
        double increment = INCREMENT_MODES[currentIncrementIndex];

        if (gamepad1.dpad_up && !lastDpadUp) {
            servoAngles[currentServoIndex] = clampAngle(servoAngles[currentServoIndex] + increment, config);
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            servoAngles[currentServoIndex] = clampAngle(servoAngles[currentServoIndex] - increment, config);
        }

        if (gamepad1.x && !lastX) {
            currentServoIndex = (currentServoIndex + 1) % servos.length;
            int attempts = 0;
            while (servos[currentServoIndex] == null && attempts < servos.length) {
                currentServoIndex = (currentServoIndex + 1) % servos.length;
                attempts++;
            }
        }

        if (gamepad1.y && !lastY) {
            currentIncrementIndex = (currentIncrementIndex + 1) % INCREMENT_MODES.length;
        }

        if (gamepad1.a && !lastA) {
            servoAngles[currentServoIndex] = config.getDefaultAngle();
        }

        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastX = gamepad1.x;
        lastY = gamepad1.y;
        lastA = gamepad1.a;
    }

    private void updateServo() {
        Servo currentServo = servos[currentServoIndex];
        if (currentServo != null) {
            ServoConfig config = SERVO_CONFIGS[currentServoIndex];
            double servoPosition = config.mapRange(servoAngles[currentServoIndex]);
            currentServo.setPosition(servoPosition);
        }
    }

    private double clampAngle(double angle, ServoConfig config) {
        return Math.max(config.minRange, Math.min(config.maxRange, angle));
    }

    private void displayTelemetry() {
        ServoConfig currentConfig = SERVO_CONFIGS[currentServoIndex];

        telemetry.addData("=== SERVO TEST ===", "");
        telemetry.addData("", "");

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
