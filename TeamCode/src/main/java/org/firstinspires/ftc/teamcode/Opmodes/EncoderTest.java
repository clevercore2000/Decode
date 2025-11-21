package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Constants.SteeringConstants;

/**
 * Encoder Test OpMode
 *
 * Tests analog encoder readings from Axon Mini servos
 * Displays raw voltage and calculated angles
 *
 * Purpose:
 * - Verify encoders are outputting voltage
 * - Check if voltage changes when servos rotate
 * - Diagnose wiring/configuration issues
 *
 * Controls:
 * - D-Pad Up/Down: Select module to test
 * - Left Bumper: Rotate selected servo CCW (slowly)
 * - Right Bumper: Rotate selected servo CW (slowly)
 * - A Button: Stop all servos
 */
@Disabled
@TeleOp(name = "Encoder Test", group = "Testing")
public class EncoderTest extends LinearOpMode {

    // Analog encoders
    private AnalogInput flEncoder;
    private AnalogInput frEncoder;
    private AnalogInput blEncoder;
    private AnalogInput brEncoder;

    // Steering servos
    private CRServo flSteer;
    private CRServo frSteer;
    private CRServo blSteer;
    private CRServo brSteer;

    // Module selection
    private int selectedModule = 0;  // 0=FL, 1=FR, 2=BL, 3=BR
    private String[] moduleNames = {"Front Left (FL)", "Front Right (FR)", "Back Left (BL)", "Back Right (BR)"};

    // Button tracking
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    @Override
    public void runOpMode() {
        // Initialize encoders
        flEncoder = hardwareMap.get(AnalogInput.class, "fl_enc");
        frEncoder = hardwareMap.get(AnalogInput.class, "fr_enc");
        blEncoder = hardwareMap.get(AnalogInput.class, "bl_enc");
        brEncoder = hardwareMap.get(AnalogInput.class, "br_enc");

        // Initialize servos
        flSteer = hardwareMap.get(CRServo.class, "fl_servo");
        frSteer = hardwareMap.get(CRServo.class, "fr_servo");
        blSteer = hardwareMap.get(CRServo.class, "bl_servo");
        brSteer = hardwareMap.get(CRServo.class, "br_servo");

        telemetry.addLine("=============================");
        telemetry.addLine("    ENCODER TEST MODE");
        telemetry.addLine("=============================");
        telemetry.addLine();
        telemetry.addLine("This will show raw voltage");
        telemetry.addLine("and angles from each encoder");
        telemetry.addLine();
        telemetry.addLine("If all voltages are 0.000V,");
        telemetry.addLine("check your wiring/config!");
        telemetry.addLine();
        telemetry.addLine("Press START when ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Handle module selection
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

            // Control selected servo
            if (gamepad1.left_bumper) {
                // Rotate CCW slowly
                setServo(selectedModule, 0.2);
            } else if (gamepad1.right_bumper) {
                // Rotate CW slowly
                setServo(selectedModule, -0.2);
            } else if (gamepad1.a) {
                // Stop all
                stopAllServos();
            } else {
                // Stop selected servo when no input
                setServo(selectedModule, 0.0);
            }

            // Update telemetry
            updateTelemetry();
        }

        // Stop servos when exiting
        stopAllServos();
    }

    /**
     * Update telemetry with encoder readings
     */
    private void updateTelemetry() {
        telemetry.clear();
        telemetry.addLine("=============================");
        telemetry.addLine("   ENCODER TEST MODE");
        telemetry.addLine("=============================");
        telemetry.addLine();

        // Display each encoder: voltage and angle side-by-side
        telemetry.addLine("ENCODER READINGS:");
        displayEncoderData("FL", flEncoder, SteeringConstants.FL_ANGLE_OFFSET);
        displayEncoderData("FR", frEncoder, SteeringConstants.FR_ANGLE_OFFSET);
        displayEncoderData("BL", blEncoder, SteeringConstants.BL_ANGLE_OFFSET);
        displayEncoderData("BR", brEncoder, SteeringConstants.BR_ANGLE_OFFSET);
        telemetry.addLine();

        // Display selected module
        telemetry.addData("Selected", moduleNames[selectedModule]);
        telemetry.addLine();

        // Display controls
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("  D-Pad Up/Down: Select module");
        telemetry.addLine("  L/R Bumper: Rotate slow");
        telemetry.addLine("  A: Stop all");
        telemetry.addLine();

        // Diagnostic hints
        int zeroCount = countZeroVoltages();
        if (zeroCount == 4) {
            telemetry.addLine("⚠ All voltages 0V - check wiring!");
        } else if (zeroCount > 0) {
            telemetry.addLine("⚠ " + zeroCount + " encoder(s) reading 0V");
        } else {
            telemetry.addLine("✓ All encoders reading voltage");
        }

        telemetry.update();
    }

    /**
     * Display voltage and angle for one encoder
     */
    private void displayEncoderData(String name, AnalogInput encoder, double offset) {
        double voltage = encoder.getVoltage();

        // Convert voltage to wheel angle (encoder on servo shaft with 2:1 reduction)
        double rawAngle = (voltage / SteeringConstants.ANALOG_VOLTAGE_MAX) * 2 * Math.PI * SteeringConstants.SERVO_TO_STEERING_RATIO;
        double angle = rawAngle - offset;
        angle = normalizeAngle(angle);

        // Display both voltage and angle
        telemetry.addData(
            String.format("  %s", name),
            "%.3fV → %.1f°",
            voltage,
            Math.toDegrees(angle)
        );
    }

    /**
     * Normalize angle to [-π, π]
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    /**
     * Count how many encoders are reading near 0V
     */
    private int countZeroVoltages() {
        int count = 0;
        if (flEncoder.getVoltage() < 0.01) count++;
        if (frEncoder.getVoltage() < 0.01) count++;
        if (blEncoder.getVoltage() < 0.01) count++;
        if (brEncoder.getVoltage() < 0.01) count++;
        return count;
    }

    /**
     * Set power to specific servo
     */
    private void setServo(int moduleIndex, double power) {
        switch (moduleIndex) {
            case 0: flSteer.setPower(power); break;
            case 1: frSteer.setPower(power); break;
            case 2: blSteer.setPower(power); break;
            case 3: brSteer.setPower(power); break;
        }
    }

    /**
     * Stop all servos
     */
    private void stopAllServos() {
        flSteer.setPower(0.0);
        frSteer.setPower(0.0);
        blSteer.setPower(0.0);
        brSteer.setPower(0.0);
    }
}
