package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Constants.SteeringConstants;

@TeleOp(name = "Swerve Encoder Reader", group = "Test")
public class SwerveEncoderReader extends LinearOpMode {

    private AnalogInput flEncoder;
    private AnalogInput frEncoder;
    private AnalogInput blEncoder;
    private AnalogInput brEncoder;

    private CRServo flSteer;
    private CRServo frSteer;
    private CRServo blSteer;
    private CRServo brSteer;

    @Override
    public void runOpMode() {
        // --- HARDWARE CONFIGURATION ---
        try {
            flEncoder = hardwareMap.get(AnalogInput.class, "fl_enc");
            frEncoder = hardwareMap.get(AnalogInput.class, "fr_enc");
            blEncoder = hardwareMap.get(AnalogInput.class, "bl_enc");
            brEncoder = hardwareMap.get(AnalogInput.class, "br_enc");

            flSteer = hardwareMap.get(CRServo.class, "fl_servo");
            frSteer = hardwareMap.get(CRServo.class, "fr_servo");
            blSteer = hardwareMap.get(CRServo.class, "bl_servo");
            brSteer = hardwareMap.get(CRServo.class, "br_servo");

        } catch (Exception e) {
            telemetry.addData("Error", "Could not find all devices. Please check your configuration.");
            telemetry.addLine(e.getMessage());
            telemetry.update();
            sleep(5000);
            requestOpModeStop();
            return;
        }

        telemetry.addLine("Swerve Encoder Reader");
        telemetry.addLine("Ready to start.");
        telemetry.addLine("This OpMode now uses voltage offsets.");
        telemetry.addLine("If calibrated correctly, all wheel angles should be near 0 when pointing straight forward.");
        telemetry.update();

        waitForStart();

        // Keep the servos powered
        flSteer.setPower(0.01);
        frSteer.setPower(0.01);
        blSteer.setPower(0.01);
        brSteer.setPower(0.01);

        while (opModeIsActive()) {

            // Display the results in the telemetry
            telemetry.addLine("--- Front Left ---");
            displayModuleData(flEncoder, SteeringConstants.FL_VOLTAGE_OFFSET);
            telemetry.addLine("--- Front Right ---");
            displayModuleData(frEncoder, SteeringConstants.FR_VOLTAGE_OFFSET);
            telemetry.addLine("--- Back Left ---");
            displayModuleData(blEncoder, SteeringConstants.BL_VOLTAGE_OFFSET);
            telemetry.addLine("--- Back Right ---");
            displayModuleData(brEncoder, SteeringConstants.BR_VOLTAGE_OFFSET);
            telemetry.update();
        }

        // Stop the servos when the OpMode is stopped
        flSteer.setPower(0);
        frSteer.setPower(0);
        blSteer.setPower(0);
        brSteer.setPower(0);
    }

    private void displayModuleData(AnalogInput encoder, double voltageOffset) {
        // Read the raw voltage
        double currentVoltage = encoder.getVoltage();

        // Subtract the offset to get the adjusted voltage
        double adjustedVoltage = currentVoltage - voltageOffset;

        // Wrap the voltage if it's negative
        if (adjustedVoltage < 0) {
            adjustedVoltage += SteeringConstants.MAX_VOLTAGE;
        }

        // Convert the adjusted voltage to a servo angle (0-360 degrees)
        double servoAngle = (adjustedVoltage / SteeringConstants.MAX_VOLTAGE) * 720.0;

        // Convert the servo angle to the wheel angle
        double wheelAngle = servoAngle / SteeringConstants.SERVO_TO_WHEEL_RATIO;

        telemetry.addData("Raw Voltage", "%.4f V", currentVoltage);
        telemetry.addData("Adjusted Voltage", "%.4f V", adjustedVoltage);
        telemetry.addData("Wheel Angle", "%.1f degrees", wheelAngle);
    }
}