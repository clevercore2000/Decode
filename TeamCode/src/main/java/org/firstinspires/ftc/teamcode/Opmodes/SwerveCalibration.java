package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Swerve Calibration", group = "Calibration")
public class SwerveCalibration extends LinearOpMode {

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

        telemetry.addLine("Swerve Calibration");
        telemetry.addLine("1. Manually align all wheels to point straight forward.");
        telemetry.addLine("2. Press the 'A' button to capture the current voltages.");
        telemetry.addLine("3. Copy the captured voltages into SteeringConstants.java.");
        telemetry.update();

        waitForStart();

        // Keep the servos powered
        flSteer.setPower(0.01);
        frSteer.setPower(0.01);
        blSteer.setPower(0.01);
        brSteer.setPower(0.01);

        while (opModeIsActive()) {

            // Read the raw voltage from the encoders
            double flVoltage = flEncoder.getVoltage();
            double frVoltage = frEncoder.getVoltage();
            double blVoltage = blEncoder.getVoltage();
            double brVoltage = brEncoder.getVoltage();

            // Display the current voltages
            telemetry.addLine("--- Current Voltages ---");
            telemetry.addData("Front Left", "%.4f V", flVoltage);
            telemetry.addData("Front Right", "%.4f V", frVoltage);
            telemetry.addData("Back Left", "%.4f V", blVoltage);
            telemetry.addData("Back Right", "%.4f V", brVoltage);
            telemetry.addLine();
            telemetry.addLine("Press 'A' to capture these voltages as offsets.");

            if (gamepad1.a) {
                telemetry.addLine("--- Captured Voltage Offsets (Copy to SteeringConstants.java) ---");
                telemetry.addData("public static final double FL_VOLTAGE_OFFSET = ", "%.4f;", flVoltage);
                telemetry.addData("public static final double FR_VOLTAGE_OFFSET = ", "%.4f;", frVoltage);
                telemetry.addData("public static final double BL_VOLTAGE_OFFSET = ", "%.4f;", blVoltage);
                telemetry.addData("public static final double BR_VOLTAGE_OFFSET = ", "%.4f;", brVoltage);
            }

            telemetry.update();
        }

        // Stop the servos when the OpMode is stopped
        flSteer.setPower(0);
        frSteer.setPower(0);
        blSteer.setPower(0);
        brSteer.setPower(0);
    }
}
