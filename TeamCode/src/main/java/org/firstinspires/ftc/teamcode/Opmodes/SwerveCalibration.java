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
            telemetry.addData("Error", "Could not find all devices");
            telemetry.addLine(e.getMessage());
            telemetry.update();
            sleep(5000);
            requestOpModeStop();
            return;
        }

        telemetry.addLine("Swerve Calibration");
        telemetry.addLine("1. Align all wheels straight forward");
        telemetry.addLine("2. Press A to capture voltages");
        telemetry.addLine("3. Copy to SteeringConstants.java");
        telemetry.update();

        waitForStart();

        flSteer.setPower(0.01);
        frSteer.setPower(0.01);
        blSteer.setPower(0.01);
        brSteer.setPower(0.01);

        while (opModeIsActive()) {
            double flVoltage = flEncoder.getVoltage();
            double frVoltage = frEncoder.getVoltage();
            double blVoltage = blEncoder.getVoltage();
            double brVoltage = brEncoder.getVoltage();

            telemetry.addLine("--- Current Voltages ---");
            telemetry.addData("FL", "%.4f V", flVoltage);
            telemetry.addData("FR", "%.4f V", frVoltage);
            telemetry.addData("BL", "%.4f V", blVoltage);
            telemetry.addData("BR", "%.4f V", brVoltage);
            telemetry.addLine();
            telemetry.addLine("Press A to capture");

            if (gamepad1.a) {
                telemetry.addLine("--- Copy to SteeringConstants.java ---");
                telemetry.addData("FL_VOLTAGE_OFFSET", "%.4f", flVoltage);
                telemetry.addData("FR_VOLTAGE_OFFSET", "%.4f", frVoltage);
                telemetry.addData("BL_VOLTAGE_OFFSET", "%.4f", blVoltage);
                telemetry.addData("BR_VOLTAGE_OFFSET", "%.4f", brVoltage);
            }

            telemetry.update();
        }

        flSteer.setPower(0);
        frSteer.setPower(0);
        blSteer.setPower(0);
        brSteer.setPower(0);
    }
}
