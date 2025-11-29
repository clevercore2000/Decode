package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Hardware configuration for Outtake subsystem (2-motor wheel mechanism)
 * Uses DcMotorEx for velocity control with encoder feedback
 */
public class OuttakeHardware {

    private static final String Wheel_1 = "w1";
    private static final String Wheel_2 = "w2";

    private static final String Ramp_servo = "rs";

    public final DcMotorEx WheelMotor1;
    public final DcMotorEx WheelMotor2;
    public Servo RampServo;

    public OuttakeHardware(HardwareMap hardwareMap) {
        WheelMotor1 = hardwareMap.get(DcMotorEx.class, Wheel_1);
        WheelMotor2 = hardwareMap.get(DcMotorEx.class, Wheel_2);

        RampServo = hardwareMap.get(Servo.class, Ramp_servo);

        // Configure motor behavior
        WheelMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WheelMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set motor direction (one motor reversed to spin same direction)
        WheelMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset encoders and enable encoder reading
        // RUN_USING_ENCODER allows us to:
        // 1. Read velocity with getVelocity() for our custom PID
        // 2. Set power directly (not use built-in velocity PID)
        WheelMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WheelMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        WheelMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        WheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
