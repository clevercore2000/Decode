package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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

        WheelMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WheelMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // NEVER call setDirection() on these ports. w1 and w2 carry the BR and FR swerve
        // steering encoders, and DcMotorImpl.getCurrentPosition() runs the reading through
        // adjustPosition(), which negates it when the direction is REVERSE. Reversing the
        // motor here silently flips a steering encoder, and only in opmodes that happen to
        // construct this class — which is how FR ended up oscillating in TranslationTest
        // while behaving in teleop. The flywheels are a mirrored pair, so the sign lives in
        // setWheelPower() instead, where it affects nothing but motor power.
        WheelMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        WheelMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        // RUN_WITHOUT_ENCODER: encoder pins on these ports are used by swerve steering encoders,
        // so we cannot use encoder-based velocity control for the outtake motors.
        WheelMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Drives both flywheels at {@code power}. They face each other, so motor 2 is negated here
     * rather than via {@code setDirection} — see the constructor for why that matters.
     */
    public void setWheelPower(double power) {
        WheelMotor1.setPower(power);
        WheelMotor2.setPower(-power);
    }
}
