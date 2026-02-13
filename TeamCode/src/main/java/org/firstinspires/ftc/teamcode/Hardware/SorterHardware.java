package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.SorterConstants;

public class SorterHardware {
    private static final String sorterMotorName = "SM1";
    private static final String kickServoName = "SS1";
    private static final String colorSensorName = "CS1";

    public final DcMotorEx SorterMotor;
    public final ServoCfg KickServo;
    public final ColorSensor ColorSensor;
    public final DistanceSensor DistanceSensor;

    public SorterHardware(HardwareMap hardwareMap) {
        SorterMotor = hardwareMap.get(DcMotorEx.class, sorterMotorName);
        SorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SorterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SorterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Servo kickRaw = hardwareMap.get(Servo.class, kickServoName);
        KickServo = new ServoCfg(kickRaw, SorterConstants.KICK_SERVO_SPEED);
        KickServo.setRange(0, 1);

        ColorSensor = hardwareMap.get(ColorSensor.class, colorSensorName);
        DistanceSensor = hardwareMap.get(DistanceSensor.class, colorSensorName);
    }
}
