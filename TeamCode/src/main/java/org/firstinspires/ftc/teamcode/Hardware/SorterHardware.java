package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SorterHardware {
    private static final String sorterMotorName = "SM1";
    private static final String kickServoName = "SS1";
    private static final String colorSensorName = "CS1";

    public final DcMotorEx  SorterMotor;
    public final Servo KickServo;
    public final ColorSensor ColorSensor;

    public SorterHardware(HardwareMap hardwareMap) {
        SorterMotor = hardwareMap.get(DcMotorEx.class, sorterMotorName);
        SorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SorterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SorterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        KickServo = hardwareMap.get(Servo.class, kickServoName);

        ColorSensor = hardwareMap.get(ColorSensor.class, colorSensorName);
    }
}
