package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OuttakeHardware {
    private static final String turretMotorName = "w1";
    private static final String turretMotor2Name = "w2";

    public final DcMotorEx TurretMotor;
    public final DcMotorEx TurretMotor2;

    public OuttakeHardware(HardwareMap hardwareMap) {
        TurretMotor = hardwareMap.get(DcMotorEx.class, turretMotorName);
        TurretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TurretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TurretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        TurretMotor2 = hardwareMap.get(DcMotorEx.class, turretMotor2Name);
        TurretMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TurretMotor2.setDirection(DcMotor.Direction.REVERSE);
        TurretMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TurretMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
