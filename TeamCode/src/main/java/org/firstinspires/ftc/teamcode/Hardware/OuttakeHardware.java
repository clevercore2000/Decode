package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OuttakeHardware {
    private static final String turretMotorName = "w1";

    public final DcMotorEx TurretMotor;

    public OuttakeHardware(HardwareMap hardwareMap) {
        TurretMotor = hardwareMap.get(DcMotorEx.class, turretMotorName);
        TurretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TurretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TurretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
