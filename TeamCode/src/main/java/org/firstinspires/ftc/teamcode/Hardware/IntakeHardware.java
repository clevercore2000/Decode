package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Hardware configuration for Intake subsystem
 * Simple single-motor intake
 */
public class IntakeHardware {

    private static final String intakeMotorName = "I1";

    public final DcMotor IntakeMotor;

    public IntakeHardware(HardwareMap hardwareMap) {
        IntakeMotor = hardwareMap.get(DcMotor.class, intakeMotorName);
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
