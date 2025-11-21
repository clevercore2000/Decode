package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

/**
 * Simple Intake Subsystem
 * Single motor with direct power control
 */
public class Intake {

    private Hardware hardware;

    public Intake(Hardware hardware) {
        this.hardware = hardware;
    }

    public void Start(double power) {
        hardware.intakeHardware.IntakeMotor.setPower(power);
    }

    public void Stop() {
        hardware.intakeHardware.IntakeMotor.setPower(0);
    }
}
