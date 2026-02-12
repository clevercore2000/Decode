package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

public class Outtake {

    private final DcMotorEx turretMotor;
    private final PIDController pidController;

    private double targetRPM = 0;
    private double currentRPM = 0;
    private double motorPower = 0;
    private boolean isActive = false;

    public Outtake(Hardware hardware) {
        this.turretMotor = hardware.outtakeHardware.TurretMotor;
        pidController = new PIDController(
                OuttakeConstants.VELOCITY_P,
                OuttakeConstants.VELOCITY_I,
                OuttakeConstants.VELOCITY_D
        );
    }

    public void setTargetRPM(double rpm) {
        targetRPM = Math.max(0, Math.min(rpm, OuttakeConstants.MAX_MOTOR_RPM));
        isActive = targetRPM > 0;
    }

    public void update() {
        if (!isActive) {
            turretMotor.setPower(0);
            motorPower = 0;
            currentRPM = 0;
            return;
        }

        double targetTPS = (targetRPM / 60.0) * OuttakeConstants.MOTOR_TICKS_PER_REV;
        double feedforward = targetTPS * OuttakeConstants.VELOCITY_FF;

        double velocityTPS = turretMotor.getVelocity();
        currentRPM = (velocityTPS / OuttakeConstants.MOTOR_TICKS_PER_REV) * 60.0;
        motorPower = pidController.calculate(targetRPM, currentRPM) + feedforward;
        motorPower = Math.max(-1.0, Math.min(motorPower, 1.0));
        if (motorPower > 0 && motorPower < OuttakeConstants.MIN_POWER) {
            motorPower = OuttakeConstants.MIN_POWER;
        }
        turretMotor.setPower(-motorPower);
    }

    public void stop() {
        isActive = false;
        targetRPM = 0;
        turretMotor.setPower(0);
    }

    public boolean isAtTargetSpeed() {
        return Math.abs(targetRPM - currentRPM) < OuttakeConstants.RPM_TOLERANCE;
    }

    public double getTargetRPM() { return targetRPM; }
    public double getCurrentRPM() { return currentRPM; }
    public double getMotorPower() { return motorPower; }
    public double getError() { return targetRPM - currentRPM; }
    public boolean isActive() { return isActive; }
}
