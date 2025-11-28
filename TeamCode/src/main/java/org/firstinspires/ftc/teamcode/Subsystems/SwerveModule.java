package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Constants.SteeringConstants;
import org.firstinspires.ftc.teamcode.Hardware.AxonEncoder;
import org.firstinspires.ftc.teamcode.Utils.MathUtils;

public class SwerveModule {
    private final DcMotorEx driveMotor;
    private final CRServo steerServo;
    private final AxonEncoder encoder;
    private final boolean steerInverted;
    private final String moduleName;

    private SwerveModuleState desiredState = new SwerveModuleState();

    public SwerveModule(
            DcMotorEx driveMotor,
            CRServo steerServo,
            AxonEncoder encoder,
            boolean driveInverted,
            boolean steerInverted,
            String moduleName
    ) {
        this.driveMotor = driveMotor;
        this.steerServo = steerServo;
        this.encoder = encoder;
        this.steerInverted = steerInverted;
        this.moduleName = moduleName;

        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (driveInverted) {
            driveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public double getSteeringAngle() {
        double angleRadians = encoder.getAngleRadians();
        return MathUtils.normalizeAngleRadians(angleRadians);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // CRITICAL FIX: Skip angle updates when velocity is zero
        // Prevents atan2(0,0) singularity and maintains last angle when idle
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            // Idle mode: maintain current angle, stop drive motor
            driveMotor.setPower(0.0);
            // Keep servo electronics active for encoder reading
            steerServo.setPower(SteeringConstants.MIN_SERVO_POWER);
            this.desiredState = new SwerveModuleState(
                    0.0,
                    new Rotation2d(getSteeringAngle())  // Maintain current angle
            );
            return;  // Exit early, don't update angle
        }

        SwerveModuleState optimized = SwerveModuleState.optimize(
                desiredState,
                new Rotation2d(getSteeringAngle())
        );

        this.desiredState = optimized;

        double current = getSteeringAngle();
        double target = optimized.angle.getRadians();
        double error = MathUtils.normalizeAngleRadians(target - current);

        double steerPower;
        if (Math.abs(error) < SteeringConstants.STEERING_DEADBAND_RADIANS) {
            steerPower = SteeringConstants.MIN_SERVO_POWER;
        } else {
            steerPower = error * SteeringConstants.STEER_P;
            steerPower += Math.signum(steerPower) * SteeringConstants.STATIC_FRICTION_COMPENSATION;
            steerPower = Math.max(-1.0, Math.min(1.0, steerPower));

            if (Math.abs(steerPower) < SteeringConstants.MIN_SERVO_POWER) {
                steerPower = Math.signum(error) * SteeringConstants.MIN_SERVO_POWER;
            }
        }

        steerServo.setPower(steerInverted ? -steerPower : steerPower);

        double drivePower = optimized.speedMetersPerSecond * DriveConstants.DRIVE_FF;
        drivePower = Math.max(-1.0, Math.min(1.0, drivePower));
        driveMotor.setPower(drivePower);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(0.0, new Rotation2d(getSteeringAngle()));
    }

    public void stop() {
        driveMotor.setPower(0.0);
        steerServo.setPower(SteeringConstants.MIN_SERVO_POWER);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- " + moduleName + " ---");

        double current = getSteeringAngle();
        double target = desiredState.angle.getRadians();
        double error = MathUtils.normalizeAngleRadians(target - current);

        telemetry.addData("Raw Voltage", "%.3f V", encoder.getRawVoltage());
        telemetry.addData("Current Angle", "%.1f°", Math.toDegrees(current));
        telemetry.addData("Target Angle", "%.1f°", Math.toDegrees(target));
        telemetry.addData("Error", "%.1f°", Math.toDegrees(error));
        telemetry.addData("Steer Power", "%.3f", steerServo.getPower());
        telemetry.addData("Drive Power", "%.3f", driveMotor.getPower());
    }
}
