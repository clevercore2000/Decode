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
    private final boolean driveInverted;
    private final boolean steerInverted;
    private final String moduleName;

    private SwerveModuleState desiredState = new SwerveModuleState();
    private Rotation2d lastNonZeroAngle = null;
    private double lastError = 0.0;
    private long lastTime = System.nanoTime();

    // Continuous angle tracking (fixes 0°/360° wrapping)
    private double prevRawAngleDeg = 0.0;
    private double unwrappedAngleDeg = 0.0;
    private boolean firstAngleUpdate = true;

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
        this.driveInverted = driveInverted;
        this.steerInverted = steerInverted;
        this.moduleName = moduleName;

        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double getSteeringAngle() {
        double rawServoDeg = encoder.getRawAngleDegrees180();

        if (firstAngleUpdate) {
            prevRawAngleDeg = rawServoDeg;
            unwrappedAngleDeg = rawServoDeg;
            firstAngleUpdate = false;
        }

        // Unwrap delta to avoid jumps at boundary
        double delta = rawServoDeg - prevRawAngleDeg;
        if (delta > 180.0) delta -= 360.0;
        if (delta < -180.0) delta += 360.0;

        unwrappedAngleDeg += delta;
        prevRawAngleDeg = rawServoDeg;

        // 2:1 gear ratio: wheel = servo / 2
        double wheelAngleDeg = unwrappedAngleDeg / 2.0;
        return Math.toRadians(MathUtils.wrap180(wheelAngleDeg));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        setDesiredState(desiredState, true); // Default to optimizing
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean optimize) {
        if (lastNonZeroAngle == null) {
            lastNonZeroAngle = new Rotation2d(getSteeringAngle());
        }

        // When stopped, hold last direction instead of snapping to 0°
        boolean isStationaryDefault =
                Math.abs(desiredState.speedMetersPerSecond) < 0.001 &&
                Math.abs(desiredState.angle.getRadians()) < 0.001;

        SwerveModuleState stateToUse = isStationaryDefault
                ? new SwerveModuleState(0, lastNonZeroAngle)
                : desiredState;

        SwerveModuleState stateToSet = optimize
                ? SwerveModuleState.optimize(stateToUse, new Rotation2d(getSteeringAngle()))
                : stateToUse;

        // Cache optimized angle for hold behavior
        if (!isStationaryDefault && Math.abs(desiredState.speedMetersPerSecond) > 0.001) {
            lastNonZeroAngle = stateToSet.angle;
        }

        double targetRad = MathUtils.normalizeAngleRadians(stateToSet.angle.getRadians());
        double speed = stateToSet.speedMetersPerSecond;

        // Clamp to ±90° - reverse drive instead of rotating further
        if (targetRad > Math.PI / 2) {
            targetRad -= Math.PI;
            speed = -speed;
        } else if (targetRad < -Math.PI / 2) {
            targetRad += Math.PI;
            speed = -speed;
        }

        this.desiredState = stateToSet;

        double drivePower = speed * DriveConstants.DRIVE_FF;
        double error = MathUtils.shortestAngularDistance(getSteeringAngle(), targetRad);

        // Calculate derivative
        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        lastTime = now;
        double derivative = (dt > 0) ? (error - lastError) / dt : 0;
        lastError = error;

        double steerPower;
        if (Math.abs(error) < SteeringConstants.STEERING_DEADBAND_RADIANS) {
            steerPower = SteeringConstants.MIN_SERVO_POWER;
        } else {
            // PD control
            steerPower = error * SteeringConstants.STEER_P + derivative * SteeringConstants.STEER_D;
            steerPower += Math.signum(steerPower) * SteeringConstants.STATIC_FRICTION_COMPENSATION;
            steerPower = Math.max(-1.0, Math.min(1.0, steerPower));

            if (Math.abs(steerPower) < SteeringConstants.MIN_SERVO_POWER) {
                steerPower = Math.signum(error) * SteeringConstants.MIN_SERVO_POWER;
            }
        }

        steerServo.setPower(steerInverted ? -steerPower : steerPower);

        drivePower = Math.max(-1.0, Math.min(1.0, drivePower));
        if (driveInverted) drivePower *= -1.0;
        driveMotor.setPower(drivePower);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(0.0, new Rotation2d(getSteeringAngle()));
    }

    public void hold() {
        driveMotor.setPower(0.0);
        steerServo.setPower(SteeringConstants.MIN_SERVO_POWER);
    }

    public void addTelemetry(Telemetry telemetry) {
        double current = getSteeringAngle();
        double target = desiredState.angle.getRadians();
        double err = Math.toDegrees(MathUtils.shortestAngularDistance(current, target));
        telemetry.addData(moduleName, "%.1f° → %.1f° (err: %.1f°)",
                Math.toDegrees(current), Math.toDegrees(target), err);
    }
}
