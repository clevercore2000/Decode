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
    private Rotation2d lastNonZeroAngle = new Rotation2d(0); // Cache for angle holding

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
        // Return raw angle (normalization happens in error calculation for shortest path)
        return encoder.getAngleRadians();
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        setDesiredState(desiredState, true); // Default to optimizing
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean optimize) {
        // Detect if this is a "stationary default" state from kinematics
        // When drive(0,0,0) is called → kinematics returns speed=0, angle=0 (default)
        boolean isStationaryDefault =
                Math.abs(desiredState.speedMetersPerSecond) < 0.001 &&
                Math.abs(desiredState.angle.getRadians()) < 0.001;

        // If stationary with default angle, preserve last known direction
        SwerveModuleState stateToUse;
        if (isStationaryDefault) {
            // Use cached angle from last movement (holds module position)
            stateToUse = new SwerveModuleState(0, lastNonZeroAngle);
        } else {
            stateToUse = desiredState;
        }

        // Cache the DESIRED angle BEFORE optimization to remember intended direction
        // (not the optimized angle which may be flipped 180°)

        //DO NOT UNCOMMENT IT BREAKS STUFF
      /*  if (!isStationaryDefault && Math.abs(desiredState.speedMetersPerSecond) > 0.001) {
            lastNonZeroAngle = desiredState.angle;
        }*/

        // CRITICAL: Only optimize when actually moving
        // Optimizing zero-velocity commands causes modules to flip 180° for no reason
        SwerveModuleState stateToSet;
        if (optimize && Math.abs(stateToUse.speedMetersPerSecond) > 0.1) {
            stateToSet = SwerveModuleState.optimize(
                    stateToUse,
                    new Rotation2d(getSteeringAngle())
            );
        } else {
            // Zero velocity or optimization disabled: use state as-is
            stateToSet = stateToUse;
        }

        this.desiredState = stateToSet;

        // Check if the angle was flipped and invert the drive power accordingly
        double drivePower = stateToUse.speedMetersPerSecond * DriveConstants.DRIVE_FF;
        double angleDifference = Math.abs(
                MathUtils.normalizeAngleRadians(stateToUse.angle.getRadians() - stateToSet.angle.getRadians())
        );
        if (angleDifference > Math.PI) {
            drivePower *= -1.0;
        }

        double current = getSteeringAngle();
        double target = stateToSet.angle.getRadians();
        // Calculate shortest path error (handles 0°/360° wrapping correctly)
        // e.g., current=350°, target=10° → error=+20° (not -340°)
        double error = MathUtils.shortestAngularDistance(current, target);

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

        drivePower = Math.max(-1.0, Math.min(1.0, drivePower));
        if (driveInverted) {
            drivePower *= -1.0;
        }
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
        telemetry.addData(moduleName, "%.0f° → %.0f°", Math.toDegrees(current), Math.toDegrees(target));
        telemetry.addData("state", getState());
    }
}
