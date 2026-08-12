package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Constants.SteeringConstants;
import org.firstinspires.ftc.teamcode.Hardware.RevThroughBoreEncoder;
import org.firstinspires.ftc.teamcode.Utils.MathUtils;
import org.firstinspires.ftc.teamcode.kinematics.SwerveModuleState;
import org.firstinspires.ftc.teamcode.kinematics.Rotation2d;

public class SwerveModule {
    private final DcMotorEx driveMotor;
    private final CRServo steerServo;
    private final RevThroughBoreEncoder encoder;
    private final DigitalChannel limitSwitch;
    private final boolean driveInverted;
    private final boolean steerInverted;
    private final String moduleName;
    private final int calibratedTickOffset;

    private enum HomingStage { IDLE, FAST_APPROACH, BACKOFF, SLOW_APPROACH, DONE }

    private SwerveModuleState targetState = new SwerveModuleState(0, new Rotation2d(0));
    private double lastAngle = 0.0;
    private long lastTime = System.nanoTime();
    private boolean homed = false;
    private boolean wheelFlipped = false;
    private double lastSteerError = 0.0;
    private double steerTrimRad = 0.0;

    private HomingStage homingStage = HomingStage.IDLE;
    private long backoffStartTime;

    public SwerveModule(
            DcMotorEx driveMotor,
            CRServo steerServo,
            RevThroughBoreEncoder encoder,
            DigitalChannel limitSwitch,
            boolean driveInverted,
            boolean steerInverted,
            String moduleName,
            int calibratedTickOffset
    ) {
        this.driveMotor = driveMotor;
        this.steerServo = steerServo;
        this.encoder = encoder;
        this.limitSwitch = limitSwitch;
        this.driveInverted = driveInverted;
        this.steerInverted = steerInverted;
        this.moduleName = moduleName;
        this.calibratedTickOffset = calibratedTickOffset;

        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (limitSwitch != null) {
            limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        }

        steerServo.setPower(0);
    }

    public void setTarget(double angleRad, double speed) {
        targetState = new SwerveModuleState(speed, new Rotation2d(angleRad));
    }

    public double getTargetAngle() {
        return targetState.angle.getRadians();
    }

    public void update() {
        if (!homed) {
            driveMotor.setPower(0);
            steerServo.setPower(0);
            return;
        }

        double currentAngle = getAngle();
        Rotation2d currentRotation = new Rotation2d(currentAngle);

        SwerveModuleState optimized = SwerveModuleState.optimize(targetState, currentRotation);

        wheelFlipped = Math.abs(optimized.angle.minus(targetState.angle).getRadians()) > 1.0;

        // Shortest path error in radians (Guaranteed <= PI/2)
        double steerError = optimized.angle.minus(currentRotation).getRadians();
        lastSteerError = steerError;

        // PD Controller
        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        lastTime = now;
        double derivative = (dt > 0) ? -(currentAngle - lastAngle) / dt : 0;
        lastAngle = currentAngle;

        double steerPower = steerError * SteeringConstants.STEER_P + derivative * SteeringConstants.STEER_D;

        if (Math.abs(steerError) > SteeringConstants.STEERING_DEADBAND_RADIANS) {
            steerPower += Math.signum(steerPower) * SteeringConstants.STATIC_FRICTION_COMPENSATION;
            if (Math.abs(steerPower) < SteeringConstants.MIN_SERVO_POWER) {
                steerPower = Math.signum(steerPower) * SteeringConstants.MIN_SERVO_POWER;
            }
            steerPower = Math.max(-1.0, Math.min(1.0, steerPower));
        } else {
            steerPower = 0;
        }

        steerServo.setPower(steerInverted ? -steerPower : steerPower);

        // Drive Motor: Cosine scale for misalignment
        double drivePower = optimized.speedMetersPerSecond * DriveConstants.DRIVE_FF * Math.max(0, Math.cos(steerError));
        if (driveInverted) drivePower *= -1;
        driveMotor.setPower(drivePower);
    }

    /**
     * Steering error the PD loop acted on during the last {@link #update()}, in radians.
     * <p>
     * This is the post-{@code optimize()} error, so a module sitting 180° from its target reads
     * ~0 — correct, because at that azimuth the wheel simply drives in reverse. Stale until
     * {@code update()} has run at least once since homing.
     */
    public double getSteerErrorRad() { return lastSteerError; }

    /**
     * Fine azimuth correction added to every angle reading, radians. Set from
     * {@code SteeringConstants.*_TRIM_DEG} so it stays live-tunable; see the constants for the
     * sign convention and how to tune it.
     */
    public void setSteerTrimRad(double trimRad) { this.steerTrimRad = trimRad; }

    /**
     * Wheel azimuth in radians, trim included — this is the angle the whole control path works
     * in, so a trimmed module reports where its wheel really points rather than what its
     * encoder says.
     */
    public double getAngle() { return encoder.getWheelAngleRad() + steerTrimRad; }
    public int getRawTicks() { return encoder.getRawTicks(); }
    public int getPositionTicks() { return encoder.getPositionTicks(); }

    public void hold() { setTarget(getAngle(), 0); update(); }
    public void stop() { driveMotor.setPower(0); steerServo.setPower(0); }

    public boolean isLimitSwitchPressed() {
        return limitSwitch != null && limitSwitch.getState() == SteeringConstants.LIMIT_SWITCH_ACTIVE_STATE;
    }

    public void startHoming() {
        homingStage = HomingStage.FAST_APPROACH;
        double power = SteeringConstants.HOMING_FAST_POWER;
        if (steerInverted) power = -power;
        steerServo.setPower(power);
    }

    public boolean updateHoming() {
        switch (homingStage) {
            case FAST_APPROACH:
                if (isLimitSwitchPressed()) {
                    double backoff = SteeringConstants.HOMING_BACKOFF_POWER;
                    if (steerInverted) backoff = -backoff;
                    steerServo.setPower(backoff);
                    backoffStartTime = System.currentTimeMillis();
                    homingStage = HomingStage.BACKOFF;
                }
                break;
            case BACKOFF:
                if (System.currentTimeMillis() - backoffStartTime >= SteeringConstants.HOMING_BACKOFF_MS) {
                    double slow = SteeringConstants.HOMING_SLOW_POWER;
                    if (steerInverted) slow = -slow;
                    steerServo.setPower(slow);
                    homingStage = HomingStage.SLOW_APPROACH;
                }
                break;
            case SLOW_APPROACH:
                if (isLimitSwitchPressed()) {
                    finishHoming();
                    homingStage = HomingStage.DONE;
                }
                break;
            case DONE: return true;
        }
        return homingStage == HomingStage.DONE;
    }

    public void finishHoming() {
        steerServo.setPower(0);
        encoder.setHomePosition(calibratedTickOffset);
        double current = getAngle();
        lastAngle = current;
        setTarget(0, 0); // Aim forward immediately
        homed = true;
    }

    public boolean homeModule(LinearOpMode opMode) {
        if (limitSwitch == null) { homed = true; setTarget(getAngle(), 0); return true; }
        startHoming();
        while (opMode.opModeIsActive()) {
            if (updateHoming()) return true;
        }
        return false;
    }

    public boolean isHomed() { return homed; }

    public void log(Telemetry telemetry) {
        double currentDeg = MathUtils.wrapDeg(Math.toDegrees(getAngle()));
        double targetDeg = MathUtils.wrapDeg(Math.toDegrees(targetState.angle.getRadians()));
        telemetry.addData(moduleName, "%.0f° → %.0f°%s", currentDeg, targetDeg, wheelFlipped ? " [F]" : "");
    }

    public void logDetailed(Telemetry telemetry) {
        double currentRad = getAngle();
        double currentDeg = MathUtils.wrapDeg(Math.toDegrees(currentRad));
        double targetDeg = MathUtils.wrapDeg(Math.toDegrees(targetState.angle.getRadians()));
        telemetry.addLine("--- " + moduleName + " ---");
        telemetry.addData("  Angle", "%.1f° → %.1f°", currentDeg, targetDeg);
        telemetry.addData("  Homed", "%s", homed);
        telemetry.addData("  Servo Power", "%.2f", steerServo.getPower());
        telemetry.addData("  Drive Power", "%.2f", driveMotor.getPower());
    }
}
