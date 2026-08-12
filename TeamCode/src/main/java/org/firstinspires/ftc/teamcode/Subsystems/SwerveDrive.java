package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Constants.SteeringConstants;
import org.firstinspires.ftc.teamcode.Hardware.RevThroughBoreEncoder;
import org.firstinspires.ftc.teamcode.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.kinematics.SwerveDriveKinematics;
import org.firstinspires.ftc.teamcode.kinematics.SwerveModuleState;
import org.firstinspires.ftc.teamcode.kinematics.Translation2d;

public class SwerveDrive {
    public final SwerveModule fl, fr, bl, br;
    private final SwerveDriveKinematics kinematics;

    public SwerveDrive(HardwareMap hardwareMap) {
        fl = createModule(hardwareMap, "fl",
                SteeringConstants.FL_ENCODER_NAME, SteeringConstants.FL_ENCODER_SHARED,
                SteeringConstants.FL_SWITCH_NAME, SteeringConstants.FL_TICK_OFFSET,
                SteeringConstants.FL_DRIVE_INVERTED, SteeringConstants.FL_ENCODER_INVERTED,
                SteeringConstants.FL_STEER_INVERTED);
        fr = createModule(hardwareMap, "fr",
                SteeringConstants.FR_ENCODER_NAME, SteeringConstants.FR_ENCODER_SHARED,
                SteeringConstants.FR_SWITCH_NAME, SteeringConstants.FR_TICK_OFFSET,
                SteeringConstants.FR_DRIVE_INVERTED, SteeringConstants.FR_ENCODER_INVERTED,
                SteeringConstants.FR_STEER_INVERTED);
        bl = createModule(hardwareMap, "bl",
                SteeringConstants.BL_ENCODER_NAME, SteeringConstants.BL_ENCODER_SHARED,
                SteeringConstants.BL_SWITCH_NAME, SteeringConstants.BL_TICK_OFFSET,
                SteeringConstants.BL_DRIVE_INVERTED, SteeringConstants.BL_ENCODER_INVERTED,
                SteeringConstants.BL_STEER_INVERTED);
        br = createModule(hardwareMap, "br",
                SteeringConstants.BR_ENCODER_NAME, SteeringConstants.BR_ENCODER_SHARED,
                SteeringConstants.BR_SWITCH_NAME, SteeringConstants.BR_TICK_OFFSET,
                SteeringConstants.BR_DRIVE_INVERTED, SteeringConstants.BR_ENCODER_INVERTED,
                SteeringConstants.BR_STEER_INVERTED);

        // FTCLib kinematics: +x = forward, +y = left
        double halfWB = DriveConstants.WHEELBASE_METERS / 2.0;
        double halfTW = DriveConstants.TRACK_WIDTH_METERS / 2.0;
        kinematics = new SwerveDriveKinematics(
                new Translation2d(halfWB, halfTW),    // FL
                new Translation2d(halfWB, -halfTW),   // FR
                new Translation2d(-halfWB, halfTW),   // BL
                new Translation2d(-halfWB, -halfTW)   // BR
        );
    }

    private SwerveModule createModule(
            HardwareMap hardwareMap,
            String name,
            String encoderName,
            boolean encoderShared,
            String switchName,
            int tickOffset,
            boolean driveInverted,
            boolean encoderInverted,
            boolean steerInverted
    ) {
        RevThroughBoreEncoder encoder = new RevThroughBoreEncoder(
                hardwareMap.get(DcMotorEx.class, encoderName), encoderShared
        );
        encoder.setInverted(encoderInverted);

        return new SwerveModule(
                hardwareMap.get(DcMotorEx.class, name),
                hardwareMap.get(CRServo.class, name + "_servo"),
                encoder,
                hardwareMap.get(DigitalChannel.class, switchName),
                driveInverted,
                steerInverted,
                name.toUpperCase(),
                tickOffset
        );
    }

    /**
     * Home all 4 modules simultaneously with dual-stage homing. Must be called after
     * {@code waitForStart()} — it runs while the opmode is active.
     */
    public boolean homeAllModules(LinearOpMode opMode) {
        return homeAllModules(opMode, false);
    }

    /**
     * Same dual-stage homing, but runnable during the init phase. Autonomous needs this:
     * {@link #homeAllModules} gates on {@code opModeIsActive()}, which is false before start,
     * so calling it from init would fall straight through and report success having homed
     * nothing. Homing in init also keeps it off the 30 second autonomous clock.
     */
    public boolean homeAllModulesDuringInit(LinearOpMode opMode) {
        return homeAllModules(opMode, true);
    }

    private boolean homeAllModules(LinearOpMode opMode, boolean duringInit) {
        SwerveModule[] modules = {fl, fr, bl, br};
        boolean[] done = new boolean[4];
        long startTime = System.currentTimeMillis();

        for (int i = 0; i < 4; i++) {
            if (modules[i].isLimitSwitchPressed()) {
                modules[i].finishHoming();
                done[i] = true;
            } else {
                modules[i].startHoming();
            }
        }

        while (duringInit ? !opMode.isStopRequested() : opMode.opModeIsActive()) {
            boolean allDone = true;
            for (int i = 0; i < 4; i++) {
                if (done[i]) continue;
                if (modules[i].updateHoming()) {
                    done[i] = true;
                } else {
                    allDone = false;
                }
            }
            if (allDone) break;

            if (System.currentTimeMillis() - startTime > SteeringConstants.HOMING_TIMEOUT_MS) {
                for (int i = 0; i < 4; i++) {
                    if (!done[i]) modules[i].stop();
                }
                return false;
            }
        }

        return true;
    }


    /**
     * Aims all four modules forward and blocks until they settle there. Must be called after
     * {@code waitForStart()}.
     */
    public boolean alignModulesForward(LinearOpMode opMode) {
        return alignModulesForward(opMode, false);
    }

    /** Init-phase variant of {@link #alignModulesForward(LinearOpMode)}. */
    public boolean alignModulesForwardDuringInit(LinearOpMode opMode) {
        return alignModulesForward(opMode, true);
    }

    /**
     * Runs the steering loop with zero wheel speed until every module holds 0° within
     * {@link SteeringConstants#ALIGN_TOLERANCE_RADIANS} for {@code ALIGN_SETTLE_MS}.
     * <p>
     * Homing alone is not enough: {@code finishHoming()} sets the target to 0° but only
     * {@link #update()} actuates a servo, so without this the modules sit at the limit switch
     * and are still rotating into place when the first path applies drive power — which shows
     * up as position error at the start of every path.
     *
     * @return false on timeout, meaning at least one module never settled.
     */
    private boolean alignModulesForward(LinearOpMode opMode, boolean duringInit) {
        SwerveModule[] modules = {fl, fr, bl, br};
        long start = System.currentTimeMillis();
        long settledSince = -1;

        while (duringInit ? !opMode.isStopRequested() : opMode.opModeIsActive()) {
            for (SwerveModule m : modules) m.setTarget(0, 0);
            update();

            boolean allWithin = true;
            for (SwerveModule m : modules) {
                if (Math.abs(m.getSteerErrorRad()) > SteeringConstants.ALIGN_TOLERANCE_RADIANS) {
                    allWithin = false;
                    break;
                }
            }

            long elapsed = System.currentTimeMillis() - start;

            if (allWithin) {
                // Require the tolerance to hold for a dwell rather than accepting the first
                // sample inside it, which a module can satisfy while still swinging through.
                if (settledSince < 0) settledSince = System.currentTimeMillis();
                boolean settled = System.currentTimeMillis() - settledSince >= SteeringConstants.ALIGN_SETTLE_MS;
                // ALIGN_HOLD_MS is a floor on the whole operation, not just the settle window:
                // it keeps driving the modules onto 0° for the full period so the result is the
                // same on carpet as it is with the wheels off the ground.
                if (settled && elapsed >= SteeringConstants.ALIGN_HOLD_MS) {
                    return true;
                }
            } else {
                settledSince = -1;
            }

            if (System.currentTimeMillis() - start > SteeringConstants.ALIGN_TIMEOUT_MS) {
                return false;
            }
        }
        return false;
    }

    /**
     * Holds all modules at 0° with zero drive power. Call every loop while waiting to start a
     * path — it keeps the PD loop live so the modules stay where {@link #alignModulesForward}
     * put them instead of drifting, and leaves the servos parked inside the PD deadband.
     */
    public void holdForward() {
        fl.setTarget(0, 0);
        fr.setTarget(0, 0);
        bl.setTarget(0, 0);
        br.setTarget(0, 0);
        update();
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    /**
     * Driver-facing entry point. Applies a stick deadband — below it the wheels hold their
     * current azimuth and coast — then defers to {@link #setChassisSpeeds}.
     * <p>
     * Autonomous must not use this: a path follower emits arbitrarily small corrections, and
     * the deadband would discard exactly the ones that keep the robot on the path.
     */
    public void drive(double fwd, double str, double rot) {
        if (Math.abs(fwd) < 0.01 && Math.abs(str) < 0.01 && Math.abs(rot) < 0.01) {
            for (SwerveModule m : new SwerveModule[]{fl, fr, bl, br}) {
                m.setTarget(m.getTargetAngle(), 0);
            }
            return;
        }
        setChassisSpeeds(new ChassisSpeeds(fwd, str, rot));
    }

    /**
     * Commands a robot-relative chassis motion with no deadband: vx forward, vy left,
     * omega CCW-positive. Wheel speeds are normalised so the fastest module sits at 1.0, so
     * the inputs are treated as a direction plus a relative magnitude rather than true m/s.
     * <p>
     * Call {@link #update()} afterwards to actually drive the hardware.
     */
    public void setChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModule[] modules = {fl, fr, bl, br};
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.normalizeWheelSpeeds(states, 1.0);

        for (int i = 0; i < 4; i++) {
            modules[i].setTarget(states[i].angle.getRadians(), states[i].speedMetersPerSecond);
        }
    }

    /**
     * Commands the azimuth pattern that {@code shape} would produce, but with zero wheel speed
     * — the modules aim as if they were about to perform that motion without driving.
     * <p>
     * Diagnostic aid: it separates "the modules cannot reach the pattern" from "the modules
     * reach it but the wheels fight each other", which look identical once drive power is on.
     */
    public void setAzimuthsOnly(ChassisSpeeds shape) {
        SwerveModule[] modules = {fl, fr, bl, br};
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(shape);
        for (int i = 0; i < 4; i++) {
            modules[i].setTarget(states[i].angle.getRadians(), 0);
        }
    }

    /**
     * Run PID and write hardware for all modules. Call every loop iteration.
     */
    public void update() {
        applyTrims();
        fl.update();
        fr.update();
        bl.update();
        br.update();
    }

    /**
     * Pushes the azimuth trims in from constants every loop rather than at construction, so
     * they can be dialled in from the dashboard while the robot is driving.
     */
    private void applyTrims() {
        fl.setSteerTrimRad(Math.toRadians(SteeringConstants.FL_TRIM_DEG));
        fr.setSteerTrimRad(Math.toRadians(SteeringConstants.FR_TRIM_DEG));
        bl.setSteerTrimRad(Math.toRadians(SteeringConstants.BL_TRIM_DEG));
        br.setSteerTrimRad(Math.toRadians(SteeringConstants.BR_TRIM_DEG));
    }

    public void hold() {
        fl.hold();
        fr.hold();
        bl.hold();
        br.hold();
    }

    public void log(Telemetry telemetry) {
        fl.log(telemetry);
        fr.log(telemetry);
        bl.log(telemetry);
        br.log(telemetry);
    }

    public void logDetailed(Telemetry telemetry) {
        fl.logDetailed(telemetry);
        fr.logDetailed(telemetry);
        bl.logDetailed(telemetry);
        br.logDetailed(telemetry);
    }
}
