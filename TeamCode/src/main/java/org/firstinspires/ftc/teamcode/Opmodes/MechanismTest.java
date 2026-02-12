package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.Constants.SorterConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.PIDController;
import org.firstinspires.ftc.teamcode.Subsystems.Sorter;

@TeleOp(name = "Mechanism Test", group = "Test")
public class MechanismTest extends LinearOpMode {

    enum Mode { RAW, TURRET_PID, DRUM_SLOTS, FULL_SEQUENCE }

    @Override
    public void runOpMode() {
        Hardware hw = new Hardware(hardwareMap);
        Outtake outtake = new Outtake(hw);
        Sorter sorter = new Sorter(hw, outtake);
        PIDController drumPID = new PIDController(
                SorterConstants.POSITION_P,
                SorterConstants.POSITION_I,
                SorterConstants.POSITION_D
        );

        Mode mode = Mode.RAW;
        boolean lastLeft = false, lastRight = false;
        boolean lastUp = false, lastDown = false;
        boolean lastCross = false, lastTriangle = false, lastCircle = false;
        boolean lastRBumper = false, lastLBumper = false;

        // Turret PID mode
        double turretTargetRPM = OuttakeConstants.TARGET_RPM;

        // Drum slots mode
        int targetSlot = 0;
        double drumTarget = 0;
        boolean drumPIDActive = false;

        // Color detection state
        boolean ballWasPresent = false;
        String lastDetected = "none";

        telemetry.addLine("== Mechanism Test ==");
        telemetry.addLine("Dpad L/R: switch mode");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Edge detection for all buttons
            boolean leftPress = gamepad2.dpad_left && !lastLeft;
            boolean rightPress = gamepad2.dpad_right && !lastRight;
            boolean upPress = gamepad2.dpad_up && !lastUp;
            boolean downPress = gamepad2.dpad_down && !lastDown;
            boolean crossPress = gamepad2.cross && !lastCross;
            boolean trianglePress = gamepad2.triangle && !lastTriangle;
            boolean circlePress = gamepad2.circle && !lastCircle;
            boolean rbPress = gamepad2.right_bumper && !lastRBumper;
            boolean lbPress = gamepad2.left_bumper && !lastLBumper;
            lastLeft = gamepad2.dpad_left;
            lastRight = gamepad2.dpad_right;
            lastUp = gamepad2.dpad_up;
            lastDown = gamepad2.dpad_down;
            lastCross = gamepad2.cross;
            lastTriangle = gamepad2.triangle;
            lastCircle = gamepad2.circle;
            lastRBumper = gamepad2.right_bumper;
            lastLBumper = gamepad2.left_bumper;

            // Mode switch
            if (rightPress) mode = Mode.values()[(mode.ordinal() + 1) % Mode.values().length];
            if (leftPress) mode = Mode.values()[(mode.ordinal() + Mode.values().length - 1) % Mode.values().length];

            // Sensor reads (always)
            int r = hw.sorterHardware.ColorSensor.red();
            int g = hw.sorterHardware.ColorSensor.green();
            int b = hw.sorterHardware.ColorSensor.blue();
            int total = r + g + b;
            boolean ballPresent = total > SorterConstants.DETECTION_THRESHOLD;
            if (ballPresent && !ballWasPresent) {
                double greenRatio = (double) g / total;
                lastDetected = (greenRatio > SorterConstants.GREEN_RATIO_THRESHOLD) ? "GREEN" : "PURPLE";
            }
            if (!ballPresent) lastDetected = "none";
            ballWasPresent = ballPresent;

            double turretVel = hw.outtakeHardware.TurretMotor.getVelocity();
            double turretRPM = (turretVel / OuttakeConstants.MOTOR_TICKS_PER_REV) * 60.0;
            int drumPos = hw.sorterHardware.SorterMotor.getCurrentPosition();

            switch (mode) {
                case RAW:
                    // Square: intake, Cross: turret raw, Triangle/Circle: drum fwd/rev
                    hw.intakeHardware.IntakeMotor.setPower(gamepad2.square ? 0.9 : 0);
                    hw.outtakeHardware.TurretMotor.setPower(gamepad2.cross ? 0.5 : 0);
                    if (gamepad2.triangle) hw.sorterHardware.SorterMotor.setPower(0.3);
                    else if (gamepad2.circle) hw.sorterHardware.SorterMotor.setPower(-0.3);
                    else hw.sorterHardware.SorterMotor.setPower(0);
                    if (gamepad2.right_bumper) hw.sorterHardware.KickServo.setPosition(SorterConstants.KICK_POSITION);
                    if (gamepad2.left_bumper) hw.sorterHardware.KickServo.setPosition(SorterConstants.RETRACT_POSITION);

                    telemetry.addLine("=== RAW MODE ===");
                    telemetry.addLine("Square=intake  Cross=turret");
                    telemetry.addLine("Tri=drum+  Circle=drum-");
                    telemetry.addLine("RB=kick  LB=retract");
                    break;

                case TURRET_PID:
                    // Test turret with PID velocity control
                    if (upPress) turretTargetRPM = Math.min(turretTargetRPM + 200, OuttakeConstants.MAX_MOTOR_RPM);
                    if (downPress) turretTargetRPM = Math.max(turretTargetRPM - 200, 0);

                    if (gamepad2.cross) {
                        outtake.setTargetRPM(turretTargetRPM);
                    } else {
                        outtake.setTargetRPM(0);
                    }
                    outtake.update();

                    telemetry.addLine("=== TURRET PID MODE ===");
                    telemetry.addLine("Hold Cross = spin turret");
                    telemetry.addLine("Dpad Up/Down = RPM +/-200");
                    telemetry.addData("Target RPM", "%.0f", turretTargetRPM);
                    telemetry.addData("Actual RPM", "%.0f", outtake.getCurrentRPM());
                    telemetry.addData("Error", "%.1f", outtake.getError());
                    telemetry.addData("Motor Power", "%.3f", outtake.getMotorPower());
                    telemetry.addData("At Target?", outtake.isAtTargetSpeed());
                    break;

                case DRUM_SLOTS:
                    // Test drum PID to each slot position
                    if (trianglePress) targetSlot = (targetSlot + 1) % 3;
                    if (circlePress) targetSlot = (targetSlot + 2) % 3;
                    drumTarget = SorterConstants.SLOT_POSITIONS[targetSlot];

                    if (crossPress) drumPIDActive = !drumPIDActive;

                    if (drumPIDActive) {
                        double err = drumTarget - drumPos;
                        if (Math.abs(err) > SorterConstants.POSITION_TOLERANCE) {
                            hw.sorterHardware.SorterMotor.setPower(drumPID.calculate(drumTarget, drumPos));
                        } else {
                            hw.sorterHardware.SorterMotor.setPower(0);
                        }
                    } else {
                        hw.sorterHardware.SorterMotor.setPower(0);
                    }

                    if (gamepad2.right_bumper) hw.sorterHardware.KickServo.setPosition(SorterConstants.KICK_POSITION);
                    if (gamepad2.left_bumper) hw.sorterHardware.KickServo.setPosition(SorterConstants.RETRACT_POSITION);

                    telemetry.addLine("=== DRUM SLOTS MODE ===");
                    telemetry.addLine("Tri/Circle = next/prev slot");
                    telemetry.addLine("Cross = toggle PID drive");
                    telemetry.addLine("RB=kick  LB=retract");
                    telemetry.addData("Target Slot", targetSlot);
                    telemetry.addData("Target Ticks", "%.0f", drumTarget);
                    telemetry.addData("Current Ticks", drumPos);
                    telemetry.addData("Error", "%.0f", drumTarget - drumPos);
                    telemetry.addData("PID Active", drumPIDActive);
                    telemetry.addData("Slot 0", "%.0f ticks", SorterConstants.SLOT_POSITIONS[0]);
                    telemetry.addData("Slot 1", "%.0f ticks", SorterConstants.SLOT_POSITIONS[1]);
                    telemetry.addData("Slot 2", "%.0f ticks", SorterConstants.SLOT_POSITIONS[2]);
                    break;

                case FULL_SEQUENCE:
                    // Test the full sorter sequence
                    hw.intakeHardware.IntakeMotor.setPower(gamepad2.square ? 0.9 : 0);
                    sorter.checkIntake();

                    if (crossPress && sorter.isIdle()) sorter.startShootSequence();
                    if (trianglePress) {
                        // Manually add a green ball
                        sorter.debugAddBall(Sorter.BallColor.GREEN);
                    }
                    if (circlePress) {
                        // Manually add a purple ball
                        sorter.debugAddBall(Sorter.BallColor.PURPLE);
                    }
                    if (rbPress) sorter.resetSequence();
                    if (lbPress) sorter.debugNextSlot();

                    sorter.update();
                    outtake.update();

                    telemetry.addLine("=== FULL SEQUENCE MODE ===");
                    telemetry.addLine("Square=intake  Cross=shoot");
                    telemetry.addLine("Tri=add GREEN  Circle=add PURPLE");
                    telemetry.addLine("LB=next slot  RB=abort");
                    telemetry.addData("Slots", sorter.getSlotsString());
                    telemetry.addData("Ball Count", sorter.getBallCount());
                    telemetry.addData("Shoot State", sorter.getShootState().name());
                    telemetry.addData("Motif", sorter.getMotif().name());
                    telemetry.addData("Turret RPM", "%.0f / %.0f", outtake.getCurrentRPM(), outtake.getTargetRPM());
                    break;
            }

            // Always show sensor data at bottom
            telemetry.addLine("--- Sensors ---");
            telemetry.addData("Color R/G/B", "%d / %d / %d", r, g, b);
            telemetry.addData("Color Total", total);
            telemetry.addData("Ball Detected", ballPresent ? lastDetected : "---");
            telemetry.addData("Drum Encoder", drumPos);
            telemetry.addData("Turret RPM (raw)", "%.0f", turretRPM);
            telemetry.addData("Mode", mode.name());
            telemetry.update();
        }
    }
}
