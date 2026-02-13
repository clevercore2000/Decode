package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.Constants.SorterConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.PIDController;
import org.firstinspires.ftc.teamcode.Subsystems.Sorter;

@TeleOp(name = "Drum Test", group = "Test")
public class DrumTest extends LinearOpMode {

    enum Mode { INTAKE_POS, SHOOT_POS, SENSOR, FULL_TEST }

    private double getIntakePos(int slot) {
        switch (slot) {
            case 0: return SorterConstants.INTAKE_POS_0;
            case 1: return SorterConstants.INTAKE_POS_1;
            case 2: return SorterConstants.INTAKE_POS_2;
            default: return 0;
        }
    }

    private double getShootPos(int slot) {
        switch (slot) {
            case 0: return SorterConstants.SHOOT_POS_0;
            case 1: return SorterConstants.SHOOT_POS_1;
            case 2: return SorterConstants.SHOOT_POS_2;
            default: return 0;
        }
    }

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

        Mode mode = Mode.INTAKE_POS;
        int currentSlot = 0;
        double drumTarget = 0;
        boolean pidActive = false;

        boolean lastLeft = false, lastRight = false;
        boolean lastTriangle = false, lastCircle = false;
        boolean lastCross = false, lastSquare = false;
        boolean lastRBumper = false, lastLBumper = false;

        telemetry.addLine("== Drum Test ==");
        telemetry.addLine("Dpad L/R: switch mode");
        telemetry.addLine("Tri/Circle: next/prev slot");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean leftPress = gamepad2.dpad_left && !lastLeft;
            boolean rightPress = gamepad2.dpad_right && !lastRight;
            boolean trianglePress = gamepad2.triangle && !lastTriangle;
            boolean circlePress = gamepad2.circle && !lastCircle;
            boolean crossPress = gamepad2.cross && !lastCross;
            boolean squarePress = gamepad2.square && !lastSquare;
            boolean rbPress = gamepad2.right_bumper && !lastRBumper;
            boolean lbPress = gamepad2.left_bumper && !lastLBumper;
            lastLeft = gamepad2.dpad_left;
            lastRight = gamepad2.dpad_right;
            lastTriangle = gamepad2.triangle;
            lastCircle = gamepad2.circle;
            lastCross = gamepad2.cross;
            lastSquare = gamepad2.square;
            lastRBumper = gamepad2.right_bumper;
            lastLBumper = gamepad2.left_bumper;

            if (rightPress) {
                mode = Mode.values()[(mode.ordinal() + 1) % Mode.values().length];
                if (mode == Mode.INTAKE_POS) drumTarget = getIntakePos(currentSlot);
                else if (mode == Mode.SHOOT_POS) drumTarget = getShootPos(currentSlot);
            }
            if (leftPress) {
                mode = Mode.values()[(mode.ordinal() + Mode.values().length - 1) % Mode.values().length];
                if (mode == Mode.INTAKE_POS) drumTarget = getIntakePos(currentSlot);
                else if (mode == Mode.SHOOT_POS) drumTarget = getShootPos(currentSlot);
            }

            int drumPos = hw.sorterHardware.SorterMotor.getCurrentPosition();

            switch (mode) {
                case INTAKE_POS:
                    if (trianglePress) { currentSlot = (currentSlot + 1) % 3; drumTarget = getIntakePos(currentSlot); }
                    if (circlePress) { currentSlot = (currentSlot + 2) % 3; drumTarget = getIntakePos(currentSlot); }
                    if (crossPress) pidActive = !pidActive;

                    if (pidActive) {
                        if (Math.abs(drumTarget - drumPos) > SorterConstants.POSITION_TOLERANCE) {
                            double power = drumPID.calculate(drumTarget, drumPos);
                            power = Math.max(-SorterConstants.DRUM_MAX_POWER, Math.min(power, SorterConstants.DRUM_MAX_POWER));
                            hw.sorterHardware.SorterMotor.setPower(power);
                        } else {
                            hw.sorterHardware.SorterMotor.setPower(0);
                        }
                    } else {
                        hw.sorterHardware.SorterMotor.setPower(0);
                    }

                    telemetry.addLine("=== INTAKE POSITIONS ===");
                    telemetry.addLine("Tri/Circle = next/prev slot");
                    telemetry.addLine("Cross = toggle PID");
                    telemetry.addData("Slot", currentSlot);
                    telemetry.addData("Target", "%.1f", drumTarget);
                    telemetry.addData("Current", drumPos);
                    telemetry.addData("Error", "%.1f", drumTarget - drumPos);
                    telemetry.addData("PID Active", pidActive);
                    telemetry.addLine("--- All Intake Positions ---");
                    telemetry.addData("Slot 0", "%.1f", SorterConstants.INTAKE_POS_0);
                    telemetry.addData("Slot 1", "%.1f", SorterConstants.INTAKE_POS_1);
                    telemetry.addData("Slot 2", "%.1f", SorterConstants.INTAKE_POS_2);
                    break;

                case SHOOT_POS:
                    if (trianglePress) { currentSlot = (currentSlot + 1) % 3; drumTarget = getShootPos(currentSlot); }
                    if (circlePress) { currentSlot = (currentSlot + 2) % 3; drumTarget = getShootPos(currentSlot); }
                    if (crossPress) pidActive = !pidActive;

                    if (pidActive) {
                        if (Math.abs(drumTarget - drumPos) > SorterConstants.POSITION_TOLERANCE) {
                            double power = drumPID.calculate(drumTarget, drumPos);
                            power = Math.max(-SorterConstants.DRUM_MAX_POWER, Math.min(power, SorterConstants.DRUM_MAX_POWER));
                            hw.sorterHardware.SorterMotor.setPower(power);
                        } else {
                            hw.sorterHardware.SorterMotor.setPower(0);
                        }
                    } else {
                        hw.sorterHardware.SorterMotor.setPower(0);
                    }

                    // Hold Square = spin outtake, release = stop
                    if (gamepad2.square) {
                        outtake.setTargetRPM(OuttakeConstants.TARGET_RPM);
                    } else {
                        outtake.stop();
                    }
                    outtake.update();

                    if (rbPress) hw.sorterHardware.KickServo.moveTo(SorterConstants.KICK_POSITION);
                    if (lbPress) hw.sorterHardware.KickServo.moveTo(SorterConstants.RETRACT_POSITION);

                    telemetry.addLine("=== SHOOT POSITIONS ===");
                    telemetry.addLine("Tri/Circle = next/prev slot");
                    telemetry.addLine("Cross = toggle PID");
                    telemetry.addLine("Hold Square = spin outtake");
                    telemetry.addLine("RB = kick  LB = retract");
                    telemetry.addData("Slot", currentSlot);
                    telemetry.addData("Target", "%.1f", drumTarget);
                    telemetry.addData("Current", drumPos);
                    telemetry.addData("Error", "%.1f", drumTarget - drumPos);
                    telemetry.addData("PID Active", pidActive);
                    telemetry.addData("Outtake RPM", "%.0f / %.0f", outtake.getCurrentRPM(), outtake.getTargetRPM());
                    telemetry.addData("At Target?", outtake.isAtTargetSpeed());
                    telemetry.addLine("--- All Shoot Positions ---");
                    telemetry.addData("Slot 0", "%.1f", SorterConstants.SHOOT_POS_0);
                    telemetry.addData("Slot 1", "%.1f", SorterConstants.SHOOT_POS_1);
                    telemetry.addData("Slot 2", "%.1f", SorterConstants.SHOOT_POS_2);
                    break;

                case SENSOR:
                    // Run intake motor with Square
                    hw.intakeHardware.IntakeMotor.setPower(gamepad2.square ? 0.9 : 0);

                    // Read sensor live
                    int sR = hw.sorterHardware.ColorSensor.red();
                    int sG = hw.sorterHardware.ColorSensor.green();
                    int sB = hw.sorterHardware.ColorSensor.blue();
                    int sTotal = sR + sG + sB;
                    double sDist = hw.sorterHardware.DistanceSensor.getDistance(DistanceUnit.CM);

                    boolean distOk = sDist < SorterConstants.DISTANCE_THRESHOLD;
                    boolean colorOk = sTotal > SorterConstants.DETECTION_THRESHOLD;
                    boolean isBall = distOk && colorOk;

                    String colorGuess = "---";
                    if (isBall && sTotal > 0) {
                        double gRatio = (double) sG / sTotal;
                        colorGuess = (gRatio > SorterConstants.GREEN_RATIO_THRESHOLD) ? "GREEN" : "PURPLE";
                    }

                    telemetry.addLine("=== SENSOR TEST ===");
                    telemetry.addLine("Hold Square = run intake");
                    telemetry.addLine("");
                    telemetry.addData("Distance (cm)", "%.1f", sDist);
                    telemetry.addData("Dist Threshold", "%.1f", SorterConstants.DISTANCE_THRESHOLD);
                    telemetry.addData("Dist OK?", distOk);
                    telemetry.addLine("");
                    telemetry.addData("R / G / B", "%d / %d / %d", sR, sG, sB);
                    telemetry.addData("Total", sTotal);
                    telemetry.addData("Color Threshold", "%.0f", SorterConstants.DETECTION_THRESHOLD);
                    telemetry.addData("Color OK?", colorOk);
                    telemetry.addLine("");
                    if (sTotal > 0) {
                        telemetry.addData("Green Ratio", "%.3f", (double) sG / sTotal);
                        telemetry.addData("Red Ratio", "%.3f", (double) sR / sTotal);
                        telemetry.addData("Blue Ratio", "%.3f", (double) sB / sTotal);
                    }
                    telemetry.addData("Green Thresh", "%.3f", SorterConstants.GREEN_RATIO_THRESHOLD);
                    telemetry.addLine("");
                    telemetry.addData(">> BALL?", isBall ? "YES - " + colorGuess : "NO");
                    telemetry.addData("Det Frames needed", SorterConstants.DETECTION_FRAMES);
                    break;

                case FULL_TEST:
                    hw.intakeHardware.IntakeMotor.setPower(gamepad2.square ? 0.9 : 0);
                    sorter.checkIntake();

                    if (crossPress && sorter.isIdle()) sorter.startShootSequence();
                    if (trianglePress) sorter.debugAddBall(Sorter.BallColor.GREEN);
                    if (circlePress) sorter.debugAddBall(Sorter.BallColor.PURPLE);
                    if (rbPress) sorter.resetSequence();
                    if (lbPress) sorter.debugNextSlot();

                    sorter.update();
                    outtake.update();

                    telemetry.addLine("=== FULL TEST ===");
                    telemetry.addLine("Square=intake  Cross=shoot");
                    telemetry.addLine("Tri=add GREEN  Circle=add PURPLE");
                    telemetry.addLine("LB=next slot  RB=abort");
                    telemetry.addData("Slots", sorter.getSlotsString());
                    telemetry.addData("Balls", sorter.getBallCount());
                    telemetry.addData("Shoot State", sorter.getShootState().name());
                    telemetry.addData("Drum Target", "%.1f", sorter.getTargetPosition());
                    telemetry.addData("Drum Encoder", sorter.getDrumEncoder());
                    telemetry.addData("Turret RPM", "%.0f / %.0f", outtake.getCurrentRPM(), outtake.getTargetRPM());
                    telemetry.addData("Color R/G/B", "%d / %d / %d", sorter.getColorR(), sorter.getColorG(), sorter.getColorB());
                    telemetry.addData("Color Total", sorter.getColorTotal());
                    break;
            }

            hw.sorterHardware.KickServo.execute();

            telemetry.addLine("--- Sensor ---");
            int r = hw.sorterHardware.ColorSensor.red();
            int g = hw.sorterHardware.ColorSensor.green();
            int b = hw.sorterHardware.ColorSensor.blue();
            int total = r + g + b;
            telemetry.addData("R/G/B", "%d / %d / %d", r, g, b);
            telemetry.addData("Total", total);
            telemetry.addData("Drum Encoder", drumPos);
            telemetry.addData("Mode", mode.name());
            telemetry.update();
        }
    }
}
