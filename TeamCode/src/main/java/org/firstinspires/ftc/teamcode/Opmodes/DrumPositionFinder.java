package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants.SorterConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

@TeleOp(name = "Drum Position Finder", group = "Test")
public class DrumPositionFinder extends LinearOpMode {

    // 6 steps: intake for slots 0,1,2 then shoot for slots 0,1,2
    private static final String[] STEP_NAMES = {
        "Slot 0 → INTAKE", "Slot 1 → INTAKE", "Slot 2 → INTAKE",
        "Slot 0 → KICK",   "Slot 1 → KICK",   "Slot 2 → KICK"
    };

    @Override
    public void runOpMode() {
        Hardware hw = new Hardware(hardwareMap);

        hw.sorterHardware.SorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hw.sorterHardware.SorterMotor.setPower(0);

        int step = 0;
        double[] saved = new double[6];

        boolean lastCross = false;
        boolean lastSquare = false;

        telemetry.addLine("== Drum Position Finder ==");
        telemetry.addLine("Jog each slot to intake/kick");
        telemetry.addLine("Cross = save, Square = reset encoder");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean crossPress = gamepad2.cross && !lastCross;
            boolean squarePress = gamepad2.square && !lastSquare;
            lastCross = gamepad2.cross;
            lastSquare = gamepad2.square;

            if (gamepad2.triangle) {
                hw.sorterHardware.SorterMotor.setPower(0.15);
            } else if (gamepad2.circle) {
                hw.sorterHardware.SorterMotor.setPower(-0.15);
            } else {
                hw.sorterHardware.SorterMotor.setPower(0);
            }

            if (squarePress) {
                hw.sorterHardware.SorterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                hw.sorterHardware.SorterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                hw.sorterHardware.SorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            int pos = hw.sorterHardware.SorterMotor.getCurrentPosition();

            if (step < 6) {
                if (crossPress) {
                    saved[step] = pos;
                    step++;
                }

                telemetry.addLine("=== STEP " + (step + 1) + " / 6 ===");
                if (step < 6) {
                    telemetry.addLine("Jog " + STEP_NAMES[step]);
                    telemetry.addLine("Cross = SAVE");
                }
            }

            if (step >= 6) {
                if (crossPress) step = 0; // restart

                telemetry.addLine("=== DONE - Set in SorterConstants ===");
                telemetry.addData(">> INTAKE_POS_0", "%.1f", saved[0] % SorterConstants.TICKS_PER_REVOLUTION);
                telemetry.addData(">> INTAKE_POS_1", "%.1f", saved[1] % SorterConstants.TICKS_PER_REVOLUTION);
                telemetry.addData(">> INTAKE_POS_2", "%.1f", saved[2] % SorterConstants.TICKS_PER_REVOLUTION);
                telemetry.addData(">> SHOOT_POS_0", "%.1f", saved[3] % SorterConstants.TICKS_PER_REVOLUTION);
                telemetry.addData(">> SHOOT_POS_1", "%.1f", saved[4] % SorterConstants.TICKS_PER_REVOLUTION);
                telemetry.addData(">> SHOOT_POS_2", "%.1f", saved[5] % SorterConstants.TICKS_PER_REVOLUTION);
                telemetry.addLine("");
                telemetry.addLine("Cross = restart");
            }

            // Show saved so far
            for (int i = 0; i < step && i < 6; i++) {
                telemetry.addData(STEP_NAMES[i], "%.1f (raw: %.1f)", saved[i] % SorterConstants.TICKS_PER_REVOLUTION, saved[i]);
            }

            telemetry.addLine("--- ---");
            telemetry.addData("Encoder", pos);
            telemetry.addData("Pos in rev", "%.1f", pos % SorterConstants.TICKS_PER_REVOLUTION);
            telemetry.addLine("Tri=jog fwd  Circle=jog rev  Square=zero");
            telemetry.update();
        }
    }
}
