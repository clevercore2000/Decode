package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.Constants.SorterConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

public class Sorter {

    public enum BallColor { GREEN, PURPLE, EMPTY }

    public enum Motif {
        GPP(new BallColor[]{BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE}),
        PGP(new BallColor[]{BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE}),
        PPG(new BallColor[]{BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN});

        public final BallColor[] sequence;
        Motif(BallColor[] sequence) { this.sequence = sequence; }
    }

    public enum ShootState { IDLE, ROTATING, KICKING, NEXT, DONE }

    private final DcMotorEx drumMotor;
    private final Servo kickServo;
    private final ColorSensor colorSensor;
    private final PIDController positionPID;
    private final Outtake outtake;

    private final BallColor[] slots = {BallColor.EMPTY, BallColor.EMPTY, BallColor.EMPTY};
    private int ballCount = 0;
    private int currentSlotAtTop = 0;
    private Motif motif = Motif.GPP;

    private ShootState shootState = ShootState.IDLE;
    private final int[] shootOrder = new int[3];
    private int shootCount = 0;
    private int shootIndex = 0;
    private final ElapsedTime stateTimer = new ElapsedTime();

    private boolean ballWasPresent = false;
    private double targetPosition = 0;

    public Sorter(Hardware hardware, Outtake outtake) {
        this.drumMotor = hardware.sorterHardware.SorterMotor;
        this.kickServo = hardware.sorterHardware.KickServo;
        this.colorSensor = hardware.sorterHardware.ColorSensor;
        this.outtake = outtake;

        positionPID = new PIDController(
                SorterConstants.POSITION_P,
                SorterConstants.POSITION_I,
                SorterConstants.POSITION_D
        );
        kickServo.setPosition(SorterConstants.RETRACT_POSITION);
    }

    // 21=GPP, 22=PGP, 23=PPG
    public void setMotif(int aprilTagId) {
        switch (aprilTagId) {
            case 22: motif = Motif.PGP; break;
            case 23: motif = Motif.PPG; break;
            default: motif = Motif.GPP; break;
        }
    }

    public void checkIntake() {
        if (ballCount >= 3) return;

        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();
        int total = r + g + b;
        boolean ballPresent = total > SorterConstants.DETECTION_THRESHOLD;

        if (ballPresent && !ballWasPresent) {
            double greenRatio = (double) g / total;
            BallColor detected = (greenRatio > SorterConstants.GREEN_RATIO_THRESHOLD)
                    ? BallColor.GREEN : BallColor.PURPLE;

            slots[currentSlotAtTop] = detected;
            ballCount++;

            if (ballCount < 3) {
                currentSlotAtTop = (currentSlotAtTop + 1) % 3;
                targetPosition = SorterConstants.SLOT_POSITIONS[currentSlotAtTop];
            }
        }
        ballWasPresent = ballPresent;
    }

    public void startShootSequence() {
        if (shootState != ShootState.IDLE) return;

        computeShootOrder();
        if (shootCount == 0) return;
        shootIndex = 0;
        outtake.setTargetRPM(OuttakeConstants.TARGET_RPM);
        targetPosition = SorterConstants.SLOT_POSITIONS[shootOrder[shootIndex]];
        shootState = ShootState.ROTATING;
        stateTimer.reset();
    }

    public void update() {
        switch (shootState) {
            case IDLE:
            case DONE:
                double idlePos = drumMotor.getCurrentPosition();
                if (Math.abs(targetPosition - idlePos) > SorterConstants.POSITION_TOLERANCE) {
                    drumMotor.setPower(positionPID.calculate(targetPosition, idlePos));
                } else {
                    drumMotor.setPower(0);
                }
                break;

            case ROTATING:
                double currentPos = drumMotor.getCurrentPosition();
                drumMotor.setPower(positionPID.calculate(targetPosition, currentPos));

                if (Math.abs(targetPosition - currentPos) < SorterConstants.POSITION_TOLERANCE
                        && outtake.isAtTargetSpeed()) {
                    drumMotor.setPower(0);
                    kickServo.setPosition(SorterConstants.KICK_POSITION);
                    stateTimer.reset();
                    shootState = ShootState.KICKING;
                }
                break;

            case KICKING:
                double elapsed = stateTimer.milliseconds();
                if (elapsed >= SorterConstants.KICK_DELAY_MS) {
                    kickServo.setPosition(SorterConstants.RETRACT_POSITION);
                }
                if (elapsed >= SorterConstants.KICK_DELAY_MS + SorterConstants.FIRE_DELAY_MS) {
                    slots[shootOrder[shootIndex]] = BallColor.EMPTY;
                    ballCount--;
                    shootIndex++;
                    shootState = ShootState.NEXT;
                }
                break;

            case NEXT:
                if (shootIndex >= shootCount) {
                    outtake.stop();
                    shootState = ShootState.DONE;
                } else {
                    targetPosition = SorterConstants.SLOT_POSITIONS[shootOrder[shootIndex]];
                    shootState = ShootState.ROTATING;
                }
                break;
        }
    }

    private void computeShootOrder() {
        shootCount = 0;

        if (ballCount == 3) {
            // Full drum: match motif sequence to slots
            boolean[] used = new boolean[3];
            for (int i = 0; i < 3; i++) {
                BallColor needed = motif.sequence[i];
                for (int j = 0; j < 3; j++) {
                    if (!used[j] && slots[j] == needed) {
                        shootOrder[shootCount++] = j;
                        used[j] = true;
                        break;
                    }
                }
            }
        } else {
            // Partial drum: clockwise order, skip empties
            for (int j = 0; j < 3; j++) {
                if (slots[j] != BallColor.EMPTY) {
                    shootOrder[shootCount++] = j;
                }
            }
        }
    }

    public void resetSequence() {
        shootState = ShootState.IDLE;
        shootIndex = 0;
        drumMotor.setPower(0);
        kickServo.setPosition(SorterConstants.RETRACT_POSITION);
    }

    public void debugNextSlot() {
        currentSlotAtTop = (currentSlotAtTop + 1) % 3;
        targetPosition = SorterConstants.SLOT_POSITIONS[currentSlotAtTop];
    }

    public void debugAddBall(BallColor color) {
        if (ballCount >= 3) return;
        slots[currentSlotAtTop] = color;
        ballCount++;
        if (ballCount < 3) {
            currentSlotAtTop = (currentSlotAtTop + 1) % 3;
            targetPosition = SorterConstants.SLOT_POSITIONS[currentSlotAtTop];
        }
    }

    public ShootState getShootState() { return shootState; }
    public BallColor[] getSlots() { return slots; }
    public int getBallCount() { return ballCount; }
    public Motif getMotif() { return motif; }
    public boolean isSequenceComplete() { return shootState == ShootState.DONE; }
    public boolean isIdle() { return shootState == ShootState.IDLE || shootState == ShootState.DONE; }

    public String getSlotsString() {
        StringBuilder sb = new StringBuilder("[");
        for (int i = 0; i < 3; i++) {
            switch (slots[i]) {
                case GREEN: sb.append("G"); break;
                case PURPLE: sb.append("P"); break;
                case EMPTY: sb.append("_"); break;
            }
            if (i < 2) sb.append("/");
        }
        return sb.append("]").toString();
    }
}
