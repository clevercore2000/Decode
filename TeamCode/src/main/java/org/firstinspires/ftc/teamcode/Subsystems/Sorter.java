package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.Constants.OuttakeConstants;
import org.firstinspires.ftc.teamcode.Constants.SorterConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.ServoCfg;

public class Sorter {

    public enum BallColor { GREEN, PURPLE, EMPTY }

    public enum ShootState { IDLE, SPIN_UP, ROTATING, KICKING, RETRACTING, COOLDOWN }

    private final DcMotorEx drumMotor;
    private final ServoCfg kickServo;
    private final ColorSensor colorSensor;
    private final DistanceSensor distanceSensor;
    private final PIDController positionPID;
    private final Outtake outtake;

    private final BallColor[] slots = {BallColor.EMPTY, BallColor.EMPTY, BallColor.EMPTY};
    private int ballCount = 0;
    private int slotAtIntake = 0;
    private int greenFireIndex = 0; // 0=green fires 1st, 1=2nd, 2=3rd

    private ShootState shootState = ShootState.IDLE;
    private final int[] shootOrder = new int[3];
    private int shootCount = 0;
    private int shootIndex = 0;

    private boolean ballWasPresent = false;
    private int detectionCount = 0; // sustained reading counter
    private int detectedR, detectedG, detectedB; // accumulated color from first detection
    private double targetPosition = 0;
    private final ElapsedTime cooldownTimer = new ElapsedTime();
    private int lastR, lastG, lastB, lastTotal;
    private double lastDistance;


    public Sorter(Hardware hardware, Outtake outtake) {
        this.drumMotor = hardware.sorterHardware.SorterMotor;
        this.kickServo = hardware.sorterHardware.KickServo;
        this.colorSensor = hardware.sorterHardware.ColorSensor;
        this.distanceSensor = hardware.sorterHardware.DistanceSensor;
        this.outtake = outtake;

        positionPID = new PIDController(
                SorterConstants.POSITION_P,
                SorterConstants.POSITION_I,
                SorterConstants.POSITION_D
        );
        kickServo.moveTo(SorterConstants.RETRACT_POSITION);

        // Initial Preload State: Slot 0 = Green, 1 & 2 = Purple
        slots[0] = BallColor.GREEN;
        slots[1] = BallColor.PURPLE;
        slots[2] = BallColor.PURPLE;
        ballCount = 3;

        slotAtIntake = 0;
        targetPosition = getIntakePos(slotAtIntake);

        colorSensor.enableLed(false); // Start with light off
    }

    // 21=green 1st, 22=green 2nd, 23=green 3rd
    public void setMotif(int aprilTagId) {
        switch (aprilTagId) {
            case 22: greenFireIndex = 1; break;
            case 23: greenFireIndex = 2; break;
            default: greenFireIndex = 0; break;
        }
    }

    public void checkIntake() {
        // Logic Gate: Disable sensing if we are full or shooting
        if (ballCount >= 3 || shootState != ShootState.IDLE) {
            lastR = 0; lastG = 0; lastB = 0; lastTotal = 0;
            lastDistance = 999.0;
            detectionCount = 0;
            return;
        }

        double currentPos = drumMotor.getCurrentPosition();
        double error = Math.abs(targetPosition - currentPos);

        // Software Blinding during drum movement
        if (error > SorterConstants.LIGHT_TOGGLE_THRESHOLD) {
            lastR = 0; lastG = 0; lastB = 0; lastTotal = 0;
            lastDistance = 999.0;
            detectionCount = 0;
            return;
        }

        // Hardware Reading
        lastR = colorSensor.red();
        lastG = colorSensor.green();
        lastB = colorSensor.blue();
        lastTotal = lastR + lastG + lastB;
        lastDistance = distanceSensor.getDistance(DistanceUnit.CM);

        boolean ballPresent = lastDistance < SorterConstants.DISTANCE_THRESHOLD
                && lastTotal > SorterConstants.DETECTION_THRESHOLD;

        if (ballPresent) {
            if (detectionCount == 0) {
                detectedR = lastR; detectedG = lastG; detectedB = lastB;
            }
            detectionCount++;
        } else {
            detectionCount = 0;
        }

        if (detectionCount >= SorterConstants.DETECTION_FRAMES && !ballWasPresent) {
            int total = detectedR + detectedG + detectedB;
            double greenRatio = (double) detectedG / (total == 0 ? 1 : total);

            slots[slotAtIntake] = (greenRatio > SorterConstants.GREEN_RATIO_THRESHOLD)
                    ? BallColor.GREEN : BallColor.PURPLE;

            ballCount++;
            if (ballCount < 3) {
                slotAtIntake = (slotAtIntake + 1) % 3;
                targetPosition = getIntakePos(slotAtIntake);
            }
            ballWasPresent = true;
        }

        if (!ballPresent) ballWasPresent = false;
    }

    public void startShootSequence() {
        if (shootState != ShootState.IDLE) return;

        computeShootOrder();
        shootIndex = 0;
        outtake.setTargetRPM(OuttakeConstants.TARGET_RPM);
        shootState = ShootState.SPIN_UP;
    }
    public void update() {
        kickServo.execute();
        double currentPos = drumMotor.getCurrentPosition();

        switch (shootState) {
            case IDLE:
                driveDrumToTarget();
                break;

            case SPIN_UP:
                if (outtake.isAtTargetSpeed()) {
                    targetPosition = getShootPos(shootOrder[shootIndex]);
                    shootState = ShootState.ROTATING;
                }
                break;

            case ROTATING:
                if (kickServo.notReady()) break;
                double rotPower = positionPID.calculate(targetPosition, currentPos);
                rotPower = Math.max(-SorterConstants.DRUM_MAX_POWER, Math.min(rotPower, SorterConstants.DRUM_MAX_POWER));
                drumMotor.setPower(rotPower);

                if (Math.abs(targetPosition - currentPos) < SorterConstants.POSITION_TOLERANCE) {
                    drumMotor.setPower(0);
                    kickServo.moveTo(SorterConstants.KICK_POSITION);
                    shootState = ShootState.KICKING;
                }
                break;

            case KICKING:
                if (kickServo.notReady()) break;
                kickServo.moveTo(SorterConstants.RETRACT_POSITION);
                shootState = ShootState.RETRACTING;
                break;

            case RETRACTING:
                if (kickServo.notReady()) break;
                slots[shootOrder[shootIndex]] = BallColor.EMPTY;
                if (ballCount > 0) ballCount--;
                shootIndex++;

                if (shootIndex >= shootCount) {
                    cooldownTimer.reset();
                    shootState = ShootState.COOLDOWN;
                } else {
                    targetPosition = getShootPos(shootOrder[shootIndex]);
                    shootState = ShootState.ROTATING;
                }
                break;

            case COOLDOWN:
                if (cooldownTimer.milliseconds() >= (SorterConstants.SHOOT_COOLDOWN_MS + 500)) {
                    outtake.stop();

                    // Reset drum for next intake cycle
                    for (int i = 0; i < 3; i++) slots[i] = BallColor.EMPTY;
                    ballCount = 0;
                    slotAtIntake = 0;

                    targetPosition = getIntakePos(slotAtIntake);
                    shootState = ShootState.IDLE;
                }
                break;
        }
    }
    private void driveDrumToTarget() {
        double pos = drumMotor.getCurrentPosition();
        if (Math.abs(targetPosition - pos) > SorterConstants.POSITION_TOLERANCE) {
            double power = positionPID.calculate(targetPosition, pos);
            power = Math.max(-SorterConstants.DRUM_MAX_POWER, Math.min(power, SorterConstants.DRUM_MAX_POWER));
            drumMotor.setPower(power);
        } else {
            drumMotor.setPower(0);
        }
    }

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

    private void computeShootOrder() {
        int greenSlot = -1;
        for (int i = 0; i < 3; i++) {
            if (slots[i] == BallColor.GREEN) { greenSlot = i; break; }
        }

        int targetGreen = (greenSlot != -1) ? greenSlot : 0;
        int otherIdx = 0;

        for (int i = 0; i < 3; i++) {
            if (i == greenFireIndex) {
                shootOrder[i] = targetGreen;
            } else {
                while (otherIdx == targetGreen) otherIdx++;
                shootOrder[i] = otherIdx++;
            }
        }
        shootCount = 3;
    }

    public void resetSequence() {
        shootState = ShootState.IDLE;
        shootIndex = 0;
        drumMotor.setPower(0);
        kickServo.moveTo(SorterConstants.RETRACT_POSITION);
    }

    public void debugNextSlot() {
        slotAtIntake = (slotAtIntake + 1) % 3;
        targetPosition = getIntakePos(slotAtIntake);
    }

    public void debugAddBall(BallColor color) {
        if (ballCount >= 3) return;
        slots[slotAtIntake] = color;
        ballCount++;
        if (ballCount < 3) {
            slotAtIntake = (slotAtIntake + 1) % 3;
            targetPosition = getIntakePos(slotAtIntake);
        }
    }

    public ShootState getShootState() { return shootState; }
    public BallColor[] getSlots() { return slots; }
    public int getBallCount() { return ballCount; }
    public int getGreenFireIndex() { return greenFireIndex; }
    public boolean isIdle() { return shootState == ShootState.IDLE; }
    public double getTargetPosition() { return targetPosition; }
    public int getColorR() { return lastR; }
    public int getColorG() { return lastG; }
    public int getColorB() { return lastB; }
    public int getColorTotal() { return lastTotal; }
    public double getDistance() { return lastDistance; }
    public int getDetectionCount() { return detectionCount; }
    public int getDrumEncoder() { return drumMotor.getCurrentPosition(); }

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
