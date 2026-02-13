package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SorterConstants {
    public static final double TICKS_PER_REVOLUTION = 537.7; // gobilda 312 RPM

    // Position (in ticks) to bring each slot to the intake hole
    public static double INTAKE_POS_0 = -267;
    public static double INTAKE_POS_1 = -440;
    public static double INTAKE_POS_2 = -90;

    // Position (in ticks) to bring each slot to the kick servo
    public static double SHOOT_POS_0 = -537;
    public static double SHOOT_POS_1 = -183;
    public static double SHOOT_POS_2 = -358;

    public static double POSITION_P = 0.005;
    public static double POSITION_I = 0.0;
    public static double POSITION_D = 0.00;
    public static double POSITION_TOLERANCE = 3.0;
    public static double DRUM_MAX_POWER = 0.3;
    public static double SHOOT_COOLDOWN_MS = 500; // keep outtake spinning after last kick

    public static final double KICK_POSITION = 1;
    public static final double RETRACT_POSITION = 0.5;
    public static final double KICK_SERVO_SPEED = 2.0;

    public static double DETECTION_THRESHOLD = 150;
    public static double DISTANCE_THRESHOLD = 5.0; // cm, ball must be closer than this
    public static int DETECTION_FRAMES = 3; // consecutive frames before registering
    public static double GREEN_RATIO_THRESHOLD = 0.4;
}
