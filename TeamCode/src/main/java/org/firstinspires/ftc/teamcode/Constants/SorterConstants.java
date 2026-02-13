package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SorterConstants {
    public static final double TICKS_PER_REVOLUTION = 537.7; // gobilda 312 RPM
    public static final double SLOT_TICKS = TICKS_PER_REVOLUTION / 3.0;
    public static final double[] SLOT_POSITIONS = {0, SLOT_TICKS, 2 * SLOT_TICKS};

    public static double POSITION_P = 0.005;
    public static double POSITION_I = 0.0;
    public static double POSITION_D = 0.00;
    public static final double POSITION_TOLERANCE = 10.0;

    public static final double KICK_POSITION = 1;
    public static final double RETRACT_POSITION = 0;
    public static final long KICK_DELAY_MS = 300;
    public static final long FIRE_DELAY_MS = 500;

    public static final int DETECTION_THRESHOLD = 150;
    public static final double GREEN_RATIO_THRESHOLD = 0.4;
    public static final double PURPLE_RED_BLUE_THRESHOLD = 0.65;
}
