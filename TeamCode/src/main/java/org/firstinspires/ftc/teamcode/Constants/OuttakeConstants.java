package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class OuttakeConstants {
    public static final double MAX_MOTOR_RPM = 6000.0;
    public static final double MOTOR_TICKS_PER_REV = 22;

    public static double TARGET_RPM = 4100;

    public static double VELOCITY_P = 0.0002;
    public static double VELOCITY_I = 0.00001;
    public static double VELOCITY_D = 0.00001;
    public static double VELOCITY_FF = 1.0 / ((MAX_MOTOR_RPM / 60.0) * MOTOR_TICKS_PER_REV);
    public static double MAX_INTEGRAL = 0.1;

    public static double RPM_TOLERANCE = 20;
    public static double MIN_POWER = 0.05;
}
