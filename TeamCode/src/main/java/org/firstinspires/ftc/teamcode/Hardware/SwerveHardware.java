package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

/**
 * Hardware abstraction for swerve drive system
 * Initializes and stores references to all physical devices
 *
 * Hardware Configuration Names:
 * Device names must match Robot Controller configuration exactly (case-sensitive)
 */
public class SwerveHardware {

    // ========== HARDWARE DEVICE NAMES ==========
    // Drive motors (DcMotorEx)
    private static final String FL_DRIVE = "fl";
    private static final String FR_DRIVE = "fr";
    private static final String BL_DRIVE = "bl";
    private static final String BR_DRIVE = "br";

    // Steering servos (CRServo)
    private static final String FL_STEER = "fl_servo";
    private static final String FR_STEER = "fr_servo";
    private static final String BL_STEER = "bl_servo";
    private static final String BR_STEER = "br_servo";

    // Steering encoders (AnalogInput)
    private static final String FL_ENCODER = "fl_enc";
    private static final String FR_ENCODER = "fr_enc";
    private static final String BL_ENCODER = "bl_enc";
    private static final String BR_ENCODER = "br_enc";

    // IMU (built into Control Hub) - Uncomment for field-centric drive
    // private static final String IMU_NAME = "imu";

    // ========== HARDWARE DEVICES ==========

    // Drive motors
    public final DcMotorEx flDrive;
    public final DcMotorEx frDrive;
    public final DcMotorEx blDrive;
    public final DcMotorEx brDrive;

    // Steering servos
    public final CRServo flSteer;
    public final CRServo frSteer;
    public final CRServo blSteer;
    public final CRServo brSteer;

    // Steering encoders
    public final AnalogInput flEncoder;
    public final AnalogInput frEncoder;
    public final AnalogInput blEncoder;
    public final AnalogInput brEncoder;

    // IMU - Uncomment for field-centric drive
    // public final IMU imu;

    public SwerveHardware(HardwareMap hardwareMap) {
        // Initialize drive motors
        flDrive = hardwareMap.get(DcMotorEx.class, FL_DRIVE);
        frDrive = hardwareMap.get(DcMotorEx.class, FR_DRIVE);
        blDrive = hardwareMap.get(DcMotorEx.class, BL_DRIVE);
        brDrive = hardwareMap.get(DcMotorEx.class, BR_DRIVE);

        // Initialize steering servos
        flSteer = hardwareMap.get(CRServo.class, FL_STEER);
        frSteer = hardwareMap.get(CRServo.class, FR_STEER);
        blSteer = hardwareMap.get(CRServo.class, BL_STEER);
        brSteer = hardwareMap.get(CRServo.class, BR_STEER);

        // Initialize steering encoders
        flEncoder = hardwareMap.get(AnalogInput.class, FL_ENCODER);
        frEncoder = hardwareMap.get(AnalogInput.class, FR_ENCODER);
        blEncoder = hardwareMap.get(AnalogInput.class, BL_ENCODER);
        brEncoder = hardwareMap.get(AnalogInput.class, BR_ENCODER);

        // Initialize IMU - Uncomment for field-centric drive
        /*
        imu = hardwareMap.get(IMU.class, IMU_NAME);

        // Configure IMU orientation (adjust for Control Hub mounting)
        IMU.Parameters imuParameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        );
        imu.initialize(imuParameters);
        */
    }
}
