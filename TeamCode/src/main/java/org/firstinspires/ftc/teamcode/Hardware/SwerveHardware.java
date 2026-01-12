package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class SwerveHardware {

    private static final String FL_DRIVE = "fl";
    private static final String FR_DRIVE = "fr";
    private static final String BL_DRIVE = "bl";
    private static final String BR_DRIVE = "br";

    private static final String FL_STEER = "fl_servo";
    private static final String FR_STEER = "fr_servo";
    private static final String BL_STEER = "bl_servo";
    private static final String BR_STEER = "br_servo";

    private static final String FL_ENCODER = "fl_enc";
    private static final String FR_ENCODER = "fr_enc";
    private static final String BL_ENCODER = "bl_enc";
    private static final String BR_ENCODER = "br_enc";

    public final DcMotorEx flDrive;
    public final DcMotorEx frDrive;
    public final DcMotorEx blDrive;
    public final DcMotorEx brDrive;

    public final CRServo flSteer;
    public final CRServo frSteer;
    public final CRServo blSteer;
    public final CRServo brSteer;

    public final AnalogInput flEncoder;
    public final AnalogInput frEncoder;
    public final AnalogInput blEncoder;
    public final AnalogInput brEncoder;

    public SwerveHardware(HardwareMap hardwareMap) {
        flDrive = hardwareMap.get(DcMotorEx.class, FL_DRIVE);
        frDrive = hardwareMap.get(DcMotorEx.class, FR_DRIVE);
        blDrive = hardwareMap.get(DcMotorEx.class, BL_DRIVE);
        brDrive = hardwareMap.get(DcMotorEx.class, BR_DRIVE);

        flSteer = hardwareMap.get(CRServo.class, FL_STEER);
        frSteer = hardwareMap.get(CRServo.class, FR_STEER);
        blSteer = hardwareMap.get(CRServo.class, BL_STEER);
        brSteer = hardwareMap.get(CRServo.class, BR_STEER);

        flEncoder = hardwareMap.get(AnalogInput.class, FL_ENCODER);
        frEncoder = hardwareMap.get(AnalogInput.class, FR_ENCODER);
        blEncoder = hardwareMap.get(AnalogInput.class, BL_ENCODER);
        brEncoder = hardwareMap.get(AnalogInput.class, BR_ENCODER);
    }
}
