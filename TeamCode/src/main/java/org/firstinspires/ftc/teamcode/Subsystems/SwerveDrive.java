package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Constants.SteeringConstants;
import org.firstinspires.ftc.teamcode.Hardware.AxonEncoder;

public class SwerveDrive {
    private final SwerveModule fl, fr, bl, br;
    private final SwerveDriveKinematics kinematics;

    public SwerveDrive(HardwareMap hardwareMap) {
        // Initialize kinematics with module positions (FL, FR, BL, BR order)
        kinematics = new SwerveDriveKinematics(
                DriveConstants.FL_POSITION,
                DriveConstants.FR_POSITION,
                DriveConstants.BL_POSITION,
                DriveConstants.BR_POSITION
        );

        // Initialize all 4 modules
        fl = new SwerveModule(
                hardwareMap.get(DcMotorEx.class, "fl"),
                hardwareMap.get(CRServo.class, "fl_servo"),
                new AxonEncoder(hardwareMap.get(AnalogInput.class, "fl_enc"), SteeringConstants.FL_VOLTAGE_OFFSET),
                true, true, "FL"  // Drive NOT inverted - all motors same direction
        );

        fr = new SwerveModule(
                hardwareMap.get(DcMotorEx.class, "fr"),
                hardwareMap.get(CRServo.class, "fr_servo"),
                new AxonEncoder(hardwareMap.get(AnalogInput.class, "fr_enc"), SteeringConstants.FR_VOLTAGE_OFFSET),
                false, true, "FR"  // Drive NOT inverted - all motors same direction
        );

        bl = new SwerveModule(
                hardwareMap.get(DcMotorEx.class, "bl"),
                hardwareMap.get(CRServo.class, "bl_servo"),
                new AxonEncoder(hardwareMap.get(AnalogInput.class, "bl_enc"), SteeringConstants.BL_VOLTAGE_OFFSET),
                false, true, "BL"
        );

        br = new SwerveModule(
                hardwareMap.get(DcMotorEx.class, "br"),
                hardwareMap.get(CRServo.class, "br_servo"),
                new AxonEncoder(hardwareMap.get(AnalogInput.class, "br_enc"), SteeringConstants.BR_VOLTAGE_OFFSET),
                false, true, "BR"
        );
    }

    public void drive(double xSpeed, double ySpeed, double rotSpeed) {
        drive(xSpeed, ySpeed, rotSpeed, true); // Default to optimizing
    }

    public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean optimize) {
        // Robot-centric control only
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);

        fl.setDesiredState(moduleStates[0], optimize);
        fr.setDesiredState(moduleStates[1], optimize);
        bl.setDesiredState(moduleStates[2], optimize);
        br.setDesiredState(moduleStates[3], optimize);
    }

    public void hold() {
        fl.hold();
        fr.hold();
        bl.hold();
        br.hold();
    }

    public void addTelemetry(Telemetry telemetry) {
        fl.addTelemetry(telemetry);
        fr.addTelemetry(telemetry);
        bl.addTelemetry(telemetry);
        br.addTelemetry(telemetry);
    }
}
