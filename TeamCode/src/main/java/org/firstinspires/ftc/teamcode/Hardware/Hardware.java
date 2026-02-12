package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hardware {
    public OuttakeHardware outtakeHardware;
    public IntakeHardware intakeHardware;
    public SorterHardware sorterHardware;

    public Hardware(HardwareMap hardwareMap) {
        outtakeHardware = new OuttakeHardware(hardwareMap);
        intakeHardware = new IntakeHardware(hardwareMap);
        sorterHardware = new SorterHardware(hardwareMap);
    }
}
