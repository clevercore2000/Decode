# CLAUDE.md

Guidance for Claude Code working with this FTC Robot Controller codebase.

## Project Overview

FTC SDK v10.3 for DECODE (2025-2026) season. Implements **4-module coaxial swerve drive** with:
- **FTCLib v2.1.1**: WPILib-style kinematics
- **Pedro Pathing v1.0.8**: Autonomous navigation
- **FTC Dashboard v0.4.17**: Runtime tuning

## Build Commands

- `./gradlew build` - Build APK
- `./gradlew clean` - Clean build
- `./gradlew installDebug` - Install to device
- Requirements: Android Studio Ladybug (2024.2)+, Java 8+, Android SDK API 24+/target 28

## Architecture

### Directory Structure
```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── Hardware/
│   ├── Hardware.java              # Central hardware manager
│   └── SwerveHardware.java        # (unused - integrated into SwerveDrive)
├── Subsystems/
│   ├── SwerveDrive.java           # 4-module coordinator + kinematics
│   ├── SwerveModule.java          # Individual module (motor + servo + encoder)
│   └── PIDController.java         # Custom PID
├── Opmodes/
│   ├── SwerveTeleop.java          # Driver control
│   ├── SwerveCalibration.java    # ⚠️ Has encoder power bug (see Known Issues)
│   └── SwerveModuleTest.java      # Module testing
└── Constants/
    └── SwerveConstants.java       # All configuration parameters
```

### Hardware Configuration Names
```
Drive Motors:     fl, fr, bl, br              (DcMotorEx)
Steering Servos:  fl_servo, fr_servo, etc.   (CRServo)
Analog Encoders:  fl_enc, fr_enc, etc.       (AnalogInput)
IMU:              imu
```

### Control Flow
```
Joystick → ChassisSpeeds → SwerveDriveKinematics →
SwerveModuleStates (optimized) → PID → Hardware
```

## Hardware Details

### Coaxial Swerve Module
- **Drive**: DC motor with encoder (e.g., GoBILDA 312 RPM)
- **Steering**: Axon Mini MK1/MK2 servo in CRServo mode
- **Position**: Analog encoder via 4th wire (0-3.3V = 0-360°)
- **Gear Ratio**: 2:1 (servo rotates 2×, steering rotates 1×)
- **State Optimization**: Modules never rotate >90° (reverse drive instead)

### Axon Mini Servo - CRITICAL INFO

**⚠️ Power Dependency (CRITICAL):**
- Analog encoder REQUIRES servo electronics active (PWM signals) to output voltage
- `setPower(0.0)` activates electronics without movement
- Unpowered servos = 0V encoder reading = invalid calibration

**CRServo Mode:**
- Unlimited rotation (>360°)
- Controlled via `setPower(-1.0 to 1.0)`
- Position read via `AnalogInput.getVoltage()` (0-3.3V)

**Connection:**
- Requires Analog-JST Board (REV-31-1535)
- Connect to Control Hub analog ports (dual-channel: 0-1 and 2-3 is NORMAL)

### REV Control Hub Analog Inputs
- **4 ports**, **2 channels each** = 8 analog inputs total
- **Port numbering**: 0-1 and 2-3 (paired channels on one physical connector)
- **Voltage**: 0-5V input range, 3.3V power output
- **Wiring**: Black=GND, Red=+3.3V, Blue/White=Signal
- Code: `AnalogInput encoder = hardwareMap.get(AnalogInput.class, "fl_enc");`

## Calibration

### ⚠️ CRITICAL BUG IN SwerveCalibration.java

**Issue**: Servos unpowered during measurement → encoders read 0V → invalid offsets → broken control

**Fix**: Send `setPower(0.0)` to all servos before reading encoders:
```java
// Instead of: hardware.swerveDrive.stop();
for (int i = 0; i < 4; i++) {
    CRServo servo = hardware.swerveDrive.getModule(i).steerServo;
    servo.setPower(0.0);  // Activates electronics without movement
}
```

### Calibration Procedure

**Current Method (Manual Alignment):**
1. Run `SwerveCalibration` OpMode
2. **CRITICAL**: Code must send `setPower(0.0)` to keep servos powered
3. Manually align wheels straight forward
4. Verify voltages are 0.3-3.0V (NOT 0.0V)
5. Copy values to `SteeringConstants.java`:
   ```java
   public static double FL_ANGLE_OFFSET = <value>;  // radians
   ```
6. Re-deploy and verify

**Recommended Method (Not Yet Implemented):**
- Active calibration: servos automatically rotate to 0°, then read encoders
- Eliminates manual alignment errors
- Based on FRC best practices (REV MAXSwerve, SDS modules)

**Between Matches:**
- NO recalibration needed (absolute encoders retain position)
- NO homing procedure needed
- Modules can start in any position - PID rotates to target

## Troubleshooting

### Encoder Issues

| Symptom | Cause | Fix |
|---------|-------|-----|
| All angles read 0° | Encoders unpowered | Send `setPower(0.0)` to servos |
| Voltages show 0.0V | Servo electronics not active | Verify PWM signals sent |
| EncoderTest works, Calibration frozen | Different code paths | Use `EncoderTest.java` to verify hardware |
| Modules rotate to wrong angles | Bad calibration offsets | Re-run calibration with servos powered |

### Control Issues

**Servos jitter/erratic:**
1. **PID too low**: Increase `STEER_P` from 0.01 to 1.0 ✅ FIXED
2. **PID too high**: Decrease `STEER_P`, increase `STEER_D` to 0.05
3. **Bad offsets**: Re-calibrate

**Robot drifts:**
1. Increase `JOYSTICK_DEADBAND` to 0.05 ✅ FIXED
2. Check gamepad calibration

**Robot doesn't move:**
1. Check `DRIVE_P` and `DRIVE_FF` in DriveConstants
2. Verify motor inversions in `SwerveDrive.java`
3. Check `MAX_SPEED_METERS_PER_SECOND`

### Hardware Configuration

**Error: "Unable to find hardware device"**
- Names in code must EXACTLY match Robot Controller configuration
- Case-sensitive: `fl`, `fr`, `bl`, `br` (motors), `fl_servo` (servos), `fl_enc` (encoders)

**Encoders paired in ports 0:1 and 2:3**
- ✓ This is NORMAL - dual-channel analog ports (2 sensors per physical port)

## Configuration Checklist

**Before First Drive:**

Robot Dimensions (`SwerveConstants.java`):
- [ ] `WHEELBASE_METERS`, `TRACK_WIDTH_METERS`, `WHEEL_DIAMETER_METERS`

Motor Config:
- [ ] `DRIVE_GEAR_RATIO`, `MOTOR_TICKS_PER_REV`
- [ ] Motor inversions in `SwerveDrive.java`

Hardware Names (match Robot Configuration exactly):
- [ ] Motors: `fl`, `fr`, `bl`, `br`
- [ ] Servos: `fl_servo`, `fr_servo`, `bl_servo`, `br_servo`
- [ ] Encoders: `fl_enc`, `fr_enc`, `bl_enc`, `br_enc`

Encoder Verification:
- [ ] Run `EncoderTest.java` - verify voltages 0.3-3.0V (NOT 0.0V)
- [ ] Voltages change when rotating wheels manually

Calibration:
- [ ] Run `SwerveCalibration` with servos POWERED
- [ ] Set offsets in `SteeringConstants.java`
- [ ] Verify angles read ~0° when wheels forward

PID Tuning (FTC Dashboard @ `http://192.168.43.1:8080/dash`):
- [ ] `STEER_P = 1.0` (was 0.01 - CRITICAL fix)
- [ ] `STEER_D = 0.05`
- [ ] `JOYSTICK_DEADBAND = 0.05` (was 0 - fix applied)

Verification:
- [ ] Run `SwerveModuleTest.java` - verify each module rotates to 0°, 90°, 180°, 270°
- [ ] No random servo movement

## Architecture Redesign (2025-01-22)

### **Critical Discovery: External PID vs Axon Internal PID Conflict**

**Problem**: Axon servos have built-in PID controllers (pre-tuned, factory set). Adding external PID creates:
- Fighting control loops (external PID vs servo internal PID)
- Erratic, random servo behavior
- Some modules work, some don't (unpredictable oscillations)

**Research Sources**:
- Chief Delphi forums: "P-only controllers oscillate as kP increases"
- FTC community: "Axon servos tricky to control due to control system constraints"
- Axon docs: "Dampening Factor shouldn't be modified - pre-tuned"
- Working implementations: Use simple proportional control only

### **New Approach: Simple Proportional Control**

**Steering Control**:
```java
// OLD (Complex - caused erratic behavior):
double output = PID.calculate(target, current)  // External PID
servoPower = output                              // Fights internal PID

// NEW (Simple - proven to work):
double error = normalizeAngle(target - current)
double servoPower = error × STEER_KP (0.05)     // Just give direction
// Let Axon internal PID do the stabilization
```

**Drive Control**:
```java
// OLD: Velocity PID + Feedforward
// NEW: Simple feedforward only
drivePower = targetSpeed / maxSpeed
```

**Key Parameters** (SteeringConstants.java):
- `STEER_KP = 0.05` (simple proportional, very small gain)
- `ENCODER_FILTER_ALPHA = 0.95` (minimal filtering for low lag)
- `STEERING_DEADBAND = 0.05 rad` (prevents jitter near target)

### **Files Changed**:
- ✅ `SwerveModule.java` - Completely rebuilt with simple control
- ✅ `SteeringConstants.java` - Removed PID constants, added STEER_KP
- ⚠️ `PIDController.java` - No longer used (kept for reference)

## Known Issues

### #1: Previous Complex PID Approach 🔴 CAUSED ERRATIC BEHAVIOR

**Issue**: External PID controller fighting with Axon servo internal PID

**Symptoms**:
- Servos behave erratically
- Some modules work, some don't (random)
- Wild oscillations or extremely slow response

**Fix**: Removed external PID, using simple proportional control

**Status**: Fixed with architecture redesign

---

### #2: Encoder Reads Wheel Angle Directly ✅ VERIFIED

**Issue**: Confusion about gear ratio application

**Truth**: Analog encoder measures **wheel output shaft** (not servo shaft)
- 0-3.3V = 0-355° wheel rotation
- ~5° dead zone where voltage jumps 3.3V → 0V
- NO gear ratio correction needed in software

**Status**: Verified - encoder conversion is correct

## Development Workflow

**Testing Sequence:**
1. Calibration: `SwerveCalibration` → set offsets
2. Module Test: `SwerveModuleTest` → verify individual modules
3. Teleop: `SwerveTeleop` → driver control
4. Tuning: FTC Dashboard → adjust STEER_KP if needed

**Tuning the New Simple Control** (FTC Dashboard @ http://192.168.43.1:8080/dash):

Start with `STEER_KP = 0.05`:
- **Too slow to respond?** Increase STEER_KP by 0.01 increments (try 0.06, 0.07, etc.)
- **Servos oscillate/jitter?** Decrease STEER_KP by 0.01 increments (try 0.04, 0.03, etc.)
- **Still jittery when stopped?** Increase `STEERING_DEADBAND_RADIANS` to 0.1

**Expected behavior**:
- Modules should smoothly rotate to target angles
- No wild oscillations or erratic movement
- All 4 modules should behave consistently (not random)
- Slight servo hum when holding position is normal

**DO NOT**:
- Add back derivative or integral terms
- Increase STEER_KP above 0.2 (will fight internal PID)
- Modify Axon servo internal dampening settings

**OpMode Template:**
```java
@TeleOp
public class MyOpMode extends LinearOpMode {
    public void runOpMode() {
        Hardware hardware = new Hardware(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            hardware.swerveDrive.drive(x, y, rot, fieldRelative);
            hardware.swerveDrive.updateOdometry();
            telemetry.update();
        }
    }
}
```

**PID Tuning:**
- Steering: Start P=0.8, D=0.05, I=0.0
- Drive: Start P=0.1, FF=1/max_speed
- Use FTC Dashboard for real-time adjustments

## Technical Standards

**Coordinate System (FRC/WPILib):**
- +X = forward, +Y = left, +Rotation = CCW
- Units: meters (distance), radians (angles)
- Module order: FL, FR, BL, BR

**Kinematics:**
- State optimization: avoid >90° rotation (reverse drive instead)
- Wheel speed desaturation: normalize while preserving motion ratios
- PID: continuous input with 2π wrapping

## External Libraries

- **FTCLib v2.1.1**: https://docs.ftclib.org/ftclib/kinematics/wpilib-kinematics
- **Pedro Pathing v1.0.8**: https://pedropathing.com/ (needs reconfiguration for swerve odometry)
- **FTC Dashboard v0.4.17**: http://192.168.43.1:8080/dash (real-time tuning)

## Reference Links

**Official FTC:**
- Docs: https://ftc-docs.firstinspires.org/
- Javadoc: https://javadoc.io/doc/org.firstinspires.ftc
- REV Control Hub: https://docs.revrobotics.com/duo-control/

**Hardware:**
- REV Analog Inputs: https://docs.revrobotics.com/duo-control/sensors/analog
- Axon Servos: https://docs.axon-robotics.com/ (analog docs incomplete)

**Swerve Resources:**
- Game Manual 0: https://gm0.org/en/latest/
- WPILib Kinematics: https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
- Team 364 BaseFalconSwerve: https://github.com/Team364/BaseFalconSwerve
- REV MAXSwerve Calibration: https://docs.revrobotics.com/brushless/spark-max/encoders/maxswerve

**Community:**
- Chief Delphi FTC: https://www.chiefdelphi.com/c/first-programs/first-tech-challenge/223
- FTC Forum: https://ftcforum.firstinspires.org/
- ALWAYS REASEARCH, TRIPLECHECK AND USE THE WEB IF UNCLEAR