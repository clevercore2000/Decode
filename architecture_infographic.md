# FTC Robot Controller Architecture Infographic

```mermaid
graph TB
    subgraph "📱 FTC Robot Controller"
        SDK[FTC SDK v10.3<br/>Core Framework]
        DS[Driver Station<br/>Communication]
    end

    subgraph "🤖 TeamCode Architecture"
        
        subgraph "🎮 OpModes Layer"
            TELEOP[Teleop.java<br/>Driver Control]
            AUTO[AutonomousCubeCollection.java<br/>Self-Driving]
            TEST[IntakeTest.java<br/>Component Testing]
        end

        subgraph "🔧 Hardware Abstraction Layer"
            HW[Hardware.java<br/>Central Manager]
            
            subgraph "Hardware Components"
                CHASSIS[Chassis.java<br/>4x Drive Motors<br/>Mecanum Wheels]
                SLIDERS[Sliders.java<br/>Linear Slide Motors]
                INTAKE[Intake.java<br/>Sample Collection]
                OUTTAKE[Outtake.java<br/>Sample Scoring]
            end

            subgraph "Servo Components"
                BOXFLIPPER[BoxFlipper.java<br/>Sample Container]
                YEETER[Yeeter.java<br/>Sample Ejection]
            end
        end

        subgraph "🎯 Subsystems Layer"
            MECANUM[MecanumDrive.java<br/>Drive Control]
            SLIDER_SUB[SliderSubsystem.java<br/>Vertical Movement]
            INTAKE_SUB[IntakeSubsystem.java<br/>Collection Logic]
            OUTTAKE_SUB[OuttakeSubsystem.java<br/>Scoring Logic]
        end

        subgraph "🧠 Control Systems"
            PID[PIDController.java<br/>Precise Control]
            EMA[EMAFilter.java<br/>Noise Reduction]
        end

        subgraph "🎬 Sequences Layer"
            COLLECT[CollectAndScore.java<br/>Multi-Step Operations<br/>State Machine Logic]
        end

        subgraph "⚙️ Constants"
            DRIVE_CONST[DriveConstants.java<br/>Motor Speeds, Ratios]
            SLIDER_CONST[SliderConstants.java<br/>Position Limits, PID]
            SERVO_CONST[ServoConstants.java<br/>Positions, Speeds]
        end
    end

    subgraph "🎯 Pedro Pathing Library"
        subgraph "Navigation Constants"
            FCONST[FConstants.java<br/>Follower Tuning]
            LCONST[LConstants.java<br/>Localization Config]
        end
        
        subgraph "Tuning Tools"
            VEL_TUNER[Velocity Tuners<br/>Speed Calibration]
            LOC_TEST[Localization Tests<br/>Position Tracking]
            PID_TUNER[PID Tuners<br/>Path Following]
        end

        subgraph "Geometry"
            CIRCLE[Circle.java<br/>Path Shapes]
            TRIANGLE[Triangle.java<br/>Path Shapes]
        end
    end

    %% Connections
    TELEOP --> HW
    AUTO --> HW
    TEST --> HW
    
    HW --> CHASSIS
    HW --> SLIDERS
    HW --> INTAKE
    HW --> OUTTAKE
    
    OUTTAKE --> BOXFLIPPER
    OUTTAKE --> YEETER
    
    MECANUM --> CHASSIS
    SLIDER_SUB --> SLIDERS
    INTAKE_SUB --> INTAKE
    OUTTAKE_SUB --> OUTTAKE
    
    SLIDER_SUB --> PID
    MECANUM --> EMA
    SLIDER_SUB --> EMA
    
    COLLECT --> INTAKE_SUB
    COLLECT --> SLIDER_SUB
    COLLECT --> OUTTAKE_SUB
    
    CHASSIS --> DRIVE_CONST
    SLIDERS --> SLIDER_CONST
    BOXFLIPPER --> SERVO_CONST
    YEETER --> SERVO_CONST
    
    AUTO --> FCONST
    AUTO --> LCONST

    %% Styling
    classDef opmode fill:#e1f5fe
    classDef hardware fill:#f3e5f5
    classDef subsystem fill:#e8f5e8
    classDef control fill:#fff3e0
    classDef sequence fill:#fce4ec
    classDef constants fill:#f1f8e9
    classDef pedro fill:#e0f2f1

    class TELEOP,AUTO,TEST opmode
    class HW,CHASSIS,SLIDERS,INTAKE,OUTTAKE,BOXFLIPPER,YEETER hardware
    class MECANUM,SLIDER_SUB,INTAKE_SUB,OUTTAKE_SUB subsystem
    class PID,EMA control
    class COLLECT sequence
    class DRIVE_CONST,SLIDER_CONST,SERVO_CONST constants
    class FCONST,LCONST,VEL_TUNER,LOC_TEST,PID_TUNER,CIRCLE,TRIANGLE pedro
```

## 📋 Architecture Overview

### 🏗️ **Layered Architecture Pattern**

1. **OpModes Layer** 🎮
   - Entry points for robot programs
   - Handle user input and match flow
   - Coordinate high-level robot behavior

2. **Hardware Abstraction Layer** 🔧
   - Single source of truth for hardware mapping
   - Encapsulates physical device configuration
   - Provides clean interface to hardware components

3. **Subsystems Layer** 🎯
   - Business logic for robot mechanisms
   - State management and control algorithms
   - Coordinate multiple hardware components

4. **Control Systems** 🧠
   - PID controllers for precise movement
   - Filtering for sensor noise reduction
   - Real-time feedback control

5. **Sequences Layer** 🎬
   - Complex multi-step operations
   - State machine implementations
   - Coordinated subsystem interactions

6. **Configuration Layer** ⚙️
   - Centralized constants and parameters
   - Easy tuning without code changes
   - Runtime configuration via FTC Dashboard

### 🌟 **Key Design Patterns**

- **Hardware Abstraction**: Single `Hardware.java` manages all devices
- **Subsystem Architecture**: Each major mechanism has dedicated logic
- **State Machines**: Complex sequences use enum-based states
- **PID Control**: Precise movement with feedback loops
- **Constants Pattern**: Centralized configuration management
- **Integration Pattern**: Pedro Pathing for advanced autonomous

### 🎯 **INTO THE DEEP Season Specific**

- **Sample Handling**: Multi-servo sequences for game pieces
- **Vertical Movement**: PID-controlled linear slides
- **Collection System**: Intake with tube-based collection
- **Scoring System**: Box flipper and yeeter for placement
- **Advanced Navigation**: Pedro Pathing integration

### 🔄 **Data Flow**

1. **OpMode** receives input/commands
2. **Subsystems** process high-level commands
3. **Hardware** translates to device operations
4. **Control Systems** provide precise execution
5. **Constants** provide tuning parameters
6. **Sequences** coordinate complex operations

This architecture provides modularity, testability, and easy maintenance for competitive FTC robotics.