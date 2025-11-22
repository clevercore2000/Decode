package org.firstinspires.ftc.teamcode.Hardware;


import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ServoCfg
{
    Servo servo;    // Servo hardware
    enum  ServoState {servoReady, servoMove}; // Status of ServoCfg object
    private String gIndex;  // Index number used in the R-Code parser
    //    ConfigVar configVar;
    public ServoState state = ServoState.servoReady;    // Current status of the servo
    public double actPos;       // Actual position with range Min .. Max
    public double trgPos;       // Target position ( set-point position)
    private double prevPos;      // prev position
    private double   defSpeed;   // The defined fixed speed of the arm
    private double  minRange;
    private double  maxRange;
    private final double IN_WINDOW = 2;
    ElapsedTime tm;             // Timer
    // Method: ServoCfg
    // Constructor
    ServoCfg(Servo armServo, String rcGIndex, double definedSpeed)
    {
        servo = armServo;

        gIndex = rcGIndex;
        defSpeed = definedSpeed;
        tm = new ElapsedTime();
        tm.startTime();
    }
    // Method: ServoCfg
    // Constructor
    public ServoCfg(Servo armServo, double definedSpeed)
    {
        servo = armServo;
        gIndex = "";
        defSpeed = definedSpeed;
        tm = new ElapsedTime();
        tm.startTime();
    }
    // Method: setRange
    // Sets the range of servo arm in degree
    public void setRange( double in_minRange, double in_maxRange )
    {
        minRange = in_minRange;
        maxRange = in_maxRange;
    }
    // Method: mapRange
    // Returns position in servo units (0 .. 1)
    public double mapRange(double inValue)
    {
        return ( ((inValue - minRange) * /*servo_full_range*/1) / (maxRange - minRange) + /*servo_min*/0 );
    }
    // Returns actual servo position in ServoCfg range units minRange ... maxRange
    double getActPos()
    {
        return ( (servo.getPosition()*( maxRange-minRange )) / /*(in_max-in_min)*/ + minRange );
    }

    // Method: moveTo
    // This method starts the move and sets appropriate status to arm status
    public void moveTo(double newPosition)
    {
        servo.setPosition( mapRange( newPosition ) );   // Initiates the move to required position
        trgPos = newPosition;               // Save the target position - necessary for calculation of travel duration
        state = ServoState.servoMove;       // Sets the arm in moving state
        tm.reset();                         // Reset the timer dt = 0;
    }
    // Method: moveTo
    // This method starts the move and sets appropriate status to arm status
    public void moveTo(String newPosition)
    {
        if(newPosition == null ) return;
        if(newPosition == "NOP") return;
        double numPosition = Double.parseDouble(newPosition);
        servo.setPosition( mapRange( numPosition ) );   // Initiates the move to required position
        trgPos = numPosition;               // Save the target position - necessary for calculation of travel duration
        state = ServoState.servoMove;       // Sets the arm in moving state
        tm.reset();                         // Reset the timer dt = 0;
    }

    /*
    Method: execute
    ** This function has to be called every machine cycle
    ** Waits for the state to become "servoMode" and for the time to elapse then sets the servo in "servoReady"
    ** The status "servoReady" and "servoMove" are necessary in the AutoOpMode

    */
    public void execute()
    {
        if(state == ServoState.servoMove )
        {
            double dt = tm.milliseconds()/1000; // calculate time elapsed from start of move ( in seconds)
            // We use speed formula v = dx/t to determine the time that elapsed since servo has start moving
            // t = dx/v in our case dx = ( trgPos - actPos) and v = defSpeed
            // The servo would take the time t  in order to complete the move.
            // If it past more than t seconds than set the servo in servoReady state

            if(  dt > ( Math.abs(trgPos-prevPos ))/defSpeed  )
            {
                prevPos = trgPos;
                state = ServoState.servoReady;
            }

        }
    }

    // Method: stop
    // Stops the servo immediately
    public void stop()
    {
        // Set the actual position as target position will stop the moves
        servo.setPosition( servo.getPosition() );
    }
    // Method: getIndex
    // Returns a string with the index number used to identify the servo in the R-Code parser
    public String getIndex(){ return gIndex; }
    // Method: isReady
    // Returns true when status is servoReady
    public boolean isReady()
    {
        return ( state == ServoState.servoReady );
    }
    // Method: notReady is the oposite of isReady
    public boolean notReady()
    {
        return ( state != ServoState.servoReady );
    }

    // Method: inPosition
    // Returns true if the servo is in the chkPosition position
    public boolean inPosition( double chkPosition)
    {
        return ( Math.abs(chkPosition - getActPos() )< IN_WINDOW );
    }
}
