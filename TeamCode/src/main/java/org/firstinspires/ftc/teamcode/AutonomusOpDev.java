package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Old.Arm;

import java.util.Timer;

public class AutonomusOpDev extends LinearOpMode
{
    /*
     *   ** This class should be inherit from a ROBOT class that should implement:
     *   - Robot status ( Initialized / opMode(Manu/Auto) / Connected
     *   - Manage the 30 sec autonomus timing
     *   - The main loop iteration ( or implement with threads ?? )
     *   - Controls connection
     *   - Telemetry connection
     *   -
     */
    //AutonomusDev thread = new AutonomusDev();
    private final ElapsedTime timer = new ElapsedTime();
    Hardware hardware;
    ConfigVar configVar;

    public SliderDev sliderDev = new SliderDev( hardware);
    public MecanumDev mecanumDev = new MecanumDev(hardware);
   // public Arm arm = new Arm(hardware);
    final double stickMinimum  =  0.01;
    @Override
    public void runOpMode()
    {
        Initialize();
        // If in Jog
        sliderDev.Initialize();
        mecanumDev.Initialize();

        waitForStart();
        while( opModeIsActive() )
        {
            mecanumDev.execute();
            sliderDev.execute();
            execute();
        }
        stopOp();
    }

    public enum AutoRunStatus { Stopped, SetStepMoves, StepRunning,  ReqStop, EmgStop /*Emergency Stop !!*/ }
    public enum ArmState {  PRE_COLLECT, COLLECT, SCORE_SPECIMEN, SCORE_SAMPLE, IDLE, PULL_UP   };
    public static class MapStep // Contains all possible operation in one step of the autonomus list
    {
        double      mecTrgX;      // Mecanum X target
        double      mecTrgY;       // Mecanum Y target
        double      mecTrgR;       // Mecanum R target
        double      mecTrgSpeed;   //  Mecanum Speed target
        ArmState    ArmStatus;
        double      sliderTrgPos;   // Slider target position {0 .. 2100(??)}
        double      sliderTrgSpeed;   // Slider target Speed
    }
    public final int MAP_STEPS = 200;
    AutoRunStatus  Status;
    private int CurrentStep;    // Current step to be executed
    private int StepsCount;    // Number of programmed steps
    public MapStep [] Map = new MapStep[200];

    public void Initialize()
    {
        hardware = new Hardware(hardwareMap);
        configVar = new ConfigVar();
        timer.startTime();
        CurrentStep = StepsCount = 0;

    }
    /*
    *   goIdleIdleState
    *   ** Moves the robot in the Idle position
    *
    */
    boolean goIdleState()
    {
        ;// executes moves to bring the robot in IdleStatus ( a ststus before start of Autonomus)
        return true;
    }

    /*
    *   startOp
    *   ** Starts running the MAP
    *   ** It has the name "startOp" because method the named "start" starts the thread
    *
    */
    boolean startOp()
    {
        // If not in stop then ignore the request
        if( Status != AutoRunStatus.Stopped) return false;
        // Initialize MAP
        CurrentStep = 0;
        // Go to set the current map step to the robot
        Status = AutoRunStatus.SetStepMoves;
        return true;

    }
    /*
     *   stopOp
     *   ** Stops running the MAP
     *   ** It has the name "stopOp" because method the named "stop" stops the thread
     *
     */
    public void stopOp()
    {
        if(Status != AutoRunStatus.Stopped ) Status = AutoRunStatus.ReqStop;
    }
    // This method will stop unconditional all robot moves in the next execute()
    public void emergencyStop()
    {
        Status = AutoRunStatus.EmgStop;
    }
    public void execute/*Map*/()
    {
        switch ( Status )
        {
            case Stopped:
                // Start Autonomus OpMode
                break;
            case SetStepMoves:
                sliderDev.MoveTo( Map[CurrentStep].sliderTrgPos, Map[CurrentStep].sliderTrgSpeed);
                mecanumDev.MoveXYRTo(Map[CurrentStep].mecTrgX, Map[CurrentStep].mecTrgY, Map[CurrentStep].mecTrgR, Map[CurrentStep].mecTrgSpeed );
                Status = AutoRunStatus.StepRunning;
                break;
            case StepRunning:
                if( mecanumDev.readyToMove() && sliderDev.inPosition() /* && Arm completed all actions */)
                {
                    CurrentStep++;
                    Status = AutoRunStatus.SetStepMoves;
                }
                break;
            case ReqStop:
                if( mecanumDev.inPosition() && sliderDev.inPosition() /* && Arm completed all actions */)
                {
                    Status = AutoRunStatus.Stopped;
                }
                break;
            case EmgStop:
                sliderDev.emergencyStop();
                mecanumDev.emergencyStop();
                // arm.emergencyStop();
                // All moving systems has to stop
                Status = AutoRunStatus.ReqStop;
                break;
        }
    }
}
