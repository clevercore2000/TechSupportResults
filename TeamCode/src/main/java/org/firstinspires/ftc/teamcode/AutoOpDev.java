package org.firstinspires.ftc.teamcode;
 import org.firstinspires.ftc.teamcode.RCodeParser;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//@Autonomous
public class AutoOpDev extends LinearOpMode
//DEBUGPC class  AutoOpDev
{
    public static void main(String[] args)
    {
        AutoOpDev autoOpDev = new AutoOpDev();
        autoOpDev.runOpMode();
    }
    /*
     *   ** This class should be inherit from a ROBOT class that should implement:
     *   - Robot status ( Initialized / opMode(Manu/Auto) / Connected
     *   - Manage the 30 sec autonomus timing
     *   - The main loop iteration ( or implement with threads ?? )
     *   - Controls connection
     *   - Telemetry connection
     *   -
     */
    private  ElapsedTime tm;
    private  double lastOpDuration = 0;
    private  RCodeParser Parser;
    private Hardware hardware;
    ArmDev   gripperArm;
    // g1GripperArm is the index in the Gxx function library
    // When a command block G1 in R-Code is parsed the code will move gripperArm servo
    private  String  g1GripperArm = "G1";  // Asocieate gripperArm with G1 in R-Code
    private  ArmDev  handlerArm;
    // Define G2 in R-Code as GripperArm
    private  String  g2HandlerArm = "G2";  // Asocieate handlerArm with G2 in R-Code
    private  ArmDev  transferArm;
    String           g3Transfer = "G3";    // Asociate transferArm with G3 in R-Code
    private  ArmDev  poleArm;
    private  String  g4PoleArm = "G4";    // Asocieate poleArm with G4 in R-Code
    private  ArmDev  turnerArm;
    private  String  g5TurnerArm = "G5";   // Asocieate turnerArm with G5 in R-Code
    private  SliderDev sliderDev;
    private  MecanumDev mecanumDev;


    // Status of R-Code execution
    private enum AutoRunStatus {blockStopped, blockRqParse, blockRunning, blockReqStop, blockModal }
    AutoRunStatus  Status;
    private  String [] BlockWords;    // This array contains the command words from current R-Code block
    int exeStatus=0;

    @Override
    public void runOpMode()
    {
        Initialize();
        waitForStart();
        tm.startTime();
        boolean opModeIsActive = true; // Replace with actual condition if available
        while(opModeIsActive)
        {


            mecanumDev.execute();   // Mecanum drive
            sliderDev.execute();    // Slider system
            gripperArm.execute();   // Gripper servo

            //handlerArm.execute();   // Handler servo NOT started up yet
            //transferArm.execute();  // Transfer servo NOT started up yet
            poleArm.execute();      // Pole servo
            //turnerArm.execute();    // Turner servo NOT started up yet
            execute();         // Processes the R-Code


 //           telemetry.addData("Sts:", Status);
//            telemetry.addData("Ste:", RCodeParser.rcError );
            telemetry.addData("Stp", exeStatus );
//            telemetry.addData("Stpr", RCodeParser.exeSts );
            telemetry.addData("Blk:", exeBlock);
//            telemetry.addData("ErX", mecanumDev.getDeviationVector()[0] );
//            telemetry.addData("ErY", mecanumDev.getDeviationVector()[1] );
//            telemetry.addData("ErR", mecanumDev.getDeviationVector()[2] );
            telemetry.addData("Ztg ", sliderDev.targetPos );
            telemetry.addData("Zact", sliderDev.actPos );
            telemetry.addData("Zerr", sliderDev.getDeviation() );
            telemetry.addData("Zspe", sliderDev.actSpeed );

            telemetry.update();
        }
        stopOp();
    }

    public void Initialize()
    {
        exeStatus = 1;
        try{
            // ... Instantiate parser and system variables ...
            hardware = new Hardware(hardwareMap);
            Parser = new RCodeParser(); // R-Code interpreter

            mecanumDev = new MecanumDev(hardware);
            sliderDev = new SliderDev( hardware);
            gripperArm = new ArmDev( hardware.gripperServo, g1GripperArm, 200);
            handlerArm = new ArmDev( hardware.handlerServo, g2HandlerArm, 200);
            transferArm = new ArmDev(hardware.transferServo, g3Transfer, 200);
            poleArm = new ArmDev( hardware.poleServo, g4PoleArm, 200);
            tm = new ElapsedTime();

            //exeStatus = 2;
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        // Initialize all systems
        mecanumDev.Initialize();
        sliderDev.Initialize();
        gripperArm.setRange(ConfigVar.ArmCfg.GRIPPER_MIN, ConfigVar.ArmCfg.GRIPPER_MAX);
        gripperArm.moveTo(ConfigVar.ArmCfg.gripperOpened);

        handlerArm.setRange(ConfigVar.ArmCfg.HANDLER_MIN, ConfigVar.ArmCfg.HANDLER_MAX);
//        handlerArm.moveTo(ConfigVar.ArmCfg.handlerClosed);

        transferArm.setRange(ConfigVar.ArmCfg.TRANSFER_MIN, ConfigVar.ArmCfg.TRANSFER_MAX);
//        transferArm.moveTo(ConfigVar.ArmCfg.handlerClosed);

        poleArm.setRange(ConfigVar.ArmCfg.POLE_MIN,ConfigVar.ArmCfg.POLE_MAX);
        poleArm.moveTo(ConfigVar.ArmCfg.poleHome);

//        turnerArm.moveTo(ConfigVar.ArmCfg.turnerIdle);
        Status = AutoRunStatus.blockStopped;
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
        if( Status != AutoRunStatus.blockStopped)
            stopOp();
        // Go to set the current map step to the robot
        Status = AutoRunStatus.blockStopped;
        return true;
    }
    /*
     *   stopOp
     *   ** Stops running the MAP
     *   ** It has the name "stopOp" because method the named "stop" stops the thread
     *
     */
    public  void stopOp()
    {
        if(Status != AutoRunStatus.blockStopped)
        {
            //  Forced stop all systems
            mecanumDev.stop();
            sliderDev.stop();
            gripperArm.stop();
            handlerArm.stop();
            transferArm.stop();
            poleArm.stop();
            turnerArm.stop();
            // Set the robot in rcStopped status
            Status = AutoRunStatus.blockStopped;
        }
    }
    // This method will stop unconditional all robot moves in the next execute()
    public  boolean allSystemsAreStopped()
    {
        // Check that all systems are in position or stopped
        return
                (
                        mecanumDev.isReady()
                                &&  sliderDev.isReady()
                                && sliderDev.isReady()
                                &&  gripperArm.isReady()
                                &&  handlerArm.isReady()
                                &&  transferArm.isReady()
                                &&  poleArm.isReady()
                                && turnerArm.isReady()
                );
    }

    String exeBlock;
    boolean modalActive = false;
    boolean modalServoGroup = false;
    boolean modalSliderGroup = false;
    boolean modalMecanumGroup = false;
    boolean modalMGroup = false;

    public  void execute/*RCode*/()
    {
        // This switch controls the status of running R-Code blocks
        switch ( Status )
        {
            case blockStopped:
                // Start the AutoOpMode R-Code execution
                if( /*DEBUG gamepad1.dpad_up && */  ( Parser.rcStatus == RCodeParser.RCPrgmStatus.rcEnded || Parser.rcStatus == RCodeParser.RCPrgmStatus.rcNotLoaded ))
                {
                    Parser.loadFile();      // Load R-Code file (ex: "auto.rc")
                    startOp();
                    tm.reset();
                    Status = AutoRunStatus.blockRqParse;
                }
                break;
            case blockRqParse:
                // Get next Block of R-Code
                // If non in modal then execute block
                if( ! ( modalMecanumGroup || modalSliderGroup || modalServoGroup) )
                    if( !Parser.nextBlock() )
                    {
                        // Some error occured while loading the program file
                        Status = AutoRunStatus.blockStopped;
                        exeBlock = Parser.crtBlock;
                    }
                // All command words in current block have been processed
                Status = AutoRunStatus.blockRunning;
                break;
            case blockRunning:
                // Process mecanum command words
                runModalMecanumGroup();
                // Process slider command words
                runSliderModalGroup();
                // Process Servos command words
                runServosModalGroup();
                Status = ( modalMecanumGroup || modalSliderGroup || modalServoGroup || modalMGroup )? AutoRunStatus.blockRunning : AutoRunStatus.blockRqParse;
                // Process M,P,Q command words
                runMCodes();
                Parser.rcSetPoints.clear();
                // Add Sensor condition to disable MODAL status for P1
                if(Parser.isModalActive("P1") /* && Sensor-1 active */)
                    Parser.disableModal("P1");
                // Add Sensor condition to disable MODAL status for P1
                if(Parser.isModalActive("Q1") /* && Sensor-2 active */)
                    Parser.disableModal("Q1");

                break;
            case blockReqStop:
                // Executes waits for systems to finish their moves
                if( allSystemsAreStopped() )
                {
                stopOp();
                lastOpDuration = tm.seconds();
                    // Set status to R-Code stopped
                    Status = AutoRunStatus.blockStopped;
                }
                break;
        }
    }

    public void runMCodes()
    {
        if( Parser.blockActive("M99")) // M99 - is R-Code end
        {
            Status = AutoRunStatus.blockReqStop;
            return;
        }

        // Proccess M2 function ==>> wait for all systems to finish ops
        Parser.parseModal() ;   // Parses modal command words P1..P4, Q1..Q4, M2
        // Processes the M2 condition
        if( Parser.isModalActive("M2") && allSystemsAreStopped() ) Parser.disableModal("M2");
    }

    public void runModalMecanumGroup()
    {
        if( mecanumDev.isReady() ) modalMecanumGroup = false;
        if( Parser.blockActive("X") || Parser.blockActive("Y") || Parser.blockActive("R"))
        {
            if( !mecanumDev.isReady()) modalMecanumGroup = true;
            if( !modalMecanumGroup ) mecanumDev.moveTo(Parser.getTarget("X"), Parser.getTarget("Y"), Parser.getTarget("R"), Parser.getTarget("F"));
        }
    }

    // Slider Modal ==>> If Slider is moving and on a next block commes a new command word for slider
    // then it goes MODAL amd waits to finish first move
    public void runSliderModalGroup()
    {
        if( sliderDev.isReady()) modalSliderGroup = false;
        if( Parser.blockActive("Z") )
        {
            if( !sliderDev.isReady()) modalSliderGroup = true;
            if( !modalSliderGroup) sliderDev.moveTo(Parser.getTarget("Z"), Parser.getTarget("S"));
        }
    }

    // If any servo is moving and and on a next block commes a new command word for any servo
    // then it goes MODAL and waits for the moving servo ( servo's) to finish move
    public void runServosModalGroup()
    {

        boolean servoModal = false;
        // Disable MODAL if all servo are READY
        if( gripperArm.isReady() && handlerArm.isReady() && transferArm.isReady() && poleArm.isReady() && turnerArm.isReady() )
            modalServoGroup =false;
        // Gripper arm modal
        if( Parser.blockActive(g1GripperArm) )
        {
            if( !gripperArm.isReady() )
                servoModal = true;
            if( !modalServoGroup )  gripperArm.moveTo(Parser.getTarget(g1GripperArm));
        }

        // Handler arm modal
        if( Parser.blockActive(g2HandlerArm) )
        {
            if( !handlerArm.isReady() ) servoModal = true;
            if( !modalServoGroup )  handlerArm.moveTo(Parser.getTarget(g2HandlerArm));
        }
        // Transfer arm modal
        if( Parser.blockActive(g3Transfer) )
        {
            if( !transferArm.isReady()) servoModal = true;
            if( !modalServoGroup )  transferArm.moveTo(Parser.getTarget(g3Transfer));
        }
        // Pole arm modal
        if( Parser.blockActive(g4PoleArm) )
        {
            if( !poleArm.isReady())
                servoModal = true;
            if( !modalServoGroup ) poleArm.moveTo(Parser.getTarget(g4PoleArm));
        }
        // Turner arm modal
        if( Parser.blockActive(g5TurnerArm) )
        {
            if( !turnerArm.isReady()) servoModal = true;
            if( !modalServoGroup ) turnerArm.moveTo(Parser.getTarget(g5TurnerArm));
        }
        modalServoGroup = servoModal;
    }
}
