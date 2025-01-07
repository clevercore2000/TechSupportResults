package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.List;

public class Arm {

    /* This class is used (or gonna be used) to:
    -Control all servos handling samples
    -Make preset position for said servos to be used in TeleOp and Auto (State Machine)
    -Prob. test servos (Testing here means being able to move them manually and incrementally to determine positions easier) (Not shure if I need to make a separate servo test class or just add as status)
     */



    Hardware hardware;
    ConfigVar configVar; //Pulling Variables from config (might be useful)
    public Arm(Hardware hw)
    {
        hardware = hw;
        configVar = new ConfigVar();
    }

    public enum  ServoState{servoReady, servoReqMove, servoMove}

    /**THINGS FOR THE TEST OPTION (WIP)**/
    /*
    public static List<String> ServoList = Arrays.asList("Gripper", "Slider", "Handler", "Turner", "Transfer");
    public enum TestStatus {TESTING, NOT_TESTING}
    private Arm.TestStatus Status = TestStatus.NOT_TESTING;*/

     /** DEFINING STATES FOR ALL ELEMENTS (basically a switch case for every servo) **/

//Have to determine states and names for them for the turner/correction(not shure of right name but it turns the specimens in the correct way)
  //TODO  enum TurnerStates

    //SA=Sample, SP=Specimen

    enum GripperPosition{Gripper_CLOSED,Gripper_OPEN}
    ServoState gripperStatus;
    public void setGripperStates (GripperPosition gripperPosition){
        switch (gripperPosition){
            case Gripper_CLOSED:{ hardware.gripper.setPosition(ConfigVar.Arm.getGripperClosedPos);
             gripperStatus = ServoState.servoReqMove;
            }
            case Gripper_OPEN: hardware.gripper.setPosition(ConfigVar.Arm.gripperOpenPos);

        }
    }

    enum PoleStates{pole_UP, pole_sample_PREPICK, pole_sample_PICK, pole_IDLE, pole_specimen_PICK, pole_specimen_PLACE}
    public void setPoleStates(PoleStates poleStates){
        switch (poleStates){
            case pole_UP: hardware.slider.setPosition(ConfigVar.Arm.poleUpPos);
            case pole_sample_PREPICK: hardware.slider.setPosition(ConfigVar.Arm.poleSaPrepickPos);
            case pole_sample_PICK: hardware.slider.setPosition(ConfigVar.Arm.poleSaPickPos);
            case pole_IDLE: hardware.slider.setPosition(ConfigVar.Arm.idlePos);
            case pole_specimen_PICK:hardware.slider.setPosition(ConfigVar.Arm.poleSpPickPos);
            case pole_specimen_PLACE:hardware.slider.setPosition(ConfigVar.Arm.poleSpPlacePos);
        }
    }
    enum HandlerStates{handler_OPEN, handler_CLOSED}
    public void setHandlerStates (HandlerStates handlerStates){
        switch (handlerStates){
            case handler_OPEN:hardware.slider.setPosition(ConfigVar.Arm.handlerOpenPos);
            case handler_CLOSED:hardware.slider.setPosition(ConfigVar.Arm.handlerClosedPos);
        }

    }
    enum TransferStates{transfer_specimen_PRESCOOP, transfer_specimen_SCOOP}
    public void setTransferStates (TransferStates transferStates){
        switch (transferStates){
            case transfer_specimen_PRESCOOP:hardware.slider.setPosition(ConfigVar.Arm.preScoopPos);
            case transfer_specimen_SCOOP:hardware.slider.setPosition(ConfigVar.Arm.scoopPos);
        }

    }

    enum ArmStates{INIT, SA_PREPICK, SA_PICK, SP_PICK, SP_PLAC }
    public void setArmStates (ArmStates armStates){
        switch (armStates){
            case INIT:{
                setGripperStates(GripperPosition.Gripper_CLOSED);
                setPoleStates(PoleStates.pole_IDLE);
                setHandlerStates(HandlerStates.handler_CLOSED);
                setTransferStates(TransferStates.transfer_specimen_PRESCOOP);
            }
        }

    }

    public void Initialize(){
    setArmStates(ArmStates.INIT);
    }

    public void Run(){
        Initialize();

        while (true){

            execute();

        }

    }
    public void execute(){

       //TODO if ( Status == TestStatus.TESTING){}
        if(gripperStatus == ServoState.servoReqMove){
            //Intitialise timer
            gripperStatus = ServoState.servoMove;
        }
        if(gripperStatus == ServoState.servoMove){
            //Check timer
            /**if(timer.Elapsed > timeToMove)**/
            gripperStatus = ServoState.servoReady;
        }









    }





}

