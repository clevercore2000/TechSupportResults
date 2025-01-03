package org.firstinspires.ftc.teamcode.Old;

public class Arm {
     Hardware hardware;

     public static double
          gripperOpen,
    gripperClosed;

     public static double
            grTrun,
            grTurnDefault;

    public static double
            wristDown = 0.55,
             wristUP;
     public static double
           elbowCollect,
           elbowSpecien,
            elbowSample,
           elbowIdle;


     public static double
          pickUpPreCollect,
          pickUpCollect,
          pickUpSpecimen,
          pickUpSample,
          pickUpIdle;

     enum ArmStates {

         PRE_COLLECT,
         COLLECT,
         SCORE_SPECIMEN,
         SCORE_SAMPLE,
         IDLE,
         PULL_UP;


    }


    public Arm(Hardware hardware){
        this.hardware = hardware;
    }

    public void setSate (ArmStates armStates){
         switch (armStates){


             case PULL_UP:{


                 hardware.incheietura.setPosition(0.45);
                 hardware.GrTrn.setPosition(0.44);
                 hardware.Axon.setPosition(0.67);
                 try{Thread.sleep(200);} catch (InterruptedException e){}
                 hardware.pickS.setPosition(0.75);
             }
             case PRE_COLLECT:{

                 hardware.pickS.setPosition(0.62);
                 hardware.incheietura.setPosition(0.55);
                 hardware.Axon.setPosition(0.8);


             }break;
             case COLLECT:{

                 hardware.pickS.setPosition(0.7);
                 try{Thread.sleep(700);} catch (InterruptedException e){}
                 hardware.Axon.setPosition(0.63);

                 setSate(Arm.ArmStates.IDLE);



             }break;

             case SCORE_SAMPLE:{




             }break;

             case SCORE_SPECIMEN:{





             }break;
             case IDLE:{
                 hardware.incheietura.setPosition(0.45);
                 hardware.Axon.setPosition(0.62);
                 hardware.GrTrn.setPosition(0.4);
                 try{Thread.sleep(500);} catch (InterruptedException e){}
                 hardware.pickS.setPosition(0.3);
                 try{Thread.sleep(200);} catch (InterruptedException e){}
                 hardware.GrTrn.setPosition(0.38);
             }break;



         }


    }



}
