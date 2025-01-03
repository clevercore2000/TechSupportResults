package org.firstinspires.ftc.teamcode.Old;


import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class MainTeleOp extends LinearOpMode {

    public Thread mecanumThread;
   public static double kp = 0.01;
    public static double ki;
    public static double kd;
    public static float target = 50;

    public PIDController controller;
    public Arm arm;

    ElapsedTime timer = new ElapsedTime();
    Hardware hardware;
    double lastError;
    MecanumDrive mecanumDrive;
    @Override
    public void runOpMode() throws InterruptedException {

   //Instances

        hardware = new Hardware(hardwareMap);

        controller = new PIDController(kp,ki,kd);
        mecanumDrive = new MecanumDrive(hardware);
        arm = new Arm(hardware);

        //Separate thread for drive train (to allow delays in arm movement)
        mecanumThread = new Thread(() ->{

            while (opModeIsActive()){

                double y = gamepad1.left_stick_y;
                double x = -gamepad1.left_stick_x * 1.1;
                double rx = -gamepad1.right_stick_x;

                mecanumDrive.drive(x, y ,rx);


            }





        });

//init servo pos

  hardware.incheietura.setPosition(0.45);
  hardware.GrTrn.setPosition(0.44);
  hardware.Axon.setPosition(0.67);
  try{Thread.sleep(200);} catch (InterruptedException e){}
  hardware.pickS.setPosition(0.15);

        waitForStart();
        //Basically init
        mecanumThread.start();
       arm.setSate(Arm.ArmStates.IDLE);
        while (opModeIsActive()){



            if(gamepad2.dpad_left){
                arm.setSate(Arm.ArmStates.PRE_COLLECT);
            }
            if(gamepad2.dpad_down){
                arm.setSate(Arm.ArmStates.COLLECT);
            }
            if (gamepad2.dpad_right){
                arm.setSate(Arm.ArmStates.IDLE);
            }

            if(gamepad2.circle){
                hardware.Axon.setPosition(0.62);
            }
            if(gamepad2.cross){
                hardware.Axon.setPosition(0.8);
            }


            if(gamepad2.square){

                hardware.GrTrn.setPosition(0.38);

            }
            if(gamepad2.triangle){

                hardware.GrTrn.setPosition(0.44);
            }

            if(gamepad2.options){

                arm.setSate(Arm.ArmStates.PULL_UP);
            }

           float g2Y = -gamepad2.left_stick_y * 45;






          //Manual lift control



                if(target<50){
                    target = 50;
                }
            target= Math.min(2100, target+g2Y);


            controller.setPID(kp,ki,kd);




            hardware.sliderMotor1.setPower(controller.calculate(-hardware.sliderMotor1.getCurrentPosition(), target));
            hardware.sliderMotor2.setPower(controller.calculate(-hardware.sliderMotor1.getCurrentPosition(), target));


 ///Telemetry
            /*
   dashbrd.addData("sliderPos1", hardware.sliderMotor1.getCurrentPosition());
   dashbrd.addData("sliderPos2", hardware.sliderMotor2.getCurrentPosition());
   dashbrd.addData("Target: ", target);

   dashbrd.addData("elbowpos:", hardware.elbow.getPosition());
   dashbrd.addData("incheieturapos:" ,  hardware.incheietura.getPosition());

   dashbrd.addData("pwr1", controller.calculate(hardware.sliderMotor1.getCurrentPosition(), target) );
   dashbrd.addData("pwr2", controller.calculate(hardware.sliderMotor2.getCurrentPosition(), target) );
   dashbrd.update();*/

        }


    }

}
