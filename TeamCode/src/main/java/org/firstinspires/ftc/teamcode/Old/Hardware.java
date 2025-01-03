package org.firstinspires.ftc.teamcode.Old;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware
{

   public  DcMotor rightFront;
    public   DcMotor leftFront ;
     public DcMotor rightBack;
    public DcMotor leftBack ;
    public  DcMotor sliderMotor1 ;
    public  DcMotor sliderMotor2;


   public Servo Axon ;
   public Servo GrTrn ;
   public Servo pickS;

   public Servo elbow ;


   public Servo incheietura;




    public Hardware(HardwareMap hw) {



        Axon = hw.get(Servo.class, "Axon");
        GrTrn = hw.get(Servo.class, "GripperTurn");
        pickS= hw.get(Servo.class, "PickUp");
        elbow = hw.get(Servo.class, "elbow");
        incheietura = hw.get(Servo.class, "incheietura");


        rightFront = hw.get(DcMotor.class, "rightFront");
        leftFront = hw.get(DcMotor.class, "leftFront");
        rightBack = hw.get(DcMotor.class, "rightBack");
        leftBack = hw.get(DcMotor.class, "leftBack");





        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sliderMotor1 = hw.get(DcMotor.class, "sliderMotor1");
        sliderMotor2 = hw.get(DcMotor.class, "sliderMotor2");

        sliderMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sliderMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        sliderMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        sliderMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);











    }




























}
