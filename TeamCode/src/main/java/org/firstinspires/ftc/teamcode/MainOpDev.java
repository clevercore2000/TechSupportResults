package org.firstinspires.ftc.teamcode;

//TODO import com.acmerobotics.dashboard.FtcDashboard;
//TODO import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Old.Arm;
//TOdo @Config
@TeleOp
public class MainOpDev extends LinearOpMode {
    Hardware hardware;

    ConfigVar configVar;
    ElapsedTime timer;
    public SliderDev sliderDev;
    public MecanumDev mecanumDev;
   // public Arm arm = new Arm(hardware);
    public double dt;
    //    MecanumDev mecanumDrive;
    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new Hardware(hardwareMap);
        configVar = new ConfigVar();

        sliderDev = new SliderDev(hardware);
        mecanumDev = new MecanumDev(hardware);
        // public Arm arm = new Arm(hardware);

        timer = new ElapsedTime();
        timer.startTime();
        // If in Jog
        waitForStart();
        //sliderDev.start();
        mecanumDev.Initialize();
        sliderDev.Initialize();

        while (opModeIsActive())
        {
            // mecanumDev moves manual according to joystick inputs
            mecanumDev.jogMoveXYR(-gamepad1.left_stick_x, gamepad1.left_stick_y,-gamepad1.right_stick_x);
            mecanumDev.execute();
            dt = timer.milliseconds();
            timer.reset();
//            sliderDev.execute();
            //if( timer.milliseconds() > 2 )
           // {



                // Slider is moving according to joystick inputs
//TODO                sliderDev.jogMove( -gamepad2.left_stick_y );
//TODO                timer.reset();
           // }


            telemetry.addData("jX:", -gamepad1.left_stick_x);
            telemetry.addData("jY:", gamepad1.left_stick_y);
            telemetry.addData("sX:", mecanumDev.actSpeed[0]);
            telemetry.addData("sY:", mecanumDev.actSpeed[1]);
            telemetry.addData("sR:", mecanumDev.actSpeed[2]);
            telemetry.addData("dT:", dt);
            telemetry.update();

        }
    }
}
