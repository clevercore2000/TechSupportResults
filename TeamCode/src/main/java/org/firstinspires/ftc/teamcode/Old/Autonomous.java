package org.firstinspires.ftc.teamcode.Old;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends LinearOpMode {


    Hardware hardware;
    @Override
    public void runOpMode() throws InterruptedException {

        hardware = new Hardware(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {

            sleep(20000);
            hardware.rightFront.setPower(0.2);
            hardware.leftFront.setPower(0.2);
            hardware.rightBack.setPower(0.2);
            hardware.leftBack.setPower(0.2);
            try{Thread.sleep(5000);} catch (InterruptedException e){}
            hardware.rightFront.setPower(0);
            hardware.leftFront.setPower(0);
            hardware.rightBack.setPower(0);
            hardware.leftBack.setPower(0);

        }
    }
}
