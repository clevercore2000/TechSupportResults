package org.firstinspires.ftc.teamcode.Old;

public class MecanumDrive {
    public Hardware hardware;
    public double sensitivity = 0.5;
    public MecanumDrive(Hardware hardware){

        this.hardware = hardware;

    }


    public void drive(double x, double y, double rx){


        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        hardware.leftFront.setPower(frontLeftPower * frontLeftPower * frontLeftPower * sensitivity);
        hardware.leftBack.setPower(backLeftPower * backLeftPower * backLeftPower * sensitivity);
        hardware.rightFront.setPower(frontRightPower * frontRightPower * frontRightPower * sensitivity);
        hardware.rightBack.setPower(backRightPower * backRightPower * backRightPower * sensitivity);

    }



}
