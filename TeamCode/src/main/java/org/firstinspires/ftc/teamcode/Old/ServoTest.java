package org.firstinspires.ftc.teamcode.Old;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;
import java.util.List;

@TeleOp
public class ServoTest extends LinearOpMode {


    public static List<String> ServoList = Arrays.asList("PickUp", "Axon", "GripperTurn", "elbow", "incheietura");

    public GamepadEx gmpad1;


    double position = 0;

    @Override
    public void runOpMode() throws InterruptedException {


        int index = 0;

        gmpad1 = new GamepadEx(gamepad1);


        Servo servo = hardwareMap.get(Servo.class, ServoList.get(index));

        waitForStart();
        while (opModeIsActive()) {


            if (gmpad1.wasJustPressed(GamepadKeys.Button.B)) {
                index += 1;


                if (index == ServoList.size()) index = 0;

                servo = hardwareMap.get(Servo.class, ServoList.get(index));
            }


            if (gmpad1.wasJustPressed(GamepadKeys.Button.X)) {
                index -= 1;


                if (index == ServoList.size()) index = 0;

                servo = hardwareMap.get(Servo.class, ServoList.get(index));
            }


            if (gmpad1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {

                position += 0.05;

                position = Range.clip(position, 0, 1);
                servo.setPosition(position);
            }
            if (gmpad1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {

                position -= 0.05;

                position = Range.clip(position, 0, 1);
                servo.setPosition(position);
            }
         /*
           dashboard.addData("CurrentServo:", ServoList.get(index));
           dashboard.addData("ServoPosition", servo.getPosition());
           dashboard.update();
          */
            gmpad1.readButtons();

        }


    }
}
