package org.firstinspires.ftc.teamcode;

 import com.acmerobotics.dashboard.FtcDashboard;
 import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

 import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
@Config
public class MainOpDev extends LinearOpMode {
    Hardware hardware;

    ConfigVar configVar;
    ElapsedTime timer;
    public SliderDev sliderDev;
    public MecanumDev mecanumDev;
    public double autoPos;
    public double autoSpeed;

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
        sliderDev.Status = SliderDev.SliderStatus.SilderMoveJog;
        //sliderDev.Status = SliderDev.SliderStatus.SliderMoveAuto;

        while (opModeIsActive())
        {
            // mecanumDev moves manual according to joystick inputs
            mecanumDev.jogMoveXYR(-gamepad1.left_stick_x, gamepad1.left_stick_y,-gamepad1.right_stick_x);
            mecanumDev.execute();

            // Test code for slider in JOG
            sliderDev.jogMove( -gamepad2.left_stick_y );
            // Test code for slider in AUTO
            //autoPos = ConfigVar.Slider.autoPosition;
            //autoSpeed = ConfigVar.Slider.autoSpeed;
            //sliderDev.MoveTo( autoPos, autoSpeed );
            sliderDev.execute();

            dt = timer.milliseconds();
            timer.reset();


                // Slider is moving according to joystick inputs
            //sliderDev.jogMove( -gamepad2.left_stick_y );

           // }

/*
            telemetry.addData("jX:", -gamepad1.left_stick_x);
            telemetry.addData("jY:", gamepad1.left_stick_y);
            telemetry.addData("sX:", mecanumDev.actSpeed[0]);
            telemetry.addData("sY:", mecanumDev.actSpeed[1]);
            telemetry.addData("sR:", mecanumDev.actSpeed[2]);
            telemetry.addData("dT:", dt);

 */
            telemetry.addData("s_tgS:", sliderDev.getTargetSpeed());
            telemetry.addData("s_tgP:", sliderDev.targetPos);
            telemetry.addData("s_actP:", sliderDev.actPosition);
            telemetry.addData("pow1:", sliderDev.slider1Power);


            telemetry.addData("sKP:", sliderDev.SPEED_KP );
            telemetry.addData("stGain:", sliderDev.STICK_GAIN);
            telemetry.update();

        }
    }
}
