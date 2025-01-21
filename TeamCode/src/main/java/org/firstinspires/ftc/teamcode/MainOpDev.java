package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
@Config
public class MainOpDev extends LinearOpMode {
    private Hardware hardware;
    // Declare all devices of the Robot
    private ArmDev gripperArm;
    private ArmDev handlerArm;
    private ArmDev poleArm;
    private ArmDev transferArm;
    private ArmDev turnerArm;
    private ConfigVar configVar;
    private SliderDev sliderDev;
    private MecanumDev mecanumDev;
    private double autoPos=0;
    private double autoSpeed=0;
    // Elaps timer
    private ElapsedTime tm;
    private double dt;
    //    MecanumDev mecanumDrive;

    public void initialize()
    {
        // ... initialization logic ...
        try {
            hardware = new Hardware(hardwareMap);
            configVar = new ConfigVar();

            sliderDev = new SliderDev(hardware);
            mecanumDev = new MecanumDev(hardware);
            tm = new ElapsedTime();

            gripperArm = new ArmDev( hardware.gripperServo, ConfigVar.ArmCfg.gripperSpeed);

            handlerArm = new ArmDev(hardware.handlerServo, ConfigVar.ArmCfg.handlerSpeed);
            poleArm = new ArmDev(hardware.poleServo, ConfigVar.ArmCfg.poleSpeed );
            transferArm = new ArmDev(hardware.transferServo, ConfigVar.ArmCfg.transferSpeed);
            turnerArm = new ArmDev(hardware.turnerServo, ConfigVar.ArmCfg.turnerSpeed);

            tm.startTime();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    public void setHomePositions()
    {
        // .. Set Home/Idle positions of all systems ...
        mecanumDev.Initialize();

        sliderDev.Initialize();
        // sliderDev.Status = SliderDev.SliderStatus.SilderMoveJog;

        gripperArm.setRange(ConfigVar.ArmCfg.GRIPPER_MIN, ConfigVar.ArmCfg.GRIPPER_MAX);
        //gripperArm.moveTo( ConfigVar.ArmCfg.gripperOpened);

        handlerArm.setRange(ConfigVar.ArmCfg.HANDLER_MIN, ConfigVar.ArmCfg.HANDLER_MAX);
        //handlerArm.moveTo( ConfigVar.ArmCfg.handlerClosed);

        transferArm.setRange(ConfigVar.ArmCfg.TRANSFER_MIN, ConfigVar.ArmCfg.TRANSFER_MAX);
        //transferArm.moveTo( ConfigVar.ArmCfg.transferSpPreCoop);

        poleArm.setRange(ConfigVar.ArmCfg.POLE_MIN,ConfigVar.ArmCfg.POLE_MAX);
        //poleArm.moveTo( ConfigVar.ArmCfg.poleHome);

        turnerArm.setRange(ConfigVar.ArmCfg.TURNER_MIN,ConfigVar.ArmCfg.TURNER_MAX);
        //turnerArm.moveTo(ConfigVar.ArmCfg.turnerIdle);
    }

    void processAllSystems()
    {
        mecanumDev.execute();
        sliderDev.execute();
        gripperArm.execute();
        handlerArm.execute();
        poleArm.execute();
        transferArm.execute();
        turnerArm.execute();
    }

    boolean gripperToggle = false;
    boolean griperRiseUp = false;
    boolean poleToggle = false;
    boolean poleRiseUp = false;
    boolean handlerToggle = false;
    boolean handlerRiseUp = false;
    boolean transferToggle = false;
    boolean transferRiseUp = false;
    boolean turnerToggle = false;
    boolean turnerRiseUp = false;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // ... OpMode logic ...
        // Instantiate all required vars
        this.initialize();

        waitForStart();
        // Set Home/Idle position on all systems
        setHomePositions();

        while (opModeIsActive())
        {
            // mecanumDev moves manual according to joystick inputs
            mecanumDev.jogMoveXYR(-gamepad1.left_stick_x, gamepad1.left_stick_y,-gamepad1.right_stick_x);
            sliderDev.moveJog( -gamepad2.left_stick_y );
            // Test code for slider in JOG
            if( gamepad2.dpad_up && !griperRiseUp )
            {
                griperRiseUp = true;
                if (gripperToggle) {
                    gripperArm.moveTo(ConfigVar.ArmCfg.gripperClosed);
                    gripperToggle = false;
                } else {
                    gripperArm.moveTo(ConfigVar.ArmCfg.gripperOpened);
                    gripperToggle = true;
                }
            }
            if( ! gamepad2.dpad_up ) griperRiseUp = false;

            if( gamepad2.dpad_down && ! poleRiseUp )
            {
                poleRiseUp = true;
                if (poleToggle) {
                    poleArm.moveTo(ConfigVar.ArmCfg.poleSaPick);
                    poleToggle = false;
                } else {
                    poleArm.moveTo(ConfigVar.ArmCfg.poleSaPrePick);
                    poleToggle = true;
                }
            }
            if( ! gamepad2.dpad_down ) poleRiseUp = false;

            if( gamepad2.dpad_left && ! handlerRiseUp )
            {
                handlerRiseUp = true;
                if (handlerToggle) {
                    handlerArm.moveTo(ConfigVar.ArmCfg.handlerClosed);
                    handlerToggle = false;
                } else {
                    handlerArm.moveTo(ConfigVar.ArmCfg.handlerOpened);
                    handlerToggle = true;
                }
            }
            if( ! gamepad2.dpad_left ) handlerRiseUp = false;

            if( gamepad2.dpad_right && ! turnerRiseUp )
            {
                turnerRiseUp = true;
                if (turnerToggle) {
                    turnerArm.moveTo(ConfigVar.ArmCfg.turnerIdle);
                    turnerToggle = false;
                } else {
                    turnerArm.moveTo(ConfigVar.ArmCfg.handlerOpened);
                    turnerToggle = true;
                }
            }
            if( ! gamepad2.dpad_right ) turnerRiseUp = false;

            if( gamepad2.circle && ! transferRiseUp )
            {
                transferRiseUp = true;
                if (transferToggle) {
                    transferArm.moveTo(ConfigVar.ArmCfg.transferSpCoop);
                    transferToggle = false;
                } else {
                    transferArm.moveTo(ConfigVar.ArmCfg.transferSpPreCoop);
                    transferToggle = true;
                }
            }
            if( ! gamepad2.circle ) transferRiseUp = false;

            dt = tm.milliseconds();
            tm.reset();
            processAllSystems();
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
            telemetry.addData("s_actP:", sliderDev.actPos);
            telemetry.addData("pow1:", sliderDev.slider1Power);
            telemetry.addData("sKP:", sliderDev.SPEED_KP );
            telemetry.addData("stGain:", sliderDev.STICK_GAIN);
            telemetry.addData("grD:", gripperArm.trgPos);
            telemetry.update();

        }
    }
}