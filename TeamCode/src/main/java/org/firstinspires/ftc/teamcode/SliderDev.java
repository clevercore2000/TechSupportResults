package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;


//@SliderDrive
public class SliderDev{
/*
*   ** This class should be inherit from a ROBOT class that should implement:
*   - Robot status ( Initialized / opMode(Manu/Auto) / Connected
*   - Manage the 30 sec autonomus timing
*   - The main loop iteration ( or implement with threads ?? )
*   - Controls
*/
    Hardware hardware;
    ConfigVar configVar;
//    SliderDev thread = new SliderDev( hardware )=-p0o9iu87y65432

    public SliderDev(Hardware hw)
    {
        hardware = hw;
        configVar = new ConfigVar();
    }
/*
    //    public Thread SliderDrive;
    public void run()
    {
        Initialize();

        while( true )
        {
            execute();
        }
    }

 */
    // Joystick EXPO factor {0,..,1}
    public final double STICK_EXPO = ConfigVar.Slider.STICK_EXPO;
    // Position and Speed PI-Controller parameters
    public double POS_KP = ConfigVar.Slider.POS_KP;  // Position PID - proportional coefficient
    public double SPEED_KP = ConfigVar.Slider.SPEED_KP;    // Speed PID - proportional coefficient
    public double SPEED_KI = ConfigVar.Slider.SPEED_KI;     // Speed PID - Integrator coefficient
    public double SPEED_KD = ConfigVar.Slider.SPEED_KD;     // Speed PID - Derivative coefficient
    public final double IN_WINDOW = ConfigVar.Slider.IN_WINDOW;
    public final double MAX_TRAVEL = ConfigVar.Slider.MAX_TRAVEL;// old robot had slider extended to max 2100 ticks and travelled it in 1.5 sec
    public final double MAX_SPEED = ConfigVar.Slider.MAX_SPEED;
    public final double MAX_POWER = ConfigVar.Slider.MAX_POWER;
    // Speed Ramp Generator
    //  * uses logistic function to generate a setpoint signal for the speed controller
    public double MAX_SPEED_LF = ConfigVar.Slider.MAX_SPEED_LF;  // Speed Gain coefficient in Logistic Function (LF) (?? test appropriate value ??)
    public double RATE_SPEED_LF = ConfigVar.Slider.RATE_SPEED_LF;  // Change Rate value in Logistic function f(x) = sspGainCoef/( 1+e^(-sspLogRate*speedSetPoint)
    public double DMP_LPF = ConfigVar.Slider.DMP_LPF;  // Dumping factor used in LowPassFilter ramp generator. Value range is ( 0..1 ) where 0 means no dumping
    public final double STICK_DEAD_ZONE = ConfigVar.Slider.STICK_DEAD_ZONE;
    public final double STICK_GAIN =  ConfigVar.Slider.STICK_GAIN;  // Joystick input value
    public final double JOG_SPEED = ConfigVar.Slider.JOG_SPEED;

    // Predefined positions ( would this even work??)
    /*
    * A strategy is to move the sliders to predefined positions then the driver would control the robot manually just for the local movements
    */
    public final double HOME_POS = ConfigVar.Slider.HOME_POS;    // Home position ( fully retracted ?? )
    public final double LOW_BASKET_POS = ConfigVar.Slider.LOW_BASKET_POS; // Low basket position
    public final double TOP_BASCKET_POS = ConfigVar.Slider.TOP_BASCKET_POS;  // Top basket position
    public final double GROUND_PICKUP_POS = ConfigVar.Slider.GROUND_PICKUP_POS;  // Ground pickup position
    private double actPosition = 0;
    private double actSpeed = 0;
    private double targetPos = 0.0D;
    private double targetSpeed = 0.0D;
    private PIDController sliderSpeedController;
    private final ElapsedTime timer = new ElapsedTime();
    private double prevTrgSpeed=0;
    private double dT;

    /*
*   speedRampGenerator
*   ** Implements a smoothen Acceleration/Deceleration Ramp using a 1st order dumping filter
*   ** out = kf*in_1 + ( 1 - Kf)*in_0 where Kf={0,..1} is the dumping factor ,in_0 - actual input value, in_1 - previous input value
*   ** The Acc/Dec is triggered by the change of the targetSpeed value
*
*/
  private double speedRampGenLPF( double inputTargetSpeed )
  {
      double retunTarget = DMP_LPF*prevTrgSpeed + (1-DMP_LPF)*inputTargetSpeed;
              prevTrgSpeed = retunTarget;
      return retunTarget;
  }
    private double speedRampGenLF(double inputTargetSpeed)
    {
        // Return the Logistic function applied to speedTarget
        // This will be a S shape ramp value depending on the parameters speedLogRate &  speedLogGain
        return ( inputTargetSpeed * MAX_SPEED_LF)/(1 + Math.exp( RATE_SPEED_LF * MAX_SPEED ));
    }

    public boolean inPosition()
    {
        return (Math.abs( targetPos-actPosition ) < IN_WINDOW);
    }

    /*
    *   valueLimitter
    *   ** returns a value of inputValue buit that do not exceeds (-maxValue, +maxValue)
    */
    private double  Limitter( double inputValue, double maxValue )
    {
        return ( Math.max(-maxValue, Math.min(inputValue, maxValue)));
    }
    public void Initialize()
    {
        // Set speed controller PID parameters - this call should be in an initialisation function - it is required to execute once when robot is powered up
        // posController.setPID(posKp, 0, 0);  // Set position controller as Proportional controller
        sliderSpeedController = new PIDController(SPEED_KP, SPEED_KI, SPEED_KD);
        prevTrgSpeed=0;
        timer.startTime();
        timer.reset();
    }

/*
*   SliderDCDrive
*   ** This function has to be called every machine cycle
*   ** Implements a motion control for the Sliders 1 & 2
*   ** Slider1 is master, slider2 follows slider1 ( Matei's concept )
*   ** For the speed control it generates a S shape acceleration/deceleration ramp
*   ** It controls the Sliders using a PID that drives the speed and uses position to trigger the move and direction of Sliders
*   ** Finally the control values are applied to motors power
*
*   *** It may require to implement power control limits [minPower .. maxPower] to avoid overstress the motors ( ... these limites could already be implemted in the DCMotor class )
*       This limits are required because the PID controller would require "infinite" power from motors in certain conditions
*
*/
public void execute()
    {
        double slider1Power;
        double slider2Power;

        /*
        *  ** tagetPosition and target Speed are set in the MoveTo and ManualMove method
        */

        // Read the elapsed time from last timer.reset() call
        // actual value of Time - dT is what timer counted since last Timer.reset() call
        dT = timer.milliseconds(); // Timer interval between two consecutive app scans
        timer.reset();

        // Calculate the actual speed of the slider v = ( X-Xo )/(T-To)
        actSpeed = ( hardware.sliderMotor1.getCurrentPosition() - actPosition/*it is actually previous position*/)/dT;
        // Read actual position of the Slider
        actPosition = hardware.sliderMotor1.getCurrentPosition();   // now we update with actual position

        // Calculates the position deviation as (targetPos - actPosition) and the targetSpeed output of P-Controller (posDeviation, posKp )
        // Target speed is limitted to range of (-MAX_SPEED, +MAX_SPEED)
        targetSpeed = Limitter( ((targetPos - actPosition) * POS_KP), MAX_SPEED);
        // Smothen the taget speed setpoint for the PI controller
        //targetSpeed = speedRampGenLF( targetSpeed );
        targetSpeed = speedRampGenLPF( targetSpeed );
                // Computes PI-Controller DCMotor Power for Slider for a control speed deviation = ( targetSpeed - actSliderSpeed )
        // Power applied to DCMotor of the slider
        slider1Power = Limitter(sliderSpeedController.calculate( /*actSliderSpeed*/hardware.sliderMotor1.getPower(), /*setpoinPower*/targetSpeed), MAX_POWER);
        // Apply to Slider2 the speed setpoint of the slider1 ( ... requires testing to check if stable )
        //  if not stable then it will be required to add P-controller to slider1 as well
        slider2Power = sliderSpeedController.calculate( /*actSliderSpeed*/hardware.sliderMotor1.getPower(), /*setpoinPower*/targetSpeed);
        // Apply calculated control value to Slider
        hardware.sliderMotor1.setPower( slider1Power );
        hardware.sliderMotor2.setPower( slider2Power );
        ///Telemetry

    }
    /*
     *   emergencyStop
     *   ** Stop all moves and puts the DCMotors in Freewheel
     *
     */
    public void emergencyStop()
    {
        // ??? Do we need to implement safety ???
        // Until above question is answered just stop movements
        targetPos = actPosition;
    }

    /*
    *   MoveTo
    *   ** Request to move the sliders to specified position
    *
    */
    public void MoveTo( double inputPosition, double inputSpeed)
    {
        /*
        *   Need revision of scaling of the targetPos
        */
        targetPos = Math.min(MAX_TRAVEL, inputPosition );
        targetSpeed = inputSpeed;
    }
    /*
    * jogMove
    *   ** Moves the slider with Joystick input
    *   ** Joystick in range {-1 .. 1}
    *   ** Joystick input value is proportional with the desired speed ( Ex: the speed increases/decreases proportionally with the forward/backward travel of the stick )
    */
    public void jogMove( double stickIn )
    {
        // targetPos is set to 0
        targetPos = ( stickExpo(stickIn) < -STICK_DEAD_ZONE )? -MAX_TRAVEL : (stickExpo(stickIn) > STICK_DEAD_ZONE )? + MAX_TRAVEL : 0;
        targetSpeed = stickExpo(stickIn) * JOG_SPEED;
    }
    public double getActPosition(){ return actPosition;  }
    public double getActualSpeed(){ return actSpeed; }
    private double stickExpo(double stickIn )
    {
        return ( stickIn * ( 1 - STICK_EXPO ) + STICK_EXPO * Math.pow(stickIn,3) );
    }
}