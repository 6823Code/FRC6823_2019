package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.networktables.*;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.*;
import edu.wpi.cscore.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.*;

public class Robot extends TimedRobot {
  private DifferentialDrive driveTrian;
  //private DifferentialDrive littleWheels;
  //private Spark intake1;

  //private Spark intake2;
  private Spark blinkinLeds;
  private MjpegServer camServer;
  private UsbCamera front,back;
  private CANSparkMax intake1CAN;
  private CANSparkMax intake2CAN;
  private CANSparkMax right1CAN,right2CAN,left1CAN,left2CAN;
  private PWMVictorSPX arm;
  private PWMVictorSPX lift;
  private PWMVictorSPX defenceMode;
  private PWMVictorSPX hatchLever;
  private Joystick joystick;
  private Joystick controller;
  private REVDigitBoard digit;
private Boolean heldLever; 
private Boolean intakePassive;
private Boolean heldIntake;
private Boolean heldCamera;
private Boolean camFront;
private Timer timeLever;
private Boolean runningLever;
  private Timer time;
  //private Compressor compressor;
  private boolean timerRunning;
  private DoubleSolenoid pistonBack;
  private Solenoid pistonFront,pistonArm;
  //private LimeLight limelight;
  //private int liftSetpoint;
  //private PIDBase liftPID;
  @Override
  
  // initialize classes
  public void robotInit() {
    //CAN DRIVETRAIN 
    left1CAN = new CANSparkMax(3,CANSparkMaxLowLevel.MotorType.kBrushless);
    left2CAN = new CANSparkMax(4,CANSparkMaxLowLevel.MotorType.kBrushless);
    right1CAN = new CANSparkMax(5,CANSparkMaxLowLevel.MotorType.kBrushless);
    right2CAN =  new CANSparkMax(6,CANSparkMaxLowLevel.MotorType.kBrushless);

    front = new UsbCamera("Front",1);
    back = new UsbCamera("Back",0);
    front.setResolution(150, 150);
    back.setResolution(150, 150);
    front.setFPS(15);
    back.setFPS(15);
    camServer = CameraServer.getInstance().addSwitchedCamera("FrontBack");
    camFront = true;
    intakePassive = true;
    SpeedControllerGroup right = new SpeedControllerGroup(right1CAN, right2CAN);
    SpeedControllerGroup left = new SpeedControllerGroup(left1CAN, left2CAN);
    left2CAN.setInverted(true);
  

    driveTrian = new DifferentialDrive(left, right);
    heldIntake = true;
    //driveTrian = new DifferentialDrive(new Spark(1), new Spark(0)); // main drivetrain

    //intake1 = new Spark(3);  //intake
    //intake2 = new Spark(2);
    blinkinLeds = new Spark(8);
    hatchLever = new PWMVictorSPX(6);
    //SWITCH FOR TROUBLE
    intake1CAN = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    intake2CAN = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
    lift = new PWMVictorSPX(4);
    defenceMode = new PWMVictorSPX(5); // lift motor
    arm = new PWMVictorSPX(9);  //arm
    //littleWheels = new DifferentialDrive(new PWMVictorSPX(5), new PWMVictorSPX(6));  //drop down drivetrain
    pistonFront = new Solenoid(2);  //front pistons
    pistonArm = new Solenoid(4);  //hatch panel mechanism
    pistonBack = new DoubleSolenoid(6, 7);  //rear piston 
    // ENDCODER NEEDED liftPID = new PIDBase(1, 1, 1, source, new PWMVictorSPX(4));
    controller = new Joystick(1);  // operator controller
    joystick = new Joystick(0);  // driver joystick
    digit = new REVDigitBoard();  // REV digit board
    //CameraServer.getInstance().startAutomaticCapture(0);  // start cameraserver
    //CameraServer.getInstance().startAutomaticCapture(1); 
    time = new Timer();
    timeLever = new Timer();
    digit.clear();
    // time.start();     //DIGIT BOARD
    timerRunning = true;
    //compressor = new Compressor();
    pistonFront.set(false);  // set intial position of front pistons
    pistonBack.set(DoubleSolenoid.Value.kReverse);  // set initial position of rear pistons
    pistonArm.set(false);
    //limelight.camMode(false);
    //limelight.ledMode(false);
    camServer.setSource(back);
  }

  @Override
  public void teleopPeriodic() {

      //if(!joystick.getRawButton(7)){  // switch drive to dropdown drivetrain
      //  driveTrian.arcadeDrive(joystick.getRawAxis(1), joystick.getTwist());  // main drive control
    //}else{
      // dropdown drivetrain control
    //  littleWheels.arcadeDrive(joystick.getRawAxis(1), joystick.getTwist());
   // }

    updateDrivetrain();
    updateLift();
    updateIntake();
    updateArm();
    updatePistons();
    updateLever();
    cameraStuff();
    blinkinLeds.set(.41);

    //displayStatus("TELE");
  }

  @Override
  public void disabledPeriodic(){
    
  }

  @Override
  public void testPeriodic(){
    //displayStatus("TEST");
  }

  @Override
  public void autonomousPeriodic(){
      updateDrivetrain();
      updateLift();
      updateIntake();
      updateArm();
      updatePistons();
      cameraStuff();
      updateLever();
  }

  public void updateLever(){
    double hatchLeverSpeed =0;
    if(controller.getRawButton(3))
      hatchLeverSpeed = .4;
    if(controller.getRawButton(4))
      hatchLeverSpeed = -.4;
    
    hatchLever.set(hatchLeverSpeed);
}

public void cameraStuff(){
  boolean pressed = controller.getRawButton(8);
 
  if(pressed && !heldCamera){
    camFront = !camFront;
    heldCamera = true;
  }
  if(!pressed)
    heldCamera = false;

  
  if(camFront)
    camServer.setSource(front);
  else
    camServer.setSource(back);
 
}

public void updateIntake(){


  if(controller.getRawButton(1))
      {
      //intake1.set(-.2);
        //intake2.set(.2);
        intake1CAN.set(.2);
        intake2CAN.set(-.2);
  
      }
      else if (controller.getRawButton(2))
      {
        //intake1.set(.2);
        //intake2.set(-.2);
        intake1CAN.set(-.4);
        intake2CAN.set(.4);
      }
      else{
        boolean pressed = controller.getRawAxis(2)>.2;
 
        if(pressed && !heldIntake){
          intakePassive = !intakePassive;
          heldIntake = true;
        }
        if(!pressed)
         heldIntake = false;
      
        
        if(!intakePassive){
        intake1CAN.set(0);
        intake2CAN.set(0);

        }else{
        intake1CAN.set(.08);
        intake2CAN.set(-.08);
        }
      }
    
      
      //main hatch panel control
       if(controller.getRawButton(6)){
        pistonArm.set(true);
        System.out.println("idkwhythisishere");
       }
       else if(controller.getRawButton(5))
       { pistonArm.set(false);
        System.out.println("codeisntfucked");
       }
        if(joystick.getRawButton(11))
        defenceMode.set(1);
        else
        defenceMode.set(0);
        
  

}
public void updateDrivetrain(){
  if(joystick.getRawButton(7)){
  
  }else if(joystick.getRawButton(2)){
    driveTrian.arcadeDrive((-joystick.getRawAxis(1)), joystick.getTwist()*.7);
  }else if(joystick.getRawButton(1)){
    
    driveTrian.arcadeDrive((-joystick.getRawAxis(1)), joystick.getTwist()*.7);
  }else{
   
  driveTrian.arcadeDrive(-joystick.getRawAxis(1)* 0.8, joystick.getTwist()*.4);
  }

}

public void updateLift(){
  double liftSpeed = controller.getRawAxis(5);
      if(liftSpeed<0)
      lift.set(liftSpeed*.7 );//value at end sets the scale for the lift
      else
      lift.set(liftSpeed*.7);
}

public void updatePistons(){
  //piston safety enabling   
  if(joystick.getRawAxis(3) <= -0.75){
    if(joystick.getRawButton(3)){
      pistonFront.set(true);
    }else if(joystick.getRawButton(5))
      pistonFront.set(false);
    if(joystick.getRawButton(4)){
      pistonBack.set(DoubleSolenoid.Value.kForward);
    }else if(joystick.getRawButton(6))
     pistonBack.set(DoubleSolenoid.Value.kReverse);
  }

}

public void updateArm(){
  double armSpeed = controller.getRawAxis(1);
  int pov = controller.getPOV();

  if(pov == 180)
  arm.set(.75);
  else if(armSpeed<0)
  arm.set(armSpeed*armSpeed*armSpeed*.6);//value at end sets the scale for the arm
  else
  arm.set(armSpeed*.2);

}

}
// penosboilogon
// yall is big gey
// ^this