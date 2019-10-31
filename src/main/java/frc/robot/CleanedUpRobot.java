package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;

public class CleanedUpRobot extends TimedRobot {
  private DifferentialDrive driveTrian, littleWheels;
  private Spark intake1;
  private Spark intake2;
  private PWMVictorSPX arm;
  private PWMVictorSPX lift;
  private Joystick joystick;
  private Joystick controller;
  private REVDigitBoard digit;
  private Timer time;
  private Compressor compressor;
  //Status private boolean timerRunning;
  private DoubleSolenoid pistonBack;
  private Solenoid pistonFront,pistonArm;
  private LimeLight limelight;
  @Override
  
  // initialize classes
  public void robotInit() {
    driveTrian = new DifferentialDrive(new Spark(1), new Spark(0)); // main drivetrain
    intake1 = new Spark(3);  //intake
    intake2 = new Spark(2);
    lift = new PWMVictorSPX(4); // lift motor
    arm = new PWMVictorSPX(9);  //arm
    littleWheels = new DifferentialDrive(new PWMVictorSPX(5), new PWMVictorSPX(6));  //drop down drivetrain
    pistonFront = new Solenoid(2);  //front pistons
    pistonArm = new Solenoid(1);  //hatch panel mechanism
    pistonBack = new DoubleSolenoid(6, 7);  //rear piston 
    // ENDCODER NEEDED liftPID = new PIDBase(1, 1, 1, source, new PWMVictorSPX(4));
    controller = new Joystick(1);  // operator controller
    joystick = new Joystick(0);  // driver joystick
    digit = new REVDigitBoard();  // REV digit board
    CameraServer.getInstance().startAutomaticCapture();  // start cameraserver
    time = new Timer();
    digit.clear();
    time.start();
    //Status timerRunning = true;
    compressor = new Compressor();
    pistonFront.set(false);  // set intial position of front pistons
    pistonBack.set(DoubleSolenoid.Value.kReverse);  // set initial position of rear pistons
    pistonArm.set(false);
  }

  @Override
  public void teleopPeriodic() {
    
    //Dropdown Drive train
    if(joystick.getRawButton(7)){
      littleWheels.arcadeDrive(joystick.getRawAxis(1), joystick.getTwist());

    //Line Up
    }else if(joystick.getRawButton(1)){
      limelight.lineUp(driveTrian,joystick.getRawAxis(1));
    
    //Main Drive train
    }else{
      driveTrian.arcadeDrive(joystick.getRawAxis(1), joystick.getTwist());
    }

    //main lift control
    lift.set(-controller.getRawAxis(1));//value at end sets the scale for the lift
    
    //main arm control
    arm.set(-controller.getRawAxis(3)*.6);//value at end sets the scale for the arm

    // main intake control
    if(controller.getRawButton(1)){
      intake1.set(-.2);
      intake2.set(.2);
    }else if (controller.getRawButton(2)){
      intake1.set(.2);
      intake2.set(-.2);
    }else{
      intake1.set(0);
      intake2.set(0);
    }

    //piston safety enabling method  
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

    //displayStatus("TELE");
  }

  @Override
  public void disabledPeriodic(){
    //displayStatus("DSBL");

    //Compression
    if(!compressor.getClosedLoopControl())
      compressor.setClosedLoopControl(true);
    if(digit.getButtonA()){
      compressor.start();
    }else{
      compressor.stop();
    }
  }

  @Override
  public void testPeriodic(){
    //displayStatus("TEST");
  }

  @Override
  public void autonomousPeriodic(){
    //displayStatus("AUTO");
  }

  /*This is to be used upon reimplementation of status messaging on the display
  private void displayStatus(String status){
    
    //Time
    if(!timerRunning){time.start();}
    int displayTime =4;
    if(time.get()<displayTime){
      digit.display(status);
    }else if(time.get()>displayTime && time.get()<2*displayTime){
      digit.display(RobotController.getBatteryVoltage());
    }else{
      time.reset();
    }
  }*/

  
}
//Edited from Robot.java by Logan