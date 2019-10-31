package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class LimeLight{
    private NetworkTable table;
    private PID pid;
    public LimeLight(){
        pid = new PID(0.043, 0.03, 0.000);
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public void lineUp(DifferentialDrive drive,double f){
        NetworkTableEntry tx = table.getEntry("tx");
        drive.arcadeDrive(f, pid.calculate(tx.getDouble(0)));
    }

    public void ledMode(boolean on){
        NetworkTableEntry lightLED = table.getEntry("ledMode");
        if(on)
        lightLED.setDouble(3);
        else
        lightLED.setDouble(1);
    }

    public void camMode(boolean vision){
        NetworkTableEntry camMode = table.getEntry("camMode");
        if(vision)
            camMode.setDouble(0);
        else 
            camMode.setDouble(1);
    }

    
}