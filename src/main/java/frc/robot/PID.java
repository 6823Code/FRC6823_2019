package frc.robot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class PID{
    private double P;
    private double I;
    private double D;

    private double rcw =0;

    private int integral, previous_error, setpoint = 0;
    private DifferentialDrive robotDrive;


    public PID(double p,double i,double d){
        P=p;
        I=i;
        D=d;
    }

    public void setSetpoint(int setpointT)
    {
        setpoint = setpointT;
    }

    public double calculate(double actual){
        double error = setpoint - actual; // Error = Target - Actual
        integral += (error*.02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
        double derivative = (error - previous_error) / .02;
        rcw = P*error + I*integral + D*derivative;
        return rcw;
    }

    public void execute(double actual)
    {
        calculate(actual);
        robotDrive.arcadeDrive(0, rcw);
    }
}
