package frc.robot.Util;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;
import pabeles.concurrency.IntOperatorTask.Max;

public class Trapezoid {
    Chassis chassis;

    private double distance;
    private double MaxAccel;
    private double MaxVelocity;
    private double targetVelocity;
    private double distanceLeft;
    private double time; /* need to add current velocity / MaxAccel */

    public Trapezoid(double MaxAccel, double MaxVelocity, double distance, double targetVelocity){
        super();
        this.MaxAccel = MaxAccel;
        this.MaxVelocity = MaxVelocity;
        this.distance = distance;
        this.targetVelocity = targetVelocity;
        double deltaV = MaxAccel * Constants.cycleTime;
        double DistanceToStop = 0.5 * MaxAccel * time * time;

    }


    public double accelDistance(){
        double CurrentVelocity; 
        double t = CurrentVelocity / MaxAccel;
        return 0.5 * MaxAccel * t * t;

    }

    

}
