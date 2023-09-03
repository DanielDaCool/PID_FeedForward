package frc.robot.Util;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;


public class Trapezoid {
    Chassis chassis;

    private double MaxAccel;
    private double MaxVelocity;
    private double CurrentVelocity;



    public Trapezoid(double MaxAccel, double MaxVelocity){
        this.MaxAccel = MaxAccel;
        this.MaxVelocity = MaxVelocity;
        
    }

   /* double distanceByCounts = distance * (Constants.countPerMeter) / 10 + firstDistance; */
    double deltaV = MaxAccel * Constants.cycleTime;




    private double timeToDistance(){
     return CurrentVelocity / MaxAccel;
    }

    public double calculate(double distanceLeft, double CurrentVelocity){
        this.CurrentVelocity = CurrentVelocity;
        
        if(CurrentVelocity < MaxVelocity && distanceLeft > accelDistance()){
            System.out.println("CALCULATED VELOCITY: " + (CurrentVelocity + deltaV));
            System.out.println("Accel");
            return CurrentVelocity + deltaV;
  

        } else if(CurrentVelocity == MaxVelocity) {
            System.out.println("CALCULATED VELOCITY: " + CurrentVelocity);
            System.out.println("keep");
            return CurrentVelocity;
        } else{
            System.out.println("CALCULATED VELOCITY: " + (CurrentVelocity - deltaV));
            System.out.println("DeAccel");
            return CurrentVelocity - deltaV;
        }
    }

    private double accelDistance(){
        
        return 0.5 * MaxAccel * timeToDistance() * timeToDistance();
    }
}


