package frc.robot.Util;
import frc.robot.subsystems.Chassis;


public class Trapezoid {
    Chassis chassis;
    private double deltaV;

    private double MaxAccel;
    private double MaxVelocity;



    public Trapezoid(double MaxAccel, double MaxVelocity){
        this.MaxAccel = MaxAccel;
        this.MaxVelocity = MaxVelocity;
    }



    public double calculate(double distanceLeft, double CurrentVelocity, double endV){



        if (accelDistance() > distanceLeft) {
            System.out.println("DEACCEL");
            return CurrentVelocity - deltaV;
        } 
        if (CurrentVelocity < 0.2){
            return 0.2 + deltaV;
        }
        else if(CurrentVelocity >= MaxVelocity){
            System.out.println("KEEP");
            return MaxVelocity;
            
        } else{
            System.out.println("ACCEL");
            return CurrentVelocity + deltaV;
        }
    }

        private double accelDistance(){
            double t = MaxVelocity / MaxAccel;
            double d1 = (0.5 * MaxAccel * t * t);
            System.out.println("ACCEL DISTANCE: " + d1);
            return d1;
            
            }
    

}


