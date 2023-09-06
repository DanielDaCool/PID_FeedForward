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

   /* double distanceByCounts = distance * (Constants.countPerMeter) / 10 + firstDistance; */
    


    /*public double calculate(double distanceLeft, double CurrentVelocity){
        deltaV = MaxAccel * Constants.cycleTime;
        System.out.println("Current Calc Velocity: " + CurrentVelocity);
        System.out.println("DISTANCE LEFT: " + distanceLeft);
        System.out.println("DELTEV" + deltaV); */

        // if (firstTime == true){
        //     firstTime = false;
        //     return MaxAccel;
        // }

        // if (accelDistance() > distanceLeft) {
        //     return CurrentVelocity - deltaV;
        // } else if (CurrentVelocity > MaxVelocity) {
        //     return CurrentVelocity;
        // } else{
        //     return CurrentVelocity + deltaV;
        // }

            public double calculate(double distanceLeft, double CurrentVelocity, double endV){
                System.out.println("MAXVELO: " + MaxVelocity);
                if((CurrentVelocity < MaxVelocity) && (distanceLeft > accelDistance())){
                    System.out.println("ACCEL");

                    return Math.min(CurrentVelocity + deltaV, MaxVelocity);
                } else if(distanceLeft >= accelDistance()){
                    return CurrentVelocity;
                } else{
                    return Math.max(CurrentVelocity - deltaV, endV);
                }
                
            }

    

    private double accelDistance(){
        double t = MaxVelocity / MaxAccel;
        double d1 = 0.5 * MaxAccel * t * t;
        System.out.println("ACCEL DISTANCE: " + d1);
        return d1;
        
        }
}


