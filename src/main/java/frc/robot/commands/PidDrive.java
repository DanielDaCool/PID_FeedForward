package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Util.Trapezoid;
import frc.robot.subsystems.Chassis;

public class PidDrive extends CommandBase {
  Chassis chassis;
  double wantedVelocity;
  double distance = 5;
  double remainingDistance;
  double CurrentVelocity;
  double firstDistance;
  Trapezoid trapezoidDrive = new Trapezoid(1, 2);
  

  

  public PidDrive(Chassis chassis) {
    this.chassis = chassis;
    addRequirements(chassis);
    firstDistance = chassis.getDistance();

  }
  

  @Override
  public void initialize() {
    wantedVelocity = SmartDashboard.getNumber("Wanted Velocity", 0.5);
    SmartDashboard.putNumber("Read Velocity", wantedVelocity);


   
  }



  @Override
  public void execute() {
    CurrentVelocity = chassis.getVelocityRight();
    remainingDistance = distance - chassis.getDistance() + firstDistance;
    System.out.println("Remaining Distance: " + remainingDistance);
    System.out.println("Current Velocity: " + CurrentVelocity);



    chassis.setVelocity(trapezoidDrive.calculate(remainingDistance, CurrentVelocity), trapezoidDrive.calculate(remainingDistance, CurrentVelocity));
    

  }


  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }


  @Override
  public boolean isFinished() {
    return remainingDistance < 0.02;
  }
}
