package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Util.Trapezoid;
import frc.robot.subsystems.Chassis;

public class PidDrive extends CommandBase {
  Chassis chassis;
  double wantedVelocity;
  double distance = 7;
  double remainingDistance;
  double CurrentVelocity;
  Trapezoid trapezoidDrive = new Trapezoid(0.5, 2);
  

  

  public PidDrive(Chassis chassis) {
    this.chassis = chassis;
    addRequirements(chassis);

  }
  

  @Override
  public void initialize() {

    distance = distance + chassis.getLeftDistance();


   
  }



  @Override
  public void execute() {
    CurrentVelocity = chassis.getVelocityRight();
    remainingDistance = distance - chassis.getLeftDistance();
    


    //chassis.setVelocity(trapezoidDrive.calculate(remainingDistance, CurrentVelocity), trapezoidDrive.calculate(remainingDistance, CurrentVelocity));
    chassis.setVelocity(trapezoidDrive.calculate(remainingDistance, CurrentVelocity, 0), trapezoidDrive.calculate(remainingDistance, CurrentVelocity, 0));
    System.out.println(trapezoidDrive.calculate(remainingDistance, CurrentVelocity, 0));
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
