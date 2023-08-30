package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class PidDrive extends CommandBase {
  Chassis chassis;
  double wantedVelocity;
  

  

  public PidDrive(Chassis chassis) {
    this.chassis = chassis;
    addRequirements(chassis);

  }
  

  @Override
  public void initialize() {
    wantedVelocity = SmartDashboard.getNumber("Wanted Velocity", 0.5);
    SmartDashboard.putNumber("Read Velocity", wantedVelocity);
    chassis.setVelocity(wantedVelocity);
    
  }



  @Override
  public void execute() {

  }


  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
