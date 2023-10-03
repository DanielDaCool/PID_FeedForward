
package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class DriveToPoint extends CommandBase {
  Chassis chassis;
  double wantedX;
  double wantedY;
  double wantedAngle;


  public DriveToPoint(Chassis chassis) {
    this.chassis = chassis;
    addRequirements(chassis);

  }

  @Override
  public void initSendable(SendableBuilder builder) {
    SmartDashboard.putNumber("Wanted X Value", 0);
    SmartDashboard.putNumber("Wanted Y Value", 0);
    SmartDashboard.putNumber("Wanted Angle Value", 0);

    wantedX = SmartDashboard.getNumber("Wanted X Value", 0);
    wantedY = SmartDashboard.getNumber("Wanted Y Value", 0);
    wantedAngle = SmartDashboard.getNumber("Wanted Angle Value", 0);
  }
  
  

  @Override
  public void initialize() {}


  @Override
  public void execute() {
    
    
  }


  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return false;
  }
}
