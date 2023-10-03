
package frc.robot;

import frc.robot.commands.DriveToPoint;
import frc.robot.commands.PidDrive;
import frc.robot.subsystems.Chassis;
import edu.wpi.first.wpilibj2.command.Command;



public class RobotContainer {
  Chassis chassis = new Chassis();


  public RobotContainer() {


  }
  public Command getAutonomousCommand() {


    return new DriveToPoint(chassis);
  }
}
