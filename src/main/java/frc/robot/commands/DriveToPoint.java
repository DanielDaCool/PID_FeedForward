
package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.*;

import frc.robot.Util.Trapezoid;
//import frc.robot.Util.Trapezoid;
import frc.robot.subsystems.Chassis;

public class DriveToPoint extends CommandBase {
  Chassis chassis;
  double wantedX;
  double wantedY;
  Rotation2d wantedAngle = new Rotation2d();
  double remainingDistance;
  double maxVelocity = 2;
  double wantedAccel = 1;
  Pose2d pose = new Pose2d();
  Translation2d translationFinal = new Translation2d();
  Trapezoid trapezoid = new Trapezoid(wantedAccel, maxVelocity);


  public DriveToPoint(Chassis chassis) {
    this.chassis = chassis;
    addRequirements(chassis);
    SmartDashboard.putData(this);


  }

  @Override
  public void initSendable(SendableBuilder builder) {
    
    builder.addDoubleProperty("Wanted Angle Value", () -> wantedAngle.getDegrees(), (wantedAngle) -> this.wantedAngle = Rotation2d.fromDegrees(wantedAngle));
    builder.addDoubleProperty("Wanted Y Value", () -> wantedY, (wantedY) -> this.wantedY = wantedY);
    builder.addDoubleProperty("Wanted X Value", () -> wantedX, (wantedX) -> this.wantedX = wantedX);
  }


  
  
  

  @Override
  public void initialize() {
    translationFinal = new Translation2d(wantedX, wantedY);

  }


  @Override
  public void execute() {

    pose = chassis.getPose();
    if (!(pose.getTranslation().getDistance(translationFinal) <= 0.3)){
      Translation2d vector = translationFinal.minus(pose.getTranslation());
      double angleError = vector.getAngle().minus(pose.getRotation()).getDegrees();
      remainingDistance = vector.getNorm();
      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(trapezoid.calculate(remainingDistance, chassis.getVelocityRight(), 0), 0, trapezoid.calculate(angleError, chassis.getVelocityLeft(), 0));

      chassis.setVelocity(chassisSpeeds);


    }
    else{
      double angleError = wantedAngle.minus(pose.getRotation()).getDegrees();
      System.out.println("ANGLE ERROR: " + angleError);
      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, Math.toRadians(angleError * degreesPerSecond * 0.07));
      chassis.setVelocity(chassisSpeeds);
    }
  }


  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }


  @Override
  public boolean isFinished() {
    if (pose.getTranslation().getDistance(translationFinal) <= 0.3){
      
      if (wantedAngle.minus(pose.getRotation()).getDegrees() <= 5){
        
        return true;
      }
      return false;
    }
    return false;
    
  }
}
