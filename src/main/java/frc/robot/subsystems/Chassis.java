package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;
import frc.robot.Constants.DDFeedforwardVelocity;
import frc.robot.Constants.velocityPID;

public class Chassis extends SubsystemBase {
  public double wantedVelocity;
  private TalonFX motorRightFront;
  private TalonFX motorRightBack;
  private TalonFX motorLeftFront;
  private TalonFX motorLeftBack;
  private PigeonIMU gyro = new PigeonIMU(14);
  public double velocity;

  private DifferentialDriveKinematics kinematics;
  private DifferentialDrivePoseEstimator poseEstimator;
  Field2d fieldPosition;

  

  DifferentialDriveFeedforward feedforward = new DifferentialDriveFeedforward(DDFeedforwardVelocity.Kv, DDFeedforwardVelocity.Ka, DDFeedforwardVelocity.Kva, DDFeedforwardVelocity.Kaa, trackWidth);


  public Chassis() {
    super();

    motorRightFront = new TalonFX(rightFrontMotorId);
    motorRightBack = new TalonFX(rightBackMotorId);
    motorLeftFront = new TalonFX(leftFrotnMotorId);
    motorLeftBack = new TalonFX(leftBackMotorId);

    motorLeftBack.follow(motorLeftFront);
    motorRightBack.follow(motorRightFront);


    motorRightFront.setInverted(true);
    motorRightBack.setInverted(true);
    motorLeftBack.setInverted(false);
    motorLeftFront.setInverted(false);



    kinematics = new DifferentialDriveKinematics(trackWidth /*enter TW*/);
    poseEstimator = new DifferentialDrivePoseEstimator(kinematics, getAngle(), getLeftDistance(), getRightDistance(), new Pose2d());
    fieldPosition = new Field2d();
    SmartDashboard.putData("Positin",fieldPosition); 





    motorRightFront.config_kP(0, velocityPID.velocityKP);
    motorLeftFront.config_kP(0, velocityPID.velocityKP);


  }


  public void setPostion(double x, double y, double angle){
    poseEstimator.resetPosition(getAngle(), getLeftDistance(), getRightDistance(), new Pose2d(x, y, Rotation2d.fromDegrees(angle)));



  }


  
  public Pose2d getPose(){
    return poseEstimator.getEstimatedPosition();
  }




  public void stop() {
    motorRightFront.set(ControlMode.PercentOutput, 0);
    motorLeftFront.set(ControlMode.PercentOutput, 0);
  }



  public double getLeftDistance(){
    return motorLeftFront.getSelectedSensorPosition() / countPerMeter;
  }
  public double getRightDistance(){
    return motorRightFront.getSelectedSensorPosition() / countPerMeter;
  }

  public void resetAngle() {
    gyro.setFusedHeading(0);
  }

  public Rotation2d getAngle(){
    return Rotation2d.fromDegrees(gyro.getFusedHeading());
  }



  public double getVelocityRight() {
    return (motorRightFront.getSelectedSensorVelocity() / countPerMeter) * 10;
  }
  
  public double getVelocityLeft() {
    return (motorLeftFront.getSelectedSensorVelocity() / countPerMeter) * 10;
  }

  public void setVelocity (ChassisSpeeds speed){
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speed);

    DifferentialDriveWheelVoltages volts = feedforward.calculate(getVelocityLeft(), wheelSpeeds.leftMetersPerSecond , getVelocityRight(), wheelSpeeds.rightMetersPerSecond, cycleTime);
    double right = (wheelSpeeds.rightMetersPerSecond * countPerMeter) / 10;
    double left = (wheelSpeeds.leftMetersPerSecond * countPerMeter) / 10;
    
    motorRightFront.set(ControlMode.Velocity, right, DemandType.ArbitraryFeedForward, volts.right);
    motorLeftFront.set(ControlMode.Velocity, left, DemandType.ArbitraryFeedForward, volts.left);
  }




  public double getPositionRight() {
    return motorRightFront.getSelectedSensorPosition() / countPerMeter;

  }

  public double getPositionLeft() {
    return motorLeftFront.getSelectedSensorPosition() / countPerMeter;
  }

  public double getPowerRight() {
    return motorRightFront.getMotorOutputPercent();
  }

  public double getPowerLeft() {
    return motorLeftFront.getMotorOutputPercent();
  }


  public void setBreak() {
    motorRightFront.setNeutralMode(NeutralMode.Brake);
    motorRightBack.setNeutralMode(NeutralMode.Brake);
    motorLeftFront.setNeutralMode(NeutralMode.Brake);
    motorLeftBack.setNeutralMode(NeutralMode.Brake);
  }

  public void setCoast() {
    motorRightFront.setNeutralMode(NeutralMode.Coast);
    motorRightBack.setNeutralMode(NeutralMode.Coast);
    motorLeftFront.setNeutralMode(NeutralMode.Coast);
    motorLeftBack.setNeutralMode(NeutralMode.Coast);
  }

 


  InstantCommand setBrakeCommand = new InstantCommand(() -> setBreak(), this);
  InstantCommand setCoastCommand = new InstantCommand(() -> setCoast(), this);

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    SmartDashboard.putData("Brake", setBrakeCommand.ignoringDisable(true));
    SmartDashboard.putData("Coast", setCoastCommand.ignoringDisable(true));

    SmartDashboard.putNumber("Wanted Velocity", wantedVelocity);

    builder.addDoubleProperty("Velocity Right", this::getVelocityRight, null);
    builder.addDoubleProperty("Velocity Left", this::getVelocityLeft, null);
    builder.addDoubleProperty("Power Right", this::getPowerRight, null);
    builder.addDoubleProperty("Power Left", this::getPowerLeft, null);
    builder.addDoubleProperty("Position Right", this::getPositionRight, null);
    builder.addDoubleProperty("Position Left", this::getPositionLeft, null);
  }

  @Override
  public void periodic() {  
    super.periodic();
    poseEstimator.update(getAngle(), getLeftDistance(), getRightDistance());
    fieldPosition.setRobotPose(poseEstimator.getEstimatedPosition());
    fieldPosition.setRobotPose(poseEstimator.getEstimatedPosition());
  }
}
