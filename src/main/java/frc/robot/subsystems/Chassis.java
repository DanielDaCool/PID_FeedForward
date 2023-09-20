package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

  SimpleMotorFeedforward SimplefeedForward = new SimpleMotorFeedforward(Constants.feedForwardVelocity.Ks, Constants.feedForwardVelocity.Kv, Constants.feedForwardVelocity.Ka);
  

  DifferentialDriveFeedforward feedforward = new DifferentialDriveFeedforward(DDFeedforwardVelocity.Kv, DDFeedforwardVelocity.Ka, DDFeedforwardVelocity.Kva, DDFeedforwardVelocity.Kaa, Constants.widthWheels);


  public Chassis() {
    super();

    motorRightFront = new TalonFX(Constants.rightFrontMotorId);
    motorRightBack = new TalonFX(Constants.rightBackMotorId);
    motorLeftFront = new TalonFX(Constants.leftFrotnMotorId);
    motorLeftBack = new TalonFX(Constants.leftBackMotorId);

    motorLeftBack.follow(motorLeftFront);
    motorRightBack.follow(motorRightFront);


    motorRightFront.setInverted(true);
    motorRightBack.setInverted(true);
    motorLeftBack.setInverted(false);
    motorLeftFront.setInverted(false);

 //   pose = new Pose2d(0, 0, getAngle());    



    motorRightFront.config_kP(0, velocityPID.velocityKP);
    motorLeftFront.config_kP(0, velocityPID.velocityKP);
    SmartDashboard.putData(this);
    /*
     * motorRightFront.config_kI(0, velocityPID.velocityKI);
     * motorLeftFront.config_kI(0, velocityPID.velocityKI);
     * motorRightFront.config_kD(0, velocityPID.velocityKD);
     * motorLeftFront.config_kD(0, velocityPID.velocityKD);
     */

  }

  public double getCounts() {
    return motorRightFront.getSelectedSensorPosition();
  }

  public void stop() {
    motorRightFront.set(ControlMode.PercentOutput, 0);
    motorLeftFront.set(ControlMode.PercentOutput, 0);
  }

  public void resetAngle() {
    gyro.setFusedHeading(0);
  }

  public double getAngle(){
    return gyro.getFusedHeading();
  }

 // public Rotation2d getAngle() {
   // return Rotation2d.fromDegrees(gyro.getFusedHeading());
  //} 

  public double getVelocityRight() {
    return (motorRightFront.getSelectedSensorVelocity() / Constants.countPerMeter) * 10;
  }
  
  public double getVelocityLeft() {
    return (motorLeftFront.getSelectedSensorVelocity() / Constants.countPerMeter) * 10;
  }



  public void setVelocity(double leftVelocity, double rightVelocity){

    

    DifferentialDriveWheelVoltages volts = feedforward.calculate(getVelocityLeft(), leftVelocity, getVelocityRight(), rightVelocity, Constants.cycleTime);


    double left = (volts.left * Constants.countPerMeter) / 10;
    double right = (volts.left * Constants.countPerMeter) / 10;
    /*
    motorRightFront.set(TalonFXControlMode.Velocity, right ,DemandType.ArbitraryFeedForward,(Constants.DDFeedforwardVelocity.Ks * Math.signum(right))  + (volts.right / 12));
    motorLeftFront.set(TalonFXControlMode.Velocity, left ,DemandType.ArbitraryFeedForward, (Constants.DDFeedforwardVelocity.Ks * Math.signum(left)) + (volts.left / 12));
   */

   motorRightFront.set(TalonFXControlMode.Velocity, right + (Constants.DDFeedforwardVelocity.Ks * Math.signum(right)));
   motorLeftFront.set(TalonFXControlMode.Velocity, left + (Constants.DDFeedforwardVelocity.Ks * Math.signum(left))); 
  }

//  public void setVelocity(double wantedVelocity) {



    // double v = (wantedVelocity * Constants.countPerMeter) / 10;
    // System.out.println("Wanted Velocity: " + wantedVelocity);
    // System.out.println("Calculated Feed Forward: " + SimplefeedForward.calculate(wantedVelocity));

    // motorRightFront.set(TalonFXControlMode.Velocity, v ,DemandType.ArbitraryFeedForward, SimplefeedForward.calculate(wantedVelocity) / 12);
    // motorLeftFront.set(TalonFXControlMode.Velocity, v ,DemandType.ArbitraryFeedForward, SimplefeedForward.calculate(wantedVelocity) / 12);
//  } //

  public void setWantedVelocity(double wantedVelocity){
    this.wantedVelocity = wantedVelocity;
  }

  public double getPositionRight() {
    return motorRightFront.getSelectedSensorPosition() / Constants.countPerMeter;

  }

  public double getPositionLeft() {
    return motorLeftFront.getSelectedSensorPosition() / Constants.countPerMeter;
  }

  public double getPowerRight() {
    return motorRightFront.getMotorOutputPercent();
  }

  public double getPowerLeft() {
    return motorLeftFront.getMotorOutputPercent();
  }

  public double getDistance(){
    return getCounts() / Constants.countPerMeter * 10;
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


  }
}
