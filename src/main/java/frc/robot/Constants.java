

package frc.robot;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final int rightFrontMotorId = 1;
  public static final int rightBackMotorId = 2;
  public static final int leftFrotnMotorId = 3;
  public static final int leftBackMotorId = 4;
  public static final double countPerMeter = 51330.6021;

  public static final double widthWheels = 0.58;
  public static final double cycleTime = 0.02;

  public static class velocityPID{
    public static final double velocityKP = 0.07;
    public static final double velocityKI = 0;
    public static final double velocityKD = 0;
  }
  public static class feedForwardVelocity{
    public static final double Ks = 0.0061182;
    public static final double Kv = 0.64538;
    public static final double Ka = 0.024367;
  }

  public static class DDFeedforwardVelocity{

    
    public static final double Kp = (0.417 * 1023) / (12 * countPerMeter);
    public static final double Kv = 2.84;
    public static final double Ka = 0.12;
    public static final double Ks = 0.5;
    public static final double Kva = 2.85;
    public static final double Kaa = 0.055;
  }
  }

