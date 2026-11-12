package first.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean shooterMotorMasterConnected = false;
    public boolean shooterMotorFollower1Connected = false;
    public boolean shooterMotorFollower2Connected = false;
    public boolean shooterMotorFollower3Connected = false;
    public double shooterVelocityRPS = 0.0;
    public double shooterAppliedVolts = 0.0;
    public double shooterStatorCurrentAmps = 0.0;
    public double shooterSupplyCurrentAmps = 0.0;
    public double shooterTempC = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void runVoltage(double volts) {}

  public default void runVelocity(double velocityRPS) {}
}
