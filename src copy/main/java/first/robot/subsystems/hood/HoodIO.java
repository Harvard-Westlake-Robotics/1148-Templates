package first.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public boolean hoodMotorConnected = false;
    public double hoodPositionRot = 0.0;
    public double hoodPositionDegrees = 0.0;
    public double hoodVelocityDPS = 0.0;
    public double hoodAppliedVolts = 0.0;
    public double hoodStatorCurrentAmps = 0.0;
    public double hoodSupplyCurrentAmps = 0.0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void runVoltage(double volts) {}

  public default void goToAngle(double degrees) {}
}
