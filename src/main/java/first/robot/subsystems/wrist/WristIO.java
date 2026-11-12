package first.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public boolean wristMotorConnected = false;
    public double wristPositionRot = 0.0;
    public double wristVelocityRPS = 0.0;
    public double wristAppliedVolts = 0.0;
    public double wristStatorCurrentAmps = 0.0;
    public double wristSupplyCurrentAmps = 0.0;
  }

  public default void updateInputs(WristIOInputs inputs) {}

  public default void runVoltage(double volts) {}

  public default void goToPosition(double rotations) {}

  public default void resetPosition(double rotations) {}
}
