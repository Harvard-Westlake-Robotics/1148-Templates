package first.robot.subsystems.funnel;

import org.littletonrobotics.junction.AutoLog;

public interface FunnelIO {
  @AutoLog
  public static class FunnelIOInputs {
    public boolean funnelMotorConnected = false;
    public double funnelPositionRot = 0.0;
    public double funnelVelocityRPS = 0.0;
    public double funnelAppliedVolts = 0.0;
    public double funnelStatorCurrentAmps = 0.0;
    public double funnelSupplyCurrentAmps = 0.0;

    public boolean indexerMotorConnected = false;
    public double indexerPositionRot = 0.0;
    public double indexerVelocityRPS = 0.0;
    public double indexerAppliedVolts = 0.0;
    public double indexerStatorCurrentAmps = 0.0;
    public double indexerSupplyCurrentAmps = 0.0;
  }

  public default void updateInputs(FunnelIOInputs inputs) {}

  public default void runVoltageFunnel(double volts) {}

  public default void runVoltageIndexer(double volts) {}
}
