package first.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean intakeMotorMasterConnected = false;
    public boolean intakeMotorFollowerConnected = false;
    public double intakeVelocityRPS = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double intakeStatorCurrentAmps = 0.0;
    public double intakeSupplyCurrentAmps = 0.0;
    public double intakeTempC = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void runVoltage(double volts) {}
}
