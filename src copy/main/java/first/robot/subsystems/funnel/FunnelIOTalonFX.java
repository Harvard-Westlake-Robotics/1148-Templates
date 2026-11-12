package first.robot.subsystems.funnel;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

import org.wpilib.hardware.bus.CANPort;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

public class FunnelIOTalonFX implements FunnelIO {
  private final TalonFX funnelMotor;
  private final VoltageOut funnelController;
  private final TalonFX indexerMotor;
  private final VoltageOut indexerController;

  private final TalonFXConfiguration funnelConfig;
  private final TalonFXConfiguration indexerConfig;

  private final StatusSignal<Angle> funnelPosition;
  private final StatusSignal<AngularVelocity> funnelVelocity;
  private final StatusSignal<Voltage> funnelAppliedVoltage;
  private final StatusSignal<Current> funnelStatorCurrentAmps;
  private final StatusSignal<Current> funnelSupplyCurrentAmps;
  private final StatusSignal<Angle> indexerPosition;
  private final StatusSignal<AngularVelocity> indexerVelocity;
  private final StatusSignal<Voltage> indexerAppliedVoltage;
  private final StatusSignal<Current> indexerStatorCurrentAmps;
  private final StatusSignal<Current> indexerSupplyCurrentAmps;

  private final Debouncer funnelConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer indexerConnectedDebouncer = new Debouncer(0.5);

  public FunnelIOTalonFX() {
    funnelMotor = new TalonFX(FunnelConstants.funnelMotorID, new CANBus(CANPort.CAN_S2));
    funnelController = new VoltageOut(0.0);
    indexerMotor = new TalonFX(FunnelConstants.indexerMotorID, new CANBus(CANPort.CAN_S2));
    indexerController = new VoltageOut(0.0);

    funnelConfig = new TalonFXConfiguration();
    indexerConfig = new TalonFXConfiguration();

    funnelConfig.MotorOutput.Inverted = FunnelConstants.funnelInverted;
    funnelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    indexerConfig.MotorOutput.Inverted = FunnelConstants.indexerInverted;
    indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    funnelConfig.CurrentLimits.StatorCurrentLimit = FunnelConstants.funnelStatorLimit;
    funnelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    funnelConfig.CurrentLimits.SupplyCurrentLimit = FunnelConstants.funnelSupplyLimit;
    funnelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    indexerConfig.CurrentLimits.StatorCurrentLimit = FunnelConstants.indexerStatorLimit;
    indexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    indexerConfig.CurrentLimits.SupplyCurrentLimit = FunnelConstants.indexerSupplyLimit;
    indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    funnelMotor.getConfigurator().apply(funnelConfig);
    funnelMotor.setControl(funnelController);
    indexerMotor.getConfigurator().apply(indexerConfig);
    indexerMotor.setControl(indexerController);

    funnelPosition = funnelMotor.getPosition();
    funnelVelocity = funnelMotor.getVelocity();
    funnelAppliedVoltage = funnelMotor.getMotorVoltage();
    funnelStatorCurrentAmps = funnelMotor.getStatorCurrent();
    funnelSupplyCurrentAmps = funnelMotor.getSupplyCurrent();

    indexerPosition = indexerMotor.getPosition();
    indexerVelocity = indexerMotor.getVelocity();
    indexerAppliedVoltage = indexerMotor.getMotorVoltage();
    indexerStatorCurrentAmps = indexerMotor.getStatorCurrent();
    indexerSupplyCurrentAmps = indexerMotor.getSupplyCurrent();
  }

  @Override
  public void updateInputs(FunnelIOInputs inputs) {
    inputs.funnelMotorConnected = funnelConnectedDebouncer.calculate(funnelMotor.isConnected());
    inputs.indexerMotorConnected = indexerConnectedDebouncer.calculate(indexerMotor.isConnected());

    if (inputs.funnelMotorConnected) {
      StatusSignal.refreshAll(
          funnelPosition,
          funnelVelocity,
          funnelAppliedVoltage,
          funnelStatorCurrentAmps,
          funnelSupplyCurrentAmps);
    }

    if (inputs.indexerMotorConnected) {
      StatusSignal.refreshAll(
          indexerPosition,
          indexerVelocity,
          indexerAppliedVoltage,
          indexerStatorCurrentAmps,
          indexerSupplyCurrentAmps);
    }

    inputs.funnelPositionRot = funnelPosition.getValue().in(Rotations);
    inputs.funnelVelocityRPS = funnelVelocity.getValue().in(RotationsPerSecond);
    inputs.funnelAppliedVolts = funnelAppliedVoltage.getValue().in(Volts);
    inputs.funnelStatorCurrentAmps = funnelStatorCurrentAmps.getValue().in(Amps);
    inputs.funnelSupplyCurrentAmps = funnelSupplyCurrentAmps.getValue().in(Amps);

    inputs.indexerPositionRot = indexerPosition.getValue().in(Rotations);
    inputs.indexerVelocityRPS = indexerVelocity.getValue().in(RotationsPerSecond);
    inputs.indexerAppliedVolts = indexerAppliedVoltage.getValue().in(Volts);
    inputs.indexerStatorCurrentAmps = indexerStatorCurrentAmps.getValue().in(Amps);
    inputs.indexerSupplyCurrentAmps = indexerSupplyCurrentAmps.getValue().in(Amps);
  }

  @Override
  public void runVoltageFunnel(double volts) {
    funnelMotor.setControl(new VoltageOut(volts));
  }

  @Override
  public void runVoltageIndexer(double volts) {
    indexerMotor.setControl(new VoltageOut(volts));
  }
}
