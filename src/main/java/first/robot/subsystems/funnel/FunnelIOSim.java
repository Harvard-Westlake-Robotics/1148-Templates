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
import com.ctre.phoenix6.sim.TalonFXSimState;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

public class FunnelIOSim implements FunnelIO {
  private final FlywheelSim funnelSim;
  private final FlywheelSim indexerSim;

  private final TalonFX funnelMotor;
  private final TalonFXSimState funnelMotorSim;
  private final VoltageOut funnelController;
  private final TalonFX indexerMotor;
  private final TalonFXSimState indexerMotorSim;
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

  private double funnelMotorPositionRot = 0.0;
  private double indexerMotorPositionRot = 0.0;

  public FunnelIOSim() {
    funnelSim =
        new FlywheelSim(
            Models.flywheelFromPhysicalConstants(
                DCMotor.getKrakenX44Foc(1),
                FunnelConstants.funnelMomentOfInertiaKilogramMetersSquared,
                FunnelConstants.motorRotationsPerFunnelRotationRatio),
            DCMotor.getKrakenX60Foc(1));
    indexerSim =
        new FlywheelSim(
            Models.flywheelFromPhysicalConstants(
                DCMotor.getKrakenX44Foc(1),
                FunnelConstants.indexerMomentOfInertiaKilogramMetersSquared,
                FunnelConstants.motorRotationsPerIndexerRotationRatio),
            DCMotor.getKrakenX60Foc(1));

    funnelMotor = new TalonFX(FunnelConstants.funnelMotorID, new CANBus(CANPort.CAN_S2));
    funnelMotorSim = funnelMotor.getSimState();
    funnelController = new VoltageOut(FunnelConstants.funnelSlowVolts);
    indexerMotor = new TalonFX(FunnelConstants.indexerMotorID, new CANBus(CANPort.CAN_S2));
    indexerMotorSim = indexerMotor.getSimState();
    indexerController = new VoltageOut(FunnelConstants.indexerSlowVolts);

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
    double funnelRequestedVolts = funnelMotorSim.getMotorVoltage();

    funnelSim.setInputVoltage(funnelRequestedVolts);
    funnelSim.update(0.02);
    funnelMotorPositionRot +=
        0.02
            * funnelSim.getAngularVelocity()
            * FunnelConstants.motorRotationsPerFunnelRotationRatio
            / (2 * Math.PI);

    funnelMotorSim.setRawRotorPosition(funnelMotorPositionRot);
    funnelMotorSim.setRotorVelocity(
        funnelSim.getAngularVelocity()
            * FunnelConstants.motorRotationsPerFunnelRotationRatio
            / (2 * Math.PI));
    funnelMotorSim.setSupplyVoltage(12.0);

    double indexerRequestedVolts = indexerMotorSim.getMotorVoltage();

    indexerSim.setInputVoltage(indexerRequestedVolts);
    indexerSim.update(0.02);
    indexerMotorPositionRot +=
        0.02
            * indexerSim.getAngularVelocity()
            * FunnelConstants.motorRotationsPerIndexerRotationRatio
            / (2 * Math.PI);

    indexerMotorSim.setRawRotorPosition(indexerMotorPositionRot);
    indexerMotorSim.setRotorVelocity(
        indexerSim.getAngularVelocity()
            * FunnelConstants.motorRotationsPerIndexerRotationRatio
            / (2 * Math.PI));
    indexerMotorSim.setSupplyVoltage(12.0);

    StatusSignal.refreshAll(
        funnelPosition,
        funnelVelocity,
        funnelAppliedVoltage,
        funnelStatorCurrentAmps,
        funnelSupplyCurrentAmps,
        indexerPosition,
        indexerVelocity,
        indexerAppliedVoltage,
        indexerStatorCurrentAmps,
        indexerSupplyCurrentAmps);

    inputs.funnelMotorConnected = true;
    inputs.indexerMotorConnected = true;
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
