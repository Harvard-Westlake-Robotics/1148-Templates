package first.robot.subsystems.intake;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Celsius;
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
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;

public class IntakeIOSim implements IntakeIO {
  private final FlywheelSim intakeSim;

  // Motors and intake controllers
  private final TalonFX intakeMotorMaster;
  private final TalonFXSimState motorSimMaster;
  private final VoltageOut intakeMasterController;

  private final TalonFXConfiguration intakeConfig;

  private final StatusSignal<AngularVelocity> intakeVelocity;
  private final StatusSignal<Voltage> motorAppliedVoltage;
  private final StatusSignal<Current> motorStatorCurrentAmps;
  private final StatusSignal<Current> motorSupplyCurrentAmps;
  private final StatusSignal<Temperature> motorTempC;

  private double intakeMotorPositionRot = 0.0;

  public IntakeIOSim() {
    intakeSim =
        new FlywheelSim(
            Models.flywheelFromPhysicalConstants(
                DCMotor.getKrakenX60Foc(2),
                IntakeConstants.momentOfInertiaKilogramMetersSquared,
                IntakeConstants.motorRotationsPerIntakeRotationRatio),
            DCMotor.getKrakenX60Foc(2));

    intakeMotorMaster = new TalonFX(IntakeConstants.motorMasterID, new CANBus(CANPort.CAN_S2));
    motorSimMaster = intakeMotorMaster.getSimState();
    intakeMasterController = new VoltageOut(0.0).withEnableFOC(true);

    intakeConfig = new TalonFXConfiguration();

    intakeConfig.MotorOutput.Inverted = IntakeConstants.motorMasterInverted;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    intakeConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.statorLimit;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    intakeMotorMaster.getConfigurator().apply(intakeConfig);
    intakeMotorMaster.setControl(intakeMasterController);

    intakeVelocity = intakeMotorMaster.getVelocity();
    motorAppliedVoltage = intakeMotorMaster.getMotorVoltage();
    motorStatorCurrentAmps = intakeMotorMaster.getStatorCurrent();
    motorSupplyCurrentAmps = intakeMotorMaster.getSupplyCurrent();
    motorTempC = intakeMotorMaster.getDeviceTemp();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    motorSimMaster.setSupplyVoltage(12.0);

    double requestedVolts = motorSimMaster.getMotorVoltage();

    intakeSim.setInputVoltage(requestedVolts);
    intakeSim.update(0.02);
    intakeMotorPositionRot +=
        0.02
            * intakeSim.getAngularVelocity()
            * IntakeConstants.motorRotationsPerIntakeRotationRatio
            / (2 * Math.PI);

    motorSimMaster.setRawRotorPosition(intakeMotorPositionRot);
    motorSimMaster.setRotorVelocity(
        intakeSim.getAngularVelocity()
            * IntakeConstants.motorRotationsPerIntakeRotationRatio
            / (2 * Math.PI));

    StatusSignal.refreshAll(
        intakeVelocity,
        motorAppliedVoltage,
        motorStatorCurrentAmps,
        motorSupplyCurrentAmps,
        motorTempC);

    inputs.intakeMotorMasterConnected = true;
    inputs.intakeMotorFollowerConnected = true;
    inputs.intakeVelocityRPS = intakeVelocity.getValue().in(RotationsPerSecond);
    inputs.intakeAppliedVolts = motorAppliedVoltage.getValue().in(Volts);
    inputs.intakeStatorCurrentAmps = motorStatorCurrentAmps.getValue().in(Amps);
    inputs.intakeSupplyCurrentAmps = motorSupplyCurrentAmps.getValue().in(Amps);
    inputs.intakeTempC = motorTempC.getValue().in(Celsius);
  }

  @Override
  public void runVoltage(double volts) {
    intakeMotorMaster.setControl(new VoltageOut(volts));
  }
}
