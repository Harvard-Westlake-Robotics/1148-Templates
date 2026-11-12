package first.robot.subsystems.intake;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Celsius;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

import org.wpilib.hardware.bus.CANPort;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;

public class IntakeIOTalonFX implements IntakeIO {
  // Motors and intake controllers
  private final TalonFX intakeMotorMaster;
  private final TalonFX intakeMotorFollower;
  private final VoltageOut intakeMasterController;
  private final Follower intakeFollowerController;

  private final TalonFXConfiguration intakeConfig;

  private final StatusSignal<AngularVelocity> intakeVelocity;
  private final StatusSignal<Voltage> motorAppliedVoltage;
  private final StatusSignal<Current> motorStatorCurrentAmps;
  private final StatusSignal<Current> motorSupplyCurrentAmps;
  private final StatusSignal<Temperature> motorTempC;

  public final Debouncer motorMasterConnectedDebouncer = new Debouncer(0.5);
  public final Debouncer motorFollowerConnectedDebouncer = new Debouncer(0.5);

  public IntakeIOTalonFX() {
    intakeMotorMaster = new TalonFX(IntakeConstants.motorMasterID, new CANBus(CANPort.CAN_S2));
    intakeMotorFollower = new TalonFX(IntakeConstants.motorFollowerID, new CANBus(CANPort.CAN_S2));
    intakeMasterController = new VoltageOut(0.0).withEnableFOC(true);
    intakeFollowerController =
        new Follower(intakeMotorMaster.getDeviceID(), IntakeConstants.motorFollowerAligned);

    intakeConfig = new TalonFXConfiguration();

    intakeConfig.MotorOutput.Inverted = IntakeConstants.motorMasterInverted;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    intakeConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.statorLimit;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.supplyLimit;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.SupplyCurrentLowerLimit = IntakeConstants.supplyLowerLimit;
    intakeConfig.CurrentLimits.SupplyCurrentLowerTime = IntakeConstants.supplyLowerTime;

    intakeMotorMaster.getConfigurator().apply(intakeConfig);
    intakeMotorFollower.getConfigurator().apply(intakeConfig);
    intakeMotorMaster.setControl(intakeMasterController);
    intakeMotorFollower.setControl(intakeFollowerController);

    intakeVelocity = intakeMotorMaster.getVelocity();
    motorAppliedVoltage = intakeMotorMaster.getMotorVoltage();
    motorStatorCurrentAmps = intakeMotorMaster.getStatorCurrent();
    motorSupplyCurrentAmps = intakeMotorMaster.getSupplyCurrent();
    motorTempC = intakeMotorMaster.getDeviceTemp();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeMotorMasterConnected =
        motorMasterConnectedDebouncer.calculate(intakeMotorMaster.isConnected());
    inputs.intakeMotorFollowerConnected =
        motorFollowerConnectedDebouncer.calculate(intakeMotorFollower.isConnected());

    if (inputs.intakeMotorMasterConnected && inputs.intakeMotorFollowerConnected) {
      StatusSignal.refreshAll(
          intakeVelocity,
          motorAppliedVoltage,
          motorStatorCurrentAmps,
          motorSupplyCurrentAmps,
          motorTempC);
    }

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
