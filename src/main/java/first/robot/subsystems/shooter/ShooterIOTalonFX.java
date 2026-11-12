package first.robot.subsystems.shooter;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Celsius;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

import org.wpilib.hardware.bus.CANPort;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX shooterMotorMaster;
  private final TalonFX shooterMotorFollower1;
  private final TalonFX shooterMotorFollower2;
  private final TalonFX shooterMotorFollower3;
  private final MotionMagicVelocityVoltage shooterMasterController;
  private final Follower shooterFollower1Controller;
  private final Follower shooterFollower2Controller;
  private final Follower shooterFollower3Controller;

  private final TalonFXConfiguration shooterConfig;

  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> motorAppliedVoltage;
  private final StatusSignal<Current> motorStatorCurrentAmps;
  private final StatusSignal<Current> motorSupplyCurrentAmps;
  private final StatusSignal<Temperature> motorTempC;

  private final Debouncer motorMasterConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer motorFollower1ConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer motorFollower2ConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer motorFollower3ConnectedDebouncer = new Debouncer(0.5);

  public ShooterIOTalonFX() {
    shooterMotorMaster = new TalonFX(ShooterConstants.motorMasterID, new CANBus(CANPort.CAN_S2));
    shooterMotorFollower1 = new TalonFX(ShooterConstants.motorFollower1ID, new CANBus(CANPort.CAN_S2));
    shooterMotorFollower2 = new TalonFX(ShooterConstants.motorFollower2ID, new CANBus(CANPort.CAN_S2));
    shooterMotorFollower3 = new TalonFX(ShooterConstants.motorFollower3ID, new CANBus(CANPort.CAN_S2));
    shooterMasterController = new MotionMagicVelocityVoltage(ShooterConstants.defaultRPS);
    shooterFollower1Controller =
        new Follower(shooterMotorMaster.getDeviceID(), ShooterConstants.motorFollower1Aligned);
    shooterFollower2Controller =
        new Follower(shooterMotorMaster.getDeviceID(), ShooterConstants.motorFollower2Aligned);
    shooterFollower3Controller =
        new Follower(shooterMotorMaster.getDeviceID(), ShooterConstants.motorFollower3Aligned);

    shooterConfig = new TalonFXConfiguration();

    shooterConfig.MotorOutput.Inverted = ShooterConstants.motorMasterInverted;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    shooterConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.statorLimit;
    shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.supplyLimit;
    shooterConfig.CurrentLimits.SupplyCurrentLowerLimit = ShooterConstants.supplyLimitLower;
    shooterConfig.CurrentLimits.SupplyCurrentLowerTime = ShooterConstants.supplyLimitTime;
    shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    shooterMotorFollower1.getConfigurator().apply(shooterConfig);
    shooterMotorFollower2.getConfigurator().apply(shooterConfig);
    shooterMotorFollower3.getConfigurator().apply(shooterConfig);

    shooterConfig.Slot0.kP = ShooterConstants.kP;
    shooterConfig.Slot0.kI = ShooterConstants.kI;
    shooterConfig.Slot0.kD = ShooterConstants.kD;
    shooterConfig.Slot0.kS = ShooterConstants.kS;
    shooterConfig.Slot0.kV = ShooterConstants.kV;
    shooterConfig.Slot0.kA = ShooterConstants.kA;

    shooterConfig.MotionMagic.MotionMagicAcceleration = ShooterConstants.motionMagicAcceleration;
    // shooterConfig.MotionMagic.MotionMagicCruiseVelocity = 9999;
    shooterConfig.MotionMagic.MotionMagicJerk = ShooterConstants.motionMagicJerk;

    shooterConfig.Feedback.SensorToMechanismRatio =
        ShooterConstants.motorRotationsPerShooterRotationRatio;

    shooterMotorMaster.getConfigurator().apply(shooterConfig);
    shooterMotorMaster.setControl(shooterMasterController);
    shooterMotorFollower1.setControl(shooterFollower1Controller);
    shooterMotorFollower2.setControl(shooterFollower2Controller);
    shooterMotorFollower3.setControl(shooterFollower3Controller);

    motorVelocity = shooterMotorMaster.getVelocity();
    motorAppliedVoltage = shooterMotorMaster.getMotorVoltage();
    motorStatorCurrentAmps = shooterMotorMaster.getStatorCurrent();
    motorSupplyCurrentAmps = shooterMotorMaster.getSupplyCurrent();
    motorTempC = shooterMotorMaster.getDeviceTemp();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterMotorMasterConnected =
        motorMasterConnectedDebouncer.calculate(shooterMotorMaster.isConnected());
    inputs.shooterMotorFollower1Connected =
        motorFollower1ConnectedDebouncer.calculate(shooterMotorFollower1.isConnected());
    inputs.shooterMotorFollower2Connected =
        motorFollower2ConnectedDebouncer.calculate(shooterMotorFollower2.isConnected());
    inputs.shooterMotorFollower3Connected =
        motorFollower3ConnectedDebouncer.calculate(shooterMotorFollower3.isConnected());

    if (inputs.shooterMotorMasterConnected
        && inputs.shooterMotorFollower1Connected
        && inputs.shooterMotorFollower2Connected
        && inputs.shooterMotorFollower3Connected) {
      StatusSignal.refreshAll(
          motorVelocity,
          motorAppliedVoltage,
          motorStatorCurrentAmps,
          motorSupplyCurrentAmps,
          motorTempC);
    }

    inputs.shooterVelocityRPS = motorVelocity.getValue().in(RotationsPerSecond);
    inputs.shooterAppliedVolts = motorAppliedVoltage.getValue().in(Volts);
    inputs.shooterStatorCurrentAmps = motorStatorCurrentAmps.getValue().in(Amps);
    inputs.shooterSupplyCurrentAmps = motorSupplyCurrentAmps.getValue().in(Amps);
    inputs.shooterTempC = motorTempC.getValue().in(Celsius);
  }

  @Override
  public void runVoltage(double volts) {
    shooterMotorMaster.setControl(new VoltageOut(volts));
  }

  @Override
  public void runVelocity(double velocityRPS) {
    shooterMotorMaster.setControl(shooterMasterController.withVelocity(velocityRPS));
  }
}
