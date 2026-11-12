package first.robot.subsystems.shooter;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Celsius;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

import org.wpilib.hardware.bus.CANPort;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
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

public class ShooterIOSim implements ShooterIO {
  private final FlywheelSim shooterSim;

  private final TalonFX shooterMotorMaster;
  private final TalonFXSimState motorSimMaster;
  private final MotionMagicVelocityVoltage shooterMasterController;

  private final TalonFXConfiguration shooterConfig;

  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> motorAppliedVoltage;
  private final StatusSignal<Current> motorStatorCurrentAmps;
  private final StatusSignal<Current> motorSupplyCurrentAmps;
  private final StatusSignal<Temperature> motorTempC;

  private double flywheelMotorPositionRot = 0.0;

  public ShooterIOSim() {
    shooterSim =
        new FlywheelSim(
            Models.flywheelFromPhysicalConstants(
                DCMotor.getKrakenX60Foc(4),
                ShooterConstants.momentOfInertiaKilogramMetersSquared,
                ShooterConstants.motorRotationsPerShooterRotationRatio),
            DCMotor.getKrakenX60Foc(4));

    shooterMotorMaster = new TalonFX(ShooterConstants.motorMasterID, new CANBus(CANPort.CAN_S2));
    motorSimMaster = shooterMotorMaster.getSimState();
    shooterMasterController = new MotionMagicVelocityVoltage(ShooterConstants.defaultRPS);

    shooterConfig = new TalonFXConfiguration();

    shooterConfig.MotorOutput.Inverted = ShooterConstants.motorMasterInverted;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    shooterConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.statorLimit;
    shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.supplyLimit;
    shooterConfig.CurrentLimits.SupplyCurrentLowerLimit = ShooterConstants.supplyLimitLower;
    shooterConfig.CurrentLimits.SupplyCurrentLowerTime = ShooterConstants.supplyLimitTime;
    shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    shooterConfig.Slot0.kP = ShooterConstants.kP;
    shooterConfig.Slot0.kI = ShooterConstants.kI;
    shooterConfig.Slot0.kD = ShooterConstants.kD;
    shooterConfig.Slot0.kS = ShooterConstants.kS;
    shooterConfig.Slot0.kV = ShooterConstants.kV;
    shooterConfig.Slot0.kA = ShooterConstants.kA;

    shooterConfig.MotionMagic.MotionMagicAcceleration = ShooterConstants.motionMagicAcceleration;
    shooterConfig.MotionMagic.MotionMagicJerk = ShooterConstants.motionMagicJerk;

    shooterConfig.Feedback.SensorToMechanismRatio =
        ShooterConstants.motorRotationsPerShooterRotationRatio;

    shooterMotorMaster.getConfigurator().apply(shooterConfig);
    shooterMotorMaster.setControl(shooterMasterController);

    motorVelocity = shooterMotorMaster.getVelocity();
    motorAppliedVoltage = shooterMotorMaster.getMotorVoltage();
    motorStatorCurrentAmps = shooterMotorMaster.getStatorCurrent();
    motorSupplyCurrentAmps = shooterMotorMaster.getSupplyCurrent();
    motorTempC = shooterMotorMaster.getDeviceTemp();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    motorSimMaster.setSupplyVoltage(12.0);

    double requestedVolts = motorSimMaster.getMotorVoltage();

    shooterSim.setInputVoltage(requestedVolts);
    shooterSim.update(0.02);
    flywheelMotorPositionRot +=
        0.02
            * shooterSim.getAngularVelocity()
            * ShooterConstants.motorRotationsPerShooterRotationRatio
            / (2 * Math.PI);

    motorSimMaster.setRawRotorPosition(flywheelMotorPositionRot);
    motorSimMaster.setRotorVelocity(
        shooterSim.getAngularVelocity()
            * ShooterConstants.motorRotationsPerShooterRotationRatio
            / (2 * Math.PI));

    StatusSignal.refreshAll(
        motorVelocity, motorAppliedVoltage, motorStatorCurrentAmps, motorSupplyCurrentAmps);

    inputs.shooterMotorMasterConnected = true;
    inputs.shooterMotorFollower1Connected = true;
    inputs.shooterMotorFollower2Connected = true;
    inputs.shooterMotorFollower3Connected = true;
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
