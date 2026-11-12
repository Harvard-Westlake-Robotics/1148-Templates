package first.robot.subsystems.hood;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Volts;

import org.wpilib.hardware.bus.CANPort;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

public class HoodIOSim implements HoodIO {
  private final SingleJointedArmSim hoodSim;

  private final TalonFX hoodMotor;
  private final TalonFXSimState motorSim;
  private final MotionMagicVoltage hoodController;

  private final TalonFXConfiguration hoodConfig;

  private final StatusSignal<Angle> hoodPosition;
  private final StatusSignal<AngularVelocity> hoodVelocity;
  private final StatusSignal<Voltage> motorAppliedVoltage;
  private final StatusSignal<Current> motorStatorCurrentAmps;
  private final StatusSignal<Current> motorSupplyCurrentAmps;

  public HoodIOSim() {
    hoodSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX44Foc(1),
            HoodConstants.motorRotationsPerHoodRotationRatio,
            HoodConstants.momentOfInertiaKilogramMetersSquared,
            HoodConstants.radiusMeters,
            Units.degreesToRadians(HoodConstants.minAngleDegrees),
            Units.degreesToRadians(HoodConstants.maxAngleDegrees),
            false,
            Units.degreesToRadians(HoodConstants.maxAngleDegrees - 0.1));

    hoodMotor = new TalonFX(HoodConstants.motorID, new CANBus(CANPort.CAN_S2));
    motorSim = hoodMotor.getSimState();
    hoodController =
        new MotionMagicVoltage(Units.degreesToRotations(HoodConstants.maxAngleDegrees - 0.1))
            .withEnableFOC(true);
    hoodConfig = new TalonFXConfiguration();

    hoodConfig.MotorOutput.Inverted = HoodConstants.motorInverted;
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    hoodConfig.Slot0.kP = HoodConstants.kP;
    hoodConfig.Slot0.kI = HoodConstants.kI;
    hoodConfig.Slot0.kD = HoodConstants.kD;
    hoodConfig.Slot0.kS = HoodConstants.kS;
    hoodConfig.Slot0.kV = HoodConstants.kV;
    hoodConfig.Slot0.kA = HoodConstants.kA;

    hoodConfig.MotionMagic.MotionMagicCruiseVelocity = HoodConstants.motionMagicCruiseVelocity;
    hoodConfig.MotionMagic.MotionMagicAcceleration = HoodConstants.motionMagicAcceleration;
    hoodConfig.MotionMagic.MotionMagicJerk = HoodConstants.motionMagicJerk;

    hoodConfig.Feedback.SensorToMechanismRatio = HoodConstants.motorRotationsPerHoodRotationRatio;

    hoodConfig.CurrentLimits.StatorCurrentLimit = HoodConstants.statorLimit;
    hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    // hoodConfig.CurrentLimits.SupplyCurrentLimit = HoodConstants.supplyLimit;
    // hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.degreesToRotations(HoodConstants.maxAngleDegrees);
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.degreesToRotations(HoodConstants.minAngleDegrees);

    hoodMotor.getConfigurator().apply(hoodConfig);
    hoodMotor.setControl(hoodController);

    motorSim.setRawRotorPosition(
        hoodSim.getAngle() * HoodConstants.motorRotationsPerHoodRotationRatio / (2 * Math.PI));

    hoodPosition = hoodMotor.getPosition();
    hoodVelocity = hoodMotor.getVelocity();
    motorAppliedVoltage = hoodMotor.getMotorVoltage();
    motorStatorCurrentAmps = hoodMotor.getStatorCurrent();
    motorSupplyCurrentAmps = hoodMotor.getSupplyCurrent();
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    motorSim.setSupplyVoltage(12.0);

    double requestedVolts = motorSim.getMotorVoltage();

    hoodSim.setInputVoltage(requestedVolts);
    hoodSim.update(0.02);

    motorSim.setRawRotorPosition(
        hoodSim.getAngle() * HoodConstants.motorRotationsPerHoodRotationRatio / (2 * Math.PI));
    motorSim.setRotorVelocity(
        hoodSim.getVelocity() * HoodConstants.motorRotationsPerHoodRotationRatio / (2 * Math.PI));

    StatusSignal.refreshAll(
        hoodPosition,
        hoodVelocity,
        motorAppliedVoltage,
        motorStatorCurrentAmps,
        motorSupplyCurrentAmps);

    inputs.hoodMotorConnected = true;
    inputs.hoodPositionRot = hoodPosition.getValue().in(Rotations);
    inputs.hoodPositionDegrees = hoodPosition.getValue().in(Degrees);
    inputs.hoodVelocityDPS = hoodVelocity.getValue().in(DegreesPerSecond);
    inputs.hoodAppliedVolts = motorAppliedVoltage.getValue().in(Volts);
    inputs.hoodStatorCurrentAmps = motorStatorCurrentAmps.getValue().in(Amps);
    inputs.hoodSupplyCurrentAmps = motorSupplyCurrentAmps.getValue().in(Amps);
  }

  @Override
  public void runVoltage(double volts) {
    hoodMotor.setControl(new VoltageOut(volts));
  }

  @Override
  public void goToAngle(double degrees) {
    hoodMotor.setControl(hoodController.withPosition(Units.degreesToRotations(degrees)));
  }
}
