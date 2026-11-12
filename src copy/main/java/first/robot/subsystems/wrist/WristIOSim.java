package first.robot.subsystems.wrist;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

import org.wpilib.hardware.bus.CANPort;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

public class WristIOSim implements WristIO {
    private final SingleJointedArmSim wristSim;

    private final TalonFX wristMotor;
    private final TalonFXSimState motorSim;
    private final MotionMagicVoltage wristController;

    private final TalonFXConfiguration wristConfig;

    private final StatusSignal<Angle> wristPosition;
    private final StatusSignal<AngularVelocity> wristVelocity;
    private final StatusSignal<Voltage> motorAppliedVoltage;
    private final StatusSignal<Current> motorStatorCurrentAmps;
    private final StatusSignal<Current> motorSupplyCurrentAmps;

    public WristIOSim() {
        wristSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60Foc(1),
                WristConstants.motorRotationsPerWristRotationRatio,
                WristConstants.momentOfInertiaKilogramMetersSquared,
                WristConstants.armLengthMeters,
                Units.rotationsToRadians(WristConstants.deployedPositionRotations),
                Units.rotationsToRadians(WristConstants.stowedPositionRotations),
                true,
                Units.rotationsToRadians(WristConstants.stowedPositionRotations));

        wristMotor = new TalonFX(WristConstants.motorID, new CANBus(CANPort.CAN_S2));
        motorSim = wristMotor.getSimState();
        wristController = new MotionMagicVoltage(WristConstants.stowedPositionRotations).withEnableFOC(true);
        wristConfig = new TalonFXConfiguration();

        wristConfig.MotorOutput.Inverted = WristConstants.motorInverted;
        wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        wristConfig.CurrentLimits.StatorCurrentLimit = WristConstants.statorLimit;
        wristConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        wristConfig.CurrentLimits.SupplyCurrentLimit = WristConstants.supplyLimit;
        wristConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        wristConfig.Slot0.kP = WristConstants.kP;
        wristConfig.Slot0.kI = WristConstants.kI;
        wristConfig.Slot0.kD = WristConstants.kD;
        wristConfig.Slot0.kS = WristConstants.kS;
        wristConfig.Slot0.kV = WristConstants.kV;
        wristConfig.Slot0.kG = WristConstants.kG;
        wristConfig.Slot0.kA = WristConstants.kA;
        wristConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        wristConfig.MotionMagic.MotionMagicCruiseVelocity = WristConstants.motionMagicCruiseVelocity;
        wristConfig.MotionMagic.MotionMagicAcceleration = WristConstants.motionMagicAcceleration;
        wristConfig.MotionMagic.MotionMagicJerk = WristConstants.motionMagicJerk;

        wristConfig.Feedback.SensorToMechanismRatio = WristConstants.motorRotationsPerWristRotationRatio;

        wristConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        wristConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = WristConstants.stowedPositionRotations;
        wristConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        wristConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = WristConstants.deployedPositionRotations;

        wristMotor.getConfigurator().apply(wristConfig);
        wristMotor.setControl(wristController);

        motorSim.setRawRotorPosition(
                (WristConstants.motorInverted == InvertedValue.Clockwise_Positive ? -1
                        : 1) * wristSim.getAngle() * WristConstants.motorRotationsPerWristRotationRatio / (2 * Math.PI));

        wristPosition = wristMotor.getPosition();
        wristVelocity = wristMotor.getVelocity();
        motorAppliedVoltage = wristMotor.getMotorVoltage();
        motorStatorCurrentAmps = wristMotor.getStatorCurrent();
        motorSupplyCurrentAmps = wristMotor.getSupplyCurrent();

        // Optimize CAN bus signals
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, wristPosition, motorAppliedVoltage, motorStatorCurrentAmps);
        ParentDevice.optimizeBusUtilizationForAll(wristMotor);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        motorSim.setSupplyVoltage(12.0);

        double requestedVolts = motorSim.getMotorVoltage();

        wristSim.setInputVoltage(
                (WristConstants.motorInverted == InvertedValue.Clockwise_Positive ? -1 : 1) * requestedVolts);
        wristSim.update(0.02);

        motorSim.setRawRotorPosition(
                (WristConstants.motorInverted == InvertedValue.Clockwise_Positive ? -1
                        : 1) * wristSim.getAngle() * WristConstants.motorRotationsPerWristRotationRatio / (2 * Math.PI));
        motorSim.setRotorVelocity(
                (WristConstants.motorInverted == InvertedValue.Clockwise_Positive ? -1
                        : 1) * wristSim.getVelocity()
                                * WristConstants.motorRotationsPerWristRotationRatio
                                / (2 * Math.PI));

        StatusSignal.refreshAll(
                wristPosition,
                wristVelocity,
                motorAppliedVoltage,
                motorStatorCurrentAmps,
                motorSupplyCurrentAmps);

        inputs.wristMotorConnected = true;
        inputs.wristPositionRot = wristPosition.getValue().in(Rotations);
        inputs.wristVelocityRPS = wristVelocity.getValue().in(RotationsPerSecond);
        inputs.wristAppliedVolts = motorAppliedVoltage.getValue().in(Volts);
        inputs.wristStatorCurrentAmps = motorStatorCurrentAmps.getValue().in(Amps);
        inputs.wristSupplyCurrentAmps = motorSupplyCurrentAmps.getValue().in(Amps);
    }

    @Override
    public void runVoltage(double volts) {
        wristMotor.setControl(new VoltageOut(volts));
    }

    @Override
    public void goToPosition(double rotations) {
        wristMotor.setControl(wristController.withPosition(rotations));
    }
}
