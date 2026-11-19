package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Current;
import static edu.wpi.first.units.Units.Amps;

public class MotorWithEncoderIOTalonFX implements MotorWithEncoderIO {
    private final TalonFX motor;
    private final PositionVoltage motorController;

    private final CANcoder encoder;

    private final TalonFXConfiguration motorConfig;
    private final CANcoderConfiguration encoderConfig;

    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Voltage> motorAppliedVoltage;
    private final StatusSignal<Current> motorStatorCurrentAmps;
    private final StatusSignal<Current> motorSupplyCurrentAmps;

    private final Debouncer motorConnectedDebouncer = new Debouncer(0.5);
    private final Debouncer encoderConnectedDebouncer = new Debouncer(0.5);

    public MotorWithEncoderIOTalonFX(int motorID, int encoderID) {

        encoder = new CANcoder(encoderID, new CANBus(CANPort.CAN_S2));

        encoderConfig = new CANcoderConfiguration();
        
        encoderConfig.MagnetSensor.SensorDirection = motorConstants.encoderSensorDirection;
        encoderConfig.MagnetSensor.MagnetOffset = motorConstants.encoderOffset;

        encoder.getConfigurator().apply(encoderConfig);

        motor = new TalonFX(motorID, new CANBus(CANPort.CAN_S2));
        motorController = new PositionVoltage(motorConstants.stowedPositionRotations).withEnableFOC(true);
        motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput.Inverted = motorConstants.motorInverted;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true; 
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true; 

        motorConfig.CurrentLimits.StatorCurrentLimit = motorConstants.statorLimit;
        motorConfig.CurrentLimits.SupplyCurrentLimit = motorConstants.supplyLimit;

        motorConfig.Slot0.kP = motorConstants.kP;
        motorConfig.Slot0.kI = motorConstants.kI;
        motorConfig.Slot0.kD = motorConstants.kD;
        motorConfig.Slot0.kS = motorConstants.kS;
        motorConfig.Slot0.kV = motorConstants.kV;
        motorConfig.Slot0.kG = motorConstants.kG;
        motorConfig.Slot0.kA = motorConstants.kA;

        motorConfig.MotionMagic.MotionMagicCruiseVelocity = motorConstants.motionMagicCruiseVelocity;
        motorConfig.MotionMagic.MotionMagicAcceleration = motorConstants.motionMagicAcceleration;
        motorConfig.MotionMagic.MotionMagicJerk = motorConstants.motionMagicJerk;

        motorConfig.Feedback.FeedbackRemoteSensorID = encoderID;
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        motorConfig.Feedback.RotorToSensorRatio = motorConstants.motorRotationsPerSensorRotationRatio;
        motorConfig.Feedback.SensorToMechanismRatio = motorConstants.sensorRotationsPerMechanismRotationRatio;

        motor.getConfigurator().apply(motorConfig);
        motor.setControl(motorController);

        motorPosition = motor.getPosition();
        motorVelocity = motor.getVelocity();
        motorAppliedVoltage = motor.getMotorVoltage();
        motorStatorCurrentAmps = motor.getStatorCurrent();
        motorSupplyCurrentAmps = motor.getSupplyCurrent();
    }

    @Override
    public void updateInputs(MotorWithEncoderIOInputs inputs) {
        inputs.motorConnected = motorConnectedDebouncer.calculate(motor.isConnected());
        inputs.encoderConnected = encoderConnectedDebouncer.calculate(encoder.isConnected());
        if (inputs.motorConnected) {
            StatusSignal.refreshAll(
                motorPosition,
                motorVelocity,
                motorAppliedVoltage,
                motorStatorCurrentAmps,
                motorSupplyCurrentAmps);
        }

        inputs.motorPositionRot = motorPosition.getValue().in(Rotations);
        inputs.motorVelocityRPS = motorVelocity.getValue().in(RotationsPerSecond);
        inputs.motorAppliedVolts = motorAppliedVoltage.getValue().in(Volts);
        inputs.motorStatorCurrentAmps = motorStatorCurrentAmps.getValue().in(Amps);
        inputs.motorSupplyCurrentAmps = motorSupplyCurrentAmps.getValue().in(Amps);
    }

    @Override
    public void runVoltage(double volts) {
        motor.setControl(new VoltageOut(volts));
    }

    @Override
    public void goToPosition(double rotations) {
        motor.setControl(motorController.withPosition(rotations));
    }
}