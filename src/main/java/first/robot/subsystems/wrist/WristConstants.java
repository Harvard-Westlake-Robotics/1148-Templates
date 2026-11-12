package first.robot.subsystems.wrist;

import org.wpilib.math.util.Units;

import com.ctre.phoenix6.signals.InvertedValue;

public class WristConstants {
  public static final int motorID = 22;
  public static final InvertedValue motorInverted = InvertedValue.Clockwise_Positive;

  public static final double supplyLimit = 40;
  public static final double statorLimit = 80;

  // PID constants
  public static final double kP = 110.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kS = 0.6;
  public static final double kV = 0.0;
  public static final double kG = 0.68;
  public static final double kA = 0.0;

  // Motion magic constants
  public static final double motionMagicCruiseVelocity = 2.0;
  public static final double motionMagicAcceleration = 4.0;
  public static final double motionMagicJerk = 32.0;

  // Physical constants
  public static final double motorRotationsPerWristRotationRatio = 23.14;

  public static final double stowedPositionRotations = 0.292697;
  public static final double deployedPositionRotations = -0.05;
  public static final double halfUpPositionRotations = Units.degreesToRotations(35) + deployedPositionRotations;
  public static final double slightlyUpPositionRotations = 0.05;

  public static final double stowToleranceRotations = 0.02;
  public static final double deployToleranceRotations = Units.degreesToRotations(10);

  // TODO: Add real values
  public static final double momentOfInertiaKilogramMetersSquared = 0.0434420639;
  public static final double armLengthMeters = 0.33;

  // Voltages
  public static final double intakeDeployVoltage = -3.0;
  public static final double shootingVoltage = 3.0;
  public static final double firstDeployVoltage = -6.0;
}
