package first.robot.subsystems.hood;

import com.ctre.phoenix6.signals.InvertedValue;

public class HoodConstants {
  public static final int motorID = 19;
  public static final InvertedValue motorInverted = InvertedValue.CounterClockwise_Positive;
  public static final double statorLimit = 40;

  // PID constants
  public static final double kP = 75.0;
  public static final double kI = 2;
  public static final double kD = 0.0;
  public static final double kS = 0.24; // 0.45
  public static final double kV = 9.9; // 0.124
  public static final double kA = 0.0;

  // Motion magic constants
  public static final double motionMagicCruiseVelocity = 0.5;
  public static final double motionMagicAcceleration = 6;
  public static final double motionMagicJerk = 80;

  // Max and min angles (degrees)
  public static final double maxAngleDegrees = 87.2;
  public static final double minAngleDegrees = 56.42;
  public static final double angleThresholdDeg = 1.0;

  // Physical constants
  public static final double motorRotationsPerHoodRotationRatio = 96.7;
  public static final double momentOfInertiaKilogramMetersSquared = 0.0282591344;
  public static final double radiusMeters = 0.2058096214;

  // Positions
  public static final double hoodAnglePassNeutralZone = 56.42;
  public static final double hoodAnglePassOpponentZone = 56.42;
  public static final double hoodAngleShootNear = 66.0;
  public static final double hoodAngleShootMiddle = 61.0;
  public static final double hoodAngleShootTower = 59.0;
  public static final double hoodAngleShootTrench = 59;
}
