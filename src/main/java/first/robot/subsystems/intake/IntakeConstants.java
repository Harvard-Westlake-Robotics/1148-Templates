package first.robot.subsystems.intake;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class IntakeConstants {
  // Motor constants
  public static final int motorMasterID = 21;
  public static final int motorFollowerID = 20;
  public static final InvertedValue motorMasterInverted = InvertedValue.Clockwise_Positive;
  public static final MotorAlignmentValue motorFollowerAligned = MotorAlignmentValue.Opposed;
  public static final double statorLimit = 80;
  public static final double supplyLimit = 80;
  public static final double supplyLowerLimit = 40;
  public static final double supplyLowerTime = 2.0;

  // Physical constants
  public static final double motorRotationsPerIntakeRotationRatio = 2;
  public static final double momentOfInertiaKilogramMetersSquared = 0.0000686998;

  public static final double fastIntakeVoltage = 12;
  public static final double slowIntakeVoltage = 0.5;
  public static final double reverseIntakeVoltage = -12.0;
}
