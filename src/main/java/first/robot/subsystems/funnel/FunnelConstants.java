package first.robot.subsystems.funnel;

import com.ctre.phoenix6.signals.InvertedValue;

public class FunnelConstants {
  // Motor constants
  public static final int funnelMotorID = 13;
  public static final int indexerMotorID = 14;
  public static final InvertedValue funnelInverted = InvertedValue.Clockwise_Positive;
  public static final InvertedValue indexerInverted = InvertedValue.Clockwise_Positive;
  public static final double funnelStatorLimit = 80;
  public static final double funnelSupplyLimit = 40;
  public static final double indexerStatorLimit = 120;
  public static final double indexerSupplyLimit = 40;

  // Physical constants
  public static final double motorRotationsPerFunnelRotationRatio = 2.0;
  public static final double motorRotationsPerIndexerRotationRatio = 2.0;
  // TODO: Set real values
  public static final double funnelMomentOfInertiaKilogramMetersSquared = 0.000025;
  public static final double indexerMomentOfInertiaKilogramMetersSquared = 0.000025;

  // Voltage constants
  public static final double funnelSlowVolts = 0;
  public static final double funnelFastVolts = 8;
  public static final double funnelPassingVolts = 12;

  public static final double indexerSlowVolts = -1;
  public static final double indexerNearVolts = 8.0;
  public static final double indexerMiddleVolts = 8;
  public static final double indexerTowerVolts = 12;
  public static final double indexerTrenchVolts = 9.0;
  public static final double indexerPassingVolts = 12;
}
