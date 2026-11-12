package first.robot.subsystems.shooter;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import first.robot.subsystems.funnel.FunnelConstants;
import first.robot.subsystems.hood.HoodConstants;
import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterConstants {
  // Motor constants
  public static final int motorMasterID = 15;
  public static final int motorFollower1ID = 16;
  public static final int motorFollower2ID = 17;
  public static final int motorFollower3ID = 18;
  public static final InvertedValue motorMasterInverted = InvertedValue.CounterClockwise_Positive;
  public static final MotorAlignmentValue motorFollower1Aligned = MotorAlignmentValue.Aligned;
  public static final MotorAlignmentValue motorFollower2Aligned = MotorAlignmentValue.Opposed;
  public static final MotorAlignmentValue motorFollower3Aligned = MotorAlignmentValue.Opposed;
  // Higher supply limit with a time limit to allow for 50A during spoolup
  public static final double supplyLimit = 60;
  // Time it takes to spool up
  public static final double supplyLimitTime = 2.0;
  // Nominal supply limit
  public static final double supplyLimitLower = 40;
  public static final double statorLimit = 120;

  // PID constants
  public static final double kP = 0.35;
  public static final double kI = 0.1;
  public static final double kD = 0;
  public static final double kS = 0.3 / 4.0;
  public static final double kV = 0.39 / 4.0;
  public static final double kA = 0.0;

  public static final double motionMagicAcceleration = 50;
  public static final double motionMagicJerk = 500;

  // Physical constants
  public static final double motorRotationsPerShooterRotationRatio = 3.0 / 4.0;
  public static final double momentOfInertiaKilogramMetersSquared = 0.0070178471;

  // Velocity Constants
  public static final double nearRPS = 42.0;
  public static final double middleRPS = 46.0;
  public static final double towerRPS = 49.0;
  public static final double trenchRPS = 52.0;
  public static final double defaultRPS = 35.0;

  public static final double passingNeutralRPS = 59.0;
  public static final double passingOpponentRPS = 120.0;

  public static final double velocityThresholdRPS = 0.5;
  public static final double minVelocityRPS = 0.0;
  public static final double maxVelocityRPS = 128.0;

  public static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap shooterVelocityMap =
      new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap indexerVoltageMap =
      new InterpolatingDoubleTreeMap();

  static {
    // Populate map

    // Aarush Near
    ShooterConstants.hoodAngleMap.put(1.533458, 71.0);
    ShooterConstants.shooterVelocityMap.put(1.533458, 40.5);
    ShooterConstants.indexerVoltageMap.put(1.533458, 5.0);
    ShooterConstants.timeOfFlightMap.put(1.533458, 1.0);

    // Grayden Near
    ShooterConstants.hoodAngleMap.put(2.057149700751805, HoodConstants.hoodAngleShootNear);
    ShooterConstants.shooterVelocityMap.put(2.057149700751805, ShooterConstants.nearRPS);
    ShooterConstants.indexerVoltageMap.put(2.057149700751805, FunnelConstants.indexerNearVolts);
    ShooterConstants.timeOfFlightMap.put(2.057149700751805, 1.5);

    // Near-Middle
    ShooterConstants.hoodAngleMap.put(2.415056, 62.0);
    ShooterConstants.shooterVelocityMap.put(2.415056, 45.0);
    ShooterConstants.indexerVoltageMap.put(2.415056, 8.0);
    ShooterConstants.timeOfFlightMap.put(2.415056, 1.75);

    // Middle
    ShooterConstants.hoodAngleMap.put(2.7105968708214836, HoodConstants.hoodAngleShootMiddle);
    ShooterConstants.shooterVelocityMap.put(2.7105968708214836, ShooterConstants.middleRPS);
    ShooterConstants.indexerVoltageMap.put(2.7105968708214836, FunnelConstants.indexerMiddleVolts);
    ShooterConstants.timeOfFlightMap.put(2.7105968708214836, 2.0);

    // Tower
    ShooterConstants.hoodAngleMap.put(3.2850226416347876, HoodConstants.hoodAngleShootTower);
    ShooterConstants.shooterVelocityMap.put(3.2850226416347876, ShooterConstants.towerRPS);
    ShooterConstants.indexerVoltageMap.put(3.2850226416347876, FunnelConstants.indexerTowerVolts);
    ShooterConstants.timeOfFlightMap.put(3.2850226416347876, 2.5);

    // Trench
    ShooterConstants.hoodAngleMap.put(3.532509590921158, HoodConstants.hoodAngleShootTrench);
    ShooterConstants.shooterVelocityMap.put(3.532509590921158, ShooterConstants.trenchRPS);
    ShooterConstants.indexerVoltageMap.put(3.532509590921158, FunnelConstants.indexerTrenchVolts);
    ShooterConstants.timeOfFlightMap.put(3.532509590921158, 3.0);

    // Depot
    ShooterConstants.hoodAngleMap.put(4.193921, 58.0);
    ShooterConstants.shooterVelocityMap.put(4.193921, 56.0);
    ShooterConstants.indexerVoltageMap.put(4.193921, 8.0);
    ShooterConstants.timeOfFlightMap.put(4.193921, 3.5);

    // Corner
    ShooterConstants.hoodAngleMap.put(4.932304, 56.43);
    ShooterConstants.shooterVelocityMap.put(4.932304, 59.0);
    ShooterConstants.indexerVoltageMap.put(4.932304, 9.0);
    ShooterConstants.timeOfFlightMap.put(4.932304, 4.0);
  }

  // Distance between estimated time and more accurate time (next iteration) at
  // which it will stop
  // optimization
  // Don't make this too low, or it will cause loop overruns, and don't make it
  // too high, or else it
  // will miss
  public static final double sotmOptimizationTimeThresholdSeconds = 0.5;
  public static final double sotmPhaseShiftTimeSeconds = 0.02;


  // PASSING INTERPOLATION MAP
  public static final InterpolatingDoubleTreeMap shooterPassingVelocityMap = 
      new InterpolatingDoubleTreeMap();

  static {
    // Near
    ShooterConstants.shooterPassingVelocityMap.put(6.5, 59.0); // approximated distance values

    // Far
    ShooterConstants.shooterPassingVelocityMap.put(14.2, 120.0);
  }
}
