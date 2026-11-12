package first.robot.subsystems.vision;

import static org.wpilib.units.Units.Meters;

import org.wpilib.units.measure.Distance;

public class VisionConstants {
  // Limelight Standard Deviation Coefficients
  public static final double xyStdDev = 0.8;
  public static final double rStdDev = 0.1;
  public static final double rMT2StdDev = Double.MAX_VALUE;

  public static final double maxAmbiguity = 0.8;
  public static final Distance maxDistToCamera = Meters.of(6.5);

  // Captures inside this window were solved with the heading from before setPose.
  public static final double poseResetBlackoutSeconds = 0.04;

  public static final String[] limelightNames = {
    "limelight-back", "limelight-left", "limelight-right"
  };
}
