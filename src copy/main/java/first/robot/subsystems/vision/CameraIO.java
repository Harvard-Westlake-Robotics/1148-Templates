package first.robot.subsystems.vision;

import static org.wpilib.units.Units.Meters;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.units.measure.Distance;

public interface CameraIO {
  @AutoLog
  public static class TimestampedPose {
    public Pose2d pose;
    public double timestamp;
    public double stdMultiplier;

    public TimestampedPose(Pose2d pose, double timestamp, double stdMultiplier) {
      this.pose = pose;
      this.timestamp = timestamp;
      this.stdMultiplier = stdMultiplier;
    }

    public TimestampedPose() {}
  }

  @AutoLog
  public static class CameraIOInputs {
    public int tagCount = 0;
    public double ambiguity = 0.0;
    public Distance distToCamera = Meters.of(0.0);
    public Pose2d estimatedPose;
  }

  public default void updateInputs(CameraIOInputs inputs) {}

  public default void updateElasticPose() {}

  public default TimestampedPose getTimestampedPose() {
    return null;
  }
}
