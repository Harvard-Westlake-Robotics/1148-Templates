package first.robot.subsystems.vision;

import static org.wpilib.units.Units.Meters;

import com.limelightvision.IMUMode;
import com.limelightvision.Limelight;
import com.limelightvision.PoseEstimate;
import com.limelightvision.PoseEstimateType;

import first.robot.subsystems.drive.Drive;
import org.wpilib.math.geometry.Rotation2d;

public class CameraIOLimelight implements CameraIO {
  private final Limelight camera;

  private boolean updateElasticPose = false;

  private boolean useMegaTag2 = false;

  public void setUseMegaTag2(boolean useMegaTag2) {
    this.useMegaTag2 = useMegaTag2;
  }

  private PoseEstimate currentPose;

  public CameraIOLimelight(String name) {
    this.camera = new Limelight(name);
    currentPose = new PoseEstimate();
  }

  public String getName() {
    return camera.getName();
  }

  @Override
  public void updateInputs(CameraIOInputs inputs) {
    if (useMegaTag2) {
      currentPose = camera.getPoseEstimate(PoseEstimateType.MT2_WPIBLUE);
    } else {
      currentPose = camera.getPoseEstimate(PoseEstimateType.MT1_WPIBLUE);
    }
    if (currentPose == null) {
      return;
    }
    inputs.tagCount = currentPose.fieldedTagCount;
    if (currentPose.fieldedTagCount != 0) {
      inputs.ambiguity = currentPose.rawFiducials[0].ambiguity;
      inputs.distToCamera = Meters.of(currentPose.rawFiducials[0].getDistanceToCamera());
    }
    inputs.estimatedPose = currentPose.pose;
    if (updateElasticPose) {
      Drive.getInstance().field.getObject(camera.getName() + " pose").setPose(inputs.estimatedPose);
      updateElasticPose = false;
    }
  }

  @Override
  public void updateElasticPose() {
    updateElasticPose = true;
  }

  public void setThrottle(int throttle) {
    camera.setThrottle(throttle);
  }

  public void setRobotOrientation(Rotation2d yaw) {
    camera.setRobotOrientation(yaw.getDegrees(), 0, 0, 0, 0, 0, true);
  }

  public void setIMUMode(IMUMode mode) {
    camera.setIMUMode(mode);
  }

  public void recordMatch(boolean isAuto) {
    if (isAuto) camera.triggerRewindCapture(20);
    else camera.triggerRewindCapture(140);
  }

  @Override
  public TimestampedPose getTimestampedPose() {
    if (!shouldAcceptPose(currentPose)) {
      return null;
    }

    return new TimestampedPose(
        currentPose.pose,
        currentPose.timestampSeconds,
        Math.max(Math.pow(currentPose.avgTagDistanceMeters, 1.8), 0.5)
            / currentPose.fieldedTagCount
            * currentPose.rawFiducials[0].ambiguity);
  }

  private boolean shouldAcceptPose(PoseEstimate pose) {
    if (pose == null) {
      return false;
    }
    if (pose.fieldedTagCount == 0 || (!useMegaTag2 && pose.fieldedTagCount < 2)) {
      return false;
    }

    if (pose.rawFiducials[0].ambiguity > VisionConstants.maxAmbiguity) {
      return false;
    }

    if (pose.rawFiducials[0].getDistanceToCamera() > VisionConstants.maxDistToCamera.in(Meters)) {
      return false;
    }
    return true;
  }
}
