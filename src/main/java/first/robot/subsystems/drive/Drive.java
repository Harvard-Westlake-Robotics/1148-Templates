// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package first.robot.subsystems.drive;

import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import first.robot.subsystems.drive.DriveConstants.FieldConstants;
import first.robot.subsystems.shooter.ShooterConstants;
import first.robot.subsystems.vision.CameraIO.TimestampedPose;
import first.robot.subsystems.vision.Vision;
import first.robot.subsystems.vision.VisionConstants;
import first.robot.util.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.sysid.SysIdRoutine;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.util.Units;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.system.Timer;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.tunable.Tunables;

public class Drive extends SubsystemBase {
  private static Drive instance;

  public static Drive getInstance() {
    return instance;
  }

  public final ProfiledPIDController rotationController;
  private final ProfiledPIDController positionXController;
  private final ProfiledPIDController positionYController;

  public final Field2d field = new Field2d();

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;

  // Passing hysteresis tracking
  private static boolean cornerPassPlusY = false; // PlusY = Red Right, Blue Left

  private final Vision vision;
  @AutoLogOutput
  private boolean visionActive = false;

  public boolean isVisionActive() {
    return visionActive;
  }

  public void setVisionActive(boolean visionActive) {
    this.visionActive = visionActive;
  }

  public Pose2d getPhaseShiftedPose() {
    return Drive.getInstance()
        .getPose()
        .plus(
            Drive.getInstance()
                .getChassisVelocities()
                .toTwist2d(ShooterConstants.sotmPhaseShiftTimeSeconds)
                .exp());
  }

  // MUTATES cornerPassPlusY
  public Translation2d getCornerPos() { 
    double threshold = FieldConstants.FIELD_HEIGHT_METERS / 2
        + FieldConstants.CORNER_PASS_HYSTERESIS_METERS * (cornerPassPlusY ? -1 : 1);

    cornerPassPlusY = getPhaseShiftedPose().getY() >= threshold;

    if (MatchState.getAlliance().isPresent() && MatchState.getAlliance().get() == Alliance.RED) {
      return cornerPassPlusY
          ? FieldConstants.RED_RIGHT_CORNER_POS // Assignments inverted based on alliance color
          : FieldConstants.RED_LEFT_CORNER_POS;
    } else {
      return cornerPassPlusY
          ? FieldConstants.BLUE_LEFT_CORNER_POS
          : FieldConstants.BLUE_RIGHT_CORNER_POS;
    }
  }


  public final Supplier<Rotation2d> rotateToHub = () -> {
    Pose2d pos = getPhaseShiftedPose();
    Pose2d hubPos = MatchState.getAlliance().isPresent() && MatchState.getAlliance().get() == Alliance.RED
        ? FieldConstants.RED_HUB_POS
        : FieldConstants.BLUE_HUB_POS;
    // double d = pos.getTranslation().getDistance(hubPos.getTranslation());
    // double t = 0;
    Pose2d newPos = pos;

    // // Optimize until close enough
    // int iterations = 0;
    // do {
    //   t = ShooterConstants.timeOfFlightMap.get(d);

    //   ChassisVelocities velocities = Drive.getInstance().getChassisVelocities();

    //   // Assume 0 angular velocity affecting the balls, because if there is angular
    //   // velocity it would mess up the shot, since the balls at the edges of the
    //   // shooter would collide with the balls at the center
    //   newPos = pos.plus(
    //       new Transform2d(
    //           velocities.vx * t, velocities.vy * t, new Rotation2d(velocities.omega *
    //               t)));
    //   d = newPos.getTranslation().getDistance(hubPos.getTranslation());
    //   iterations++;
    //   if (iterations >= 10)
    //     break;
    // } while (Math
    //     .abs(t - ShooterConstants.timeOfFlightMap.get(d)) > ShooterConstants.sotmOptimizationTimeThresholdSeconds);

    Translation2d hub = hubPos.getTranslation();
    Translation2d robotToHub = hub.minus(newPos.getTranslation());

    if (robotToHub.getNorm() < 1e-6) {
      return pos.getRotation();
    }
    
    return new Rotation2d(robotToHub.getAngle().get().getRadians() + Math.PI);
  };

  public final Supplier<Rotation2d> rotateToCorner = () -> {
    Pose2d pos = Drive.getInstance()
        .getPose()
        .plus(
            Drive.getInstance()
                .getChassisVelocities()
                .toTwist2d(ShooterConstants.sotmPhaseShiftTimeSeconds)
                .exp());

    Translation2d cornerPos = getCornerPos();
    Translation2d robotToCorner = cornerPos.minus(pos.getTranslation());

    if (robotToCorner.getNorm() < 1e-6) {
      return pos.getRotation();
    }

    return new Rotation2d(robotToCorner.getAngle().get().getRadians() + Math.PI);
  };

  public final Supplier<Rotation2d> rotateToZoneTarget = () -> isInShootingRange()
      ? rotateToHub.get()
      : rotateToCorner.get();

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = Rotation2d.ZERO;
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation,
      lastModulePositions, Pose2d.ZERO);

  // Vision captures at or after this time were taken with the post-reset heading.
  private double acceptVisionAfter = Double.NEGATIVE_INFINITY;

  @AutoLogOutput
  private double sdMultiplier = 1.0;

  public void setSdMultiplier(double sdMultiplier) {
    this.sdMultiplier = sdMultiplier;
  }

  private double lastFieldUpdate = 0;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      Vision vision) {
    // Prevents double initialization since this constructor is called directly in
    // RobotContainer
    Drive.instance = this;

    this.gyroIO = gyroIO;
    this.vision = vision;
    modules[0] = new Module(flModuleIO, 0, DriveConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, DriveConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, DriveConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, DriveConstants.BackRight);

    rotationController = new ProfiledPIDController(
        DriveConstants.kAngleP,
        DriveConstants.kAngleI,
        DriveConstants.kAngleD,
        new TrapezoidProfile.Constraints(
            getMaxAngularSpeedRadPerSec(), getMaxAngularAccelerationRadPerSecSquared()));
    rotationController.setIZone(Units.degreesToRadians(DriveConstants.kAngleIZoneDeg));
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(Units.degreesToRadians(1));

    positionXController = new ProfiledPIDController(
        DriveConstants.kDriveP,
        DriveConstants.kDriveI,
        DriveConstants.kDriveD,
        new TrapezoidProfile.Constraints(
            getMaxLinearSpeedMetersPerSec(), getMaxLinearAccelerationMetersPerSecSquared()));
    positionXController.setTolerance(0.1, 0.05);

    positionYController = new ProfiledPIDController(
        DriveConstants.kDriveP,
        DriveConstants.kDriveI,
        DriveConstants.kDriveD,
        new TrapezoidProfile.Constraints(
            getMaxLinearSpeedMetersPerSec(), getMaxLinearAccelerationMetersPerSecSquared()));
    positionYController.setTolerance(0.1, 0.05);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisVelocities,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(
                DriveConstants.PP_TRANSLATION_P,
                DriveConstants.PP_TRANSLATION_I,
                DriveConstants.PP_TRANSLATION_D),
            new PIDConstants(
                DriveConstants.PP_ROTATION_P,
                DriveConstants.PP_ROTATION_I,
                DriveConstants.PP_ROTATION_D)),
        DriveConstants.PPConfig,
        () -> MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

    // Log field
    Tunables.publish("Field", field);

    // Setup NetworkTables communication
    NetworkCommunicator.getInstance().init();
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (RobotState.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }

      // Log empty setpoint states when disabled
      Logger.recordOutput("SwerveVelocities/Setpoints", new SwerveModuleVelocity[] {});
      Logger.recordOutput("SwerveVelocities/SetpointsOptimized", new SwerveModuleVelocity[] {});
      vision.setThrottle(200);
      vision.setUseMegaTag2(false);
    } else {
      vision.setThrottle(0);
      vision.setUseMegaTag2(true);
    }

    // Update odometry
    double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] = new SwerveModulePosition(
            modulePositions[moduleIndex].distance - lastModulePositions[moduleIndex].distance,
            modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    TimestampedPose[] timestampedPoses = vision.getTimestampedPoses();

    boolean middleTags = false;
    boolean acceptingTags = false;

    for (TimestampedPose pose : timestampedPoses) {
      if (pose != null) {
        middleTags = true;
        if (pose.timestamp < acceptVisionAfter) {
          continue;
        }
        if (shouldAcceptPose(pose.pose) && visionActive) {
          acceptingTags = true;
          addVisionMeasurement(
              pose.pose,
              pose.timestamp,
              VecBuilder.fill(
                  pose.stdMultiplier * VisionConstants.xyStdDev * (vision.shootingMode ? 0.5 : 1.0),
                  pose.stdMultiplier * VisionConstants.xyStdDev * (vision.shootingMode ? 0.5 : 1.0),
                  pose.stdMultiplier
                      * (RobotState.isDisabled()
                          ? VisionConstants.rStdDev
                          : VisionConstants.rMT2StdDev)));
        }
      }
    }

    vision.setRobotOrientation(getPose().getRotation());

    Telemetry.log("MiddleTags", middleTags);
    Telemetry.log("AcceptingTags", acceptingTags);
    Logger.recordOutput("Odometry/AcceptVisionAfter", acceptVisionAfter);

    double timestamp = Timer.getTimestamp();
    if (timestamp - lastFieldUpdate > 0.1) {
      // field.setRobotPose(getPose());
      field.getObject("robot").setPose(getPose());
      vision.updateElasticPose();
      lastFieldUpdate = Timer.getTimestamp();
    }
  }

  @Override
  public void simulationPeriodic() {
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisVelocities speeds) {
    // Calculate module setpoints
    speeds = speeds.discretize(0.02);
    SwerveModuleVelocity[] setpointStates = kinematics.toSwerveModuleVelocities(speeds);
    setpointStates = SwerveDriveKinematics.desaturateWheelVelocities(
        setpointStates, DriveConstants.kMaxLinearSpeed);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveVelocities/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisVelocities/Setpoints", speeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveVelocities/SetpointsOptimized", setpointStates);
  }

  /**
   * Rotates the drive to the desired setpoint while maintaining x and y motion.
   *
   * @param rotation Target rotation of the drive.
   */
  public void autoRotate(ChassisVelocities speeds, Rotation2d targetRotation) {
    double current = getRotation().getRadians();
    double target = targetRotation.getRadians();

    double omega = rotationController.calculate(current, target);

    runVelocity(new ChassisVelocities(speeds.vx, speeds.vy, omega));
  }

  @AutoLogOutput
  public boolean isAutoRotated() {
    return rotationController.atGoal();
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisVelocities());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement.
   * The modules will
   * return to their normal orientations the next time a nonzero velocity is
   * requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle().orElse(new Rotation2d());
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  // SysId Commands

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /**
   * Returns the module velocities (turn angles and drive velocities) for all of
   * the modules.
   */
  @AutoLogOutput(key = "SwerveVelocities/Measured")
  private SwerveModuleVelocity[] getModuleVelocities() {
    SwerveModuleVelocity[] velocities = new SwerveModuleVelocity[4];
    for (int i = 0; i < 4; i++) {
      velocities[i] = modules[i].getVelocity();
    }
    return velocities;
  }

  /**
   * Returns the module positions (turn angles and drive positions) for all of the
   * modules.
   */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisVelocities/Measured")
  public ChassisVelocities getChassisVelocities() {
    return kinematics.toChassisVelocities(getModuleVelocities());
  }

  /**
   * Returns the average velocity of the modules in rotations/sec (Phoenix native
   * units).
   */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    acceptVisionAfter = Timer.getTimestamp() + VisionConstants.poseResetBlackoutSeconds;
    vision.setRobotOrientation(pose.getRotation());
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return DriveConstants.kMaxLinearSpeed.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DriveConstants.DRIVE_BASE_RADIUS;
  }

  /** Returns the maximum linear acceleration in meters per sec squared. */
  public double getMaxLinearAccelerationMetersPerSecSquared() {
    return DriveConstants.kMaxLinearAcceleration.in(MetersPerSecondPerSecond);
  }

  /** Returns the maximum angular acceleration in radians per sec squared. */
  public double getMaxAngularAccelerationRadPerSecSquared() {
    return getMaxLinearAccelerationMetersPerSecSquared() / DriveConstants.DRIVE_BASE_RADIUS;
  }

  @AutoLogOutput(key = "Drive/DistanceFromHub")
  public double getDistanceFromHub() {
    Pose2d pos = poseEstimator.getEstimatedPosition();
    Pose2d hubPos = MatchState.getAlliance().isPresent() && MatchState.getAlliance().get() == Alliance.RED
        ? FieldConstants.RED_HUB_POS
        : FieldConstants.BLUE_HUB_POS;
    return pos.getTranslation().getDistance(hubPos.getTranslation());
  }

  @AutoLogOutput(key = "Drive/InShootingRange")
  public boolean isInShootingRange() {
    boolean allianceIsRed = MatchState.getAlliance().isPresent() && MatchState.getAlliance().get() == Alliance.RED;
    return allianceIsRed
        ? poseEstimator.getEstimatedPosition().getX() > FieldConstants.RED_HUB_CENTER_X_METERS
        : poseEstimator.getEstimatedPosition().getX() < FieldConstants.BLUE_HUB_CENTER_X_METERS;
  }

  @AutoLogOutput(key = "Drive/InOpponentShootingRange")
  public boolean isInOpponentShootingRange() {
    boolean allianceIsRed = MatchState.getAlliance().isPresent() && MatchState.getAlliance().get() == Alliance.RED;
    return allianceIsRed
        ? poseEstimator.getEstimatedPosition().getX() < FieldConstants.BLUE_HUB_CENTER_X_METERS
        : poseEstimator.getEstimatedPosition().getX() > FieldConstants.RED_HUB_CENTER_X_METERS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
        new Translation2d(DriveConstants.FrontLeft.LocationX, DriveConstants.FrontLeft.LocationY),
        new Translation2d(DriveConstants.FrontRight.LocationX, DriveConstants.FrontRight.LocationY),
        new Translation2d(DriveConstants.BackLeft.LocationX, DriveConstants.BackLeft.LocationY),
        new Translation2d(DriveConstants.BackRight.LocationX, DriveConstants.BackRight.LocationY)
    };
  }

  @AutoLogOutput(key = "Odometry/Velocity/LinearVelocity")
  private double getLinearVelocity() {
    ChassisVelocities velocities = getChassisVelocities();
    double linearSpeed = Math.hypot(velocities.vx, velocities.vy);
    return linearSpeed;
  }

  @AutoLogOutput(key = "Odometry/Velocity/AngularVelocity")
  private double getAngularVelocity() {
    return gyroInputs.yawVelocityRadPerSec;
  }

  /**
   * Determines if a vision pose should be accepted based on various criteria.
   *
   * @param pose The vision pose
   * @return True if the pose should be accepted, false otherwise
   */
  public boolean shouldAcceptPose(Pose2d pose) {
    // Always accept poses when disabled
    if (RobotState.isDisabled()) {
      return true;
    }

    // Reject if rotating too quickly
    if (Math
        .abs(Units.radiansToDegrees(gyroInputs.yawVelocityRadPerSec)) >= DriveConstants.MAX_YAW_RATE_DEGREES_PER_SEC) {
      return false;
    }

    // Reject if outside field bounds
    if (isOutsideFieldBounds(pose)) {
      return false;
    }

    // Accept the pose
    return true;
  }

  /** Checks if a pose is outside the field boundaries. */
  private boolean isOutsideFieldBounds(Pose2d pose) {
    return pose.getX() < -FieldConstants.FIELD_BORDER_MARGIN_METERS
        || pose.getX() > FieldConstants.FIELD_WIDTH_METERS + FieldConstants.FIELD_BORDER_MARGIN_METERS
        || pose.getY() < -FieldConstants.FIELD_BORDER_MARGIN_METERS
        || pose.getY() > FieldConstants.FIELD_HEIGHT_METERS + FieldConstants.FIELD_BORDER_MARGIN_METERS;
  }

  public void changeCurrentLimitsAndNeutralMode(
      double newStatorLimit,
      double newSupplyLimit,
      double newSupplyLowerLimit,
      double newSupplyLowerTime,
      NeutralModeValue newNeutralMode) {
    for (Module module : modules) {
      module.changeCurrentLimitsAndNeutralMode(
          newStatorLimit, newSupplyLimit, newSupplyLowerLimit, newSupplyLowerTime, newNeutralMode);
    }
  }
}
