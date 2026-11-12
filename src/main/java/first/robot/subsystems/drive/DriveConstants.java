// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package first.robot.subsystems.drive;

import static org.wpilib.units.Units.*;

import org.wpilib.hardware.bus.CANPort;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.*;

public class DriveConstants {
    // The steer motor uses any SwerveModule.SteerRequestType control request with
    // the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput

    // ================================= PID Tuning
    // =================================

    // Swerve Steer PID Values
    public static final double kSteerP = 45;
    public static final double kSteerI = 0;
    public static final double kSteerD = 0;
    public static final double kSteerS = 0;
    public static final double kSteerV = 0;
    public static final double kSteerA = 0;
    public static final StaticFeedforwardSignValue kStaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    // Swerve Drive PID Values
    public static final double kDriveP = 0.15564 * 7.03;
    public static final double kDriveI = 0;
    public static final double kDriveD = 0;
    public static final double kDriveS = 0.082778;
    public static final double kDriveV = 0.13555 * 7.03;
    public static final double kDriveA = 0.034453 * 7.03;

    // Swerve Angular PID Values (Position)
    public static final double kAngleP = 6.7;
    public static final double kAngleI = 0.1;
    public static final double kAngleD = 0.15;
    public static final double kAngleIZoneDeg = 20.0;

    // Swerve Position PID Values
    public static final double kPositionP = 25;
    public static final double kPositionI = 0.1;
    public static final double kPositionD = 50;

    // Sim PID Values and Settings
    // TODO: Tune
    public static final double DRIVE_SIM_KP = 0.05;
    public static final double DRIVE_SIM_KD = 0.0;
    public static final double DRIVE_SIM_KS = 0.0;
    public static final double DRIVE_SIM_KV_ROT = 0.91035; // Same units as TunerConstants: (volt * secs) / rotation
    public static final double DRIVE_SIM_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_SIM_KV_ROT);
    public static final double TURN_SIM_KP = 8.0;
    public static final double TURN_SIM_KD = 0.0;
    public static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
    public static final DCMotor TURN_GEARBOX = DCMotor.getKrakenX44Foc(1);

    // Position, Velocity
    // TODO: Tune
    public static final double[] DRIVE_SIM_STDDEVS = { 0.01, 0.1 };
    public static final double[] TURN_SIM_STDDEVS = { 0.01, 0.1 };

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // PathPlanner PIDs
    public static double PP_ROTATION_P = 6.0;
    public static double PP_ROTATION_I = 0.0;
    public static double PP_ROTATION_D = 1.4;
    public static double PP_TRANSLATION_P = 6.0;
    public static double PP_TRANSLATION_I = 0;
    public static double PP_TRANSLATION_D = 0.5;

    // ================================= Hardware Tuning
    // =================================

    public static final double ROBOT_MASS_KG = 62.20248248; // KG, NOT LB
    public static final double ROBOT_MOI = 8.2916048504; // KG*M^2 , NOT LB*IN^2
    public static final double WHEEL_COF = 1.2;
    public static final double ROBOT_LENGTH_X_BUMPER = 33.75;
    public static final double ROBOT_LENGTH_Y_BUMPER = 33.875;

    // The type of motor used for the drive motor
    private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
    // The type of motor used for the drive motor
    private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to
    // RemoteCANcoder
    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    // Current limits

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    // This is also the stator current limit for the drive motors
    public static final Current kSlipCurrent = Amps.of(90);

    private static final Current driveSupplyCurrentLimit = Amps.of(80);
    private static final Current driveSupplyCurrentLowerLimit = Amps.of(40);
    private static final double driveSupplyCurrentLowerTime = 2.0;

    private static final Current steerStatorCurrentLimit = Amps.of(40);

    // Theoretical linear free speed (m/s) at 12 V applied output;
    public static final LinearVelocity kMaxLinearSpeed = MetersPerSecond.of(4.15); // 4.39
    public static final LinearAcceleration kMaxLinearAcceleration = MetersPerSecondPerSecond.of(13.73); // 13.73

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3.375;

    public static final double kDriveGearRatio = 7.03;
    public static final double kSteerGearRatio = 287.0 / 11.0;
    private static final Distance kWheelRadius = Inches.of(1.962754845085376500000000000);

    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final int kPigeonId = 30;

    // These are only used for simulation
    public static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.004);
    public static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.025);
    // Simulated voltage necessary to overcome friction
    private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

    // Front Left
    private static final int kFrontLeftDriveMotorId = 5;
    private static final int kFrontLeftSteerMotorId = 6;
    private static final int kFrontLeftEncoderId = 11;
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.146728515625);
    private static final boolean kFrontLeftSteerMotorInverted = false;
    private static final boolean kFrontLeftEncoderInverted = false;

    private static final Distance kFrontLeftXPos = Inches.of(10.875);
    private static final Distance kFrontLeftYPos = Inches.of(10.875);

    // Front Right
    private static final int kFrontRightDriveMotorId = 7;
    private static final int kFrontRightSteerMotorId = 8;
    private static final int kFrontRightEncoderId = 12;
    private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.314208984375);
    private static final boolean kFrontRightSteerMotorInverted = false;
    private static final boolean kFrontRightEncoderInverted = false;

    private static final Distance kFrontRightXPos = Inches.of(10.875);
    private static final Distance kFrontRightYPos = Inches.of(-10.875);

    // Back Left
    private static final int kBackLeftDriveMotorId = 1;
    private static final int kBackLeftSteerMotorId = 2;
    private static final int kBackLeftEncoderId = 9;
    private static final Angle kBackLeftEncoderOffset = Rotations.of(0.292236328125);
    private static final boolean kBackLeftSteerMotorInverted = false;
    private static final boolean kBackLeftEncoderInverted = false;

    private static final Distance kBackLeftXPos = Inches.of(-10.875);
    private static final Distance kBackLeftYPos = Inches.of(10.875);

    // Back Right
    private static final int kBackRightDriveMotorId = 3;
    private static final int kBackRightSteerMotorId = 4;
    private static final int kBackRightEncoderId = 10;
    private static final Angle kBackRightEncoderOffset = Rotations.of(-0.20263671875);
    private static final boolean kBackRightSteerMotorInverted = false;
    private static final boolean kBackRightEncoderInverted = false;

    private static final Distance kBackRightXPos = Inches.of(-10.875);
    private static final Distance kBackRightYPos = Inches.of(-10.875);

    // Gyro offsets
    public static final double gyroYawOffset = 0;
    public static final double gyroPitchOffset = 0.615234;
    public static final double gyroRollOffset = 0.175;

    // ================================= FIELD CONSTANTS
    // =================================

    public static class FieldConstants {
        public static final double FIELD_WIDTH_METERS = 16.513048;
        public static final double FIELD_HEIGHT_METERS = 8.042656;
        public static final double FIELD_BORDER_MARGIN_METERS = 0.05;

        public static final double BLUE_HUB_CENTER_X_METERS = 4.63;
        public static final double BLUE_HUB_CENTER_Y_METERS = 4.035;

        public static final double RED_HUB_CENTER_X_METERS = 11.92;
        public static final double RED_HUB_CENTER_Y_METERS = 4.035;

        public static final double HUB_HEIGHT_METERS = 1.06;

        public static final Pose2d RED_HUB_POS = new Pose2d(RED_HUB_CENTER_X_METERS, RED_HUB_CENTER_Y_METERS,
                new Rotation2d());
        public static final Pose2d BLUE_HUB_POS = new Pose2d(BLUE_HUB_CENTER_X_METERS, BLUE_HUB_CENTER_Y_METERS,
                new Rotation2d());
        public static final Pose2d FIELD_CENTER_POS = new Pose2d(FIELD_WIDTH_METERS / 2, FIELD_HEIGHT_METERS / 2,
                new Rotation2d());

        public static final double CORNER_PASS_INSET_METERS = 3.0;

        public static final double CORNER_PASS_HYSTERESIS_METERS = 0.75;

        // LEFT/RIGHT derived from perspective of DRIVER STATION
        public static final Translation2d BLUE_LEFT_CORNER_POS = new Translation2d(
                CORNER_PASS_INSET_METERS,
                FIELD_HEIGHT_METERS - CORNER_PASS_INSET_METERS);

        public static final Translation2d BLUE_RIGHT_CORNER_POS = new Translation2d(
                CORNER_PASS_INSET_METERS,
                CORNER_PASS_INSET_METERS);

        public static final Translation2d RED_LEFT_CORNER_POS = new Translation2d(
                FIELD_WIDTH_METERS - CORNER_PASS_INSET_METERS,
                CORNER_PASS_INSET_METERS);

        public static final Translation2d RED_RIGHT_CORNER_POS = new Translation2d(
                FIELD_WIDTH_METERS - CORNER_PASS_INSET_METERS,
                FIELD_HEIGHT_METERS - CORNER_PASS_INSET_METERS);
    }

    // ================================= END OF TUNER CONSTANTS
    // =================================

    // ================================= Robot Software Configs
    // =================================

    public static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(kSteerP)
            .withKI(kSteerI)
            .withKD(kSteerD)
            .withKS(kSteerS)
            .withKV(kSteerV)
            .withKA(kSteerA)
            .withStaticFeedforwardSign(kStaticFeedforwardSign);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    public static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(kDriveP)
            .withKI(kDriveI)
            .withKD(kDriveD)
            .withKS(kDriveS)
            .withKV(kDriveV)
            .withKA(kDriveA);

    // Initial configs for the drive and steer motors and the azimuth encoder; these
    // cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API
    // documentation.
    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(
                    new CurrentLimitsConfigs()
                            .withSupplyCurrentLimitEnable(true)
                            .withSupplyCurrentLimit(driveSupplyCurrentLimit)
                            .withSupplyCurrentLimitEnable(true)
                            .withSupplyCurrentLowerLimit(driveSupplyCurrentLowerLimit)
                            .withSupplyCurrentLowerTime(driveSupplyCurrentLowerTime)
                            .withStatorCurrentLimitEnable(true)
                            .withStatorCurrentLimit(kSlipCurrent))
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(
                    new CurrentLimitsConfigs()
                            // Swerve azimuth does not require much torque output, so we can
                            // set a
                            // relatively low stator current limit to help avoid brownouts
                            // without
                            // impacting performance.
                            .withStatorCurrentLimitEnable(true)
                            .withStatorCurrentLimit(steerStatorCurrentLimit))
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));

    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private static final Pigeon2Configuration pigeonConfigs = new Pigeon2Configuration()
            .withMountPose(
                    new MountPoseConfigs()
                            .withMountPoseYaw(gyroYawOffset)
                            .withMountPosePitch(gyroPitchOffset)
                            .withMountPoseRoll(gyroRollOffset));

    // CAN bus that the devices are located on;
    // All swerve devices must share the same CAN bus
    public static final CANBus kCANBus = new CANBus(CANPort.CAN_S0);

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withNetwork(kCANBus)
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(pigeonConfigs);

    // ================================= Module Configurations
    // =================================

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withCouplingGearRatio(kCoupleRatio)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kMaxLinearSpeed)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage);

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft = ConstantCreator
            .createModuleConstants(
                    kFrontLeftSteerMotorId,
                    kFrontLeftDriveMotorId,
                    kFrontLeftEncoderId,
                    kFrontLeftEncoderOffset,
                    kFrontLeftXPos,
                    kFrontLeftYPos,
                    kInvertLeftSide,
                    kFrontLeftSteerMotorInverted,
                    kFrontLeftEncoderInverted);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight = ConstantCreator
            .createModuleConstants(
                    kFrontRightSteerMotorId,
                    kFrontRightDriveMotorId,
                    kFrontRightEncoderId,
                    kFrontRightEncoderOffset,
                    kFrontRightXPos,
                    kFrontRightYPos,
                    kInvertRightSide,
                    kFrontRightSteerMotorInverted,
                    kFrontRightEncoderInverted);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft = ConstantCreator
            .createModuleConstants(
                    kBackLeftSteerMotorId,
                    kBackLeftDriveMotorId,
                    kBackLeftEncoderId,
                    kBackLeftEncoderOffset,
                    kBackLeftXPos,
                    kBackLeftYPos,
                    kInvertLeftSide,
                    kBackLeftSteerMotorInverted,
                    kBackLeftEncoderInverted);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight = ConstantCreator
            .createModuleConstants(
                    kBackRightSteerMotorId,
                    kBackRightDriveMotorId,
                    kBackRightEncoderId,
                    kBackRightEncoderOffset,
                    kBackRightXPos,
                    kBackRightYPos,
                    kInvertRightSide,
                    kBackRightSteerMotorInverted,
                    kBackRightEncoderInverted);

    public static final double DRIVE_BASE_RADIUS = Math.max(
            Math.max(
                    Math.hypot(DriveConstants.FrontLeft.LocationX, DriveConstants.FrontLeft.LocationY),
                    Math.hypot(DriveConstants.FrontRight.LocationX, DriveConstants.FrontRight.LocationY)),
            Math.max(
                    Math.hypot(DriveConstants.BackLeft.LocationX, DriveConstants.BackLeft.LocationY),
                    Math.hypot(DriveConstants.BackRight.LocationX, DriveConstants.BackRight.LocationY)));

    // ================================= PathPlanner Configs
    // =================================

    public static RobotConfig PPConfig = new RobotConfig(
                ROBOT_MASS_KG,
                ROBOT_MOI,
                new ModuleConfig(
                        FrontLeft.WheelRadius,
                        kMaxLinearSpeed.in(MetersPerSecond),
                        WHEEL_COF,
                        DCMotor.getKrakenX60Foc(1).withReduction(FrontLeft.DriveMotorGearRatio),
                        FrontLeft.SlipCurrent,
                        1),
                Drive.getModuleTranslations());

    // ================================= Extra Configurations
    // =================================

    public static final double ODOMETRY_FREQUENCY = kCANBus.isNetworkFD() ? 250.0 : 100.0;
    public static final double MAX_YAW_RATE_DEGREES_PER_SEC = 520.0;
}
