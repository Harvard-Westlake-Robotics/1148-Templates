// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package first.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import first.robot.commands.DriveCommands;
import first.robot.commands.ShooterCommand;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.DriveConstants;
import first.robot.subsystems.drive.GyroIO;
import first.robot.subsystems.drive.GyroIOPigeon2;
import first.robot.subsystems.drive.ModuleIO;
import first.robot.subsystems.drive.ModuleIOSim;
import first.robot.subsystems.drive.ModuleIOTalonFX;
import first.robot.subsystems.funnel.Funnel;
import first.robot.subsystems.hood.Hood;
import first.robot.subsystems.intake.Intake;
import first.robot.subsystems.intake.IntakeConstants;
import first.robot.subsystems.shooter.Shooter;
import first.robot.subsystems.vision.Vision;
import first.robot.subsystems.wrist.IntakeWrist;
import first.robot.subsystems.wrist.WristConstants;

import org.littletonrobotics.junction.networktables.LoggedNetworkChooser;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.sysid.SysIdRoutine;
import org.wpilib.driverstation.GenericHID;
import org.wpilib.command2.button.CommandNiDsPS5Controller;
import org.wpilib.command2.button.CommandNiDsXboxController;
import org.wpilib.driverstation.XboxController;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  // Subsystems
  public final Drive drive;
  public final Funnel funnel;
  public final Intake intake;
  public final Shooter shooter;
  public final Hood hood;
  public final IntakeWrist intakeWrist;
  public final Vision vision;

  // Commands
  public static ShooterCommand shooterCommand;

  // Controllers
  public static final CommandNiDsXboxController operator = new CommandNiDsXboxController(1);
  public static final CommandNiDsPS5Controller driver = new CommandNiDsPS5Controller(0);

  // Dashboard inputs
  private final LoggedNetworkChooser<Command> autoChooser;
  // private final LoggedNetworkChooser<Command> songChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    switch (currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        vision = Vision.getInstance();
        drive = new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(DriveConstants.FrontLeft),
            new ModuleIOTalonFX(DriveConstants.FrontRight),
            new ModuleIOTalonFX(DriveConstants.BackLeft),
            new ModuleIOTalonFX(DriveConstants.BackRight),
            vision);
        funnel = Funnel.getInstance(Mode.REAL);
        intake = Intake.getInstance(Mode.REAL);
        shooter = Shooter.getInstance(Mode.REAL);
        hood = Hood.getInstance(Mode.REAL);
        intakeWrist = IntakeWrist.getInstance(Mode.REAL);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        vision = Vision.getInstance();
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIOSim(DriveConstants.FrontLeft),
            new ModuleIOSim(DriveConstants.FrontRight),
            new ModuleIOSim(DriveConstants.BackLeft),
            new ModuleIOSim(DriveConstants.BackRight),
            vision);
        funnel = Funnel.getInstance(Mode.SIM);
        intake = Intake.getInstance(Mode.SIM);
        shooter = Shooter.getInstance(Mode.SIM);
        hood = Hood.getInstance(Mode.SIM);
        intakeWrist = IntakeWrist.getInstance(Mode.SIM);
        break;

      default:
        // Replayed robot, disable IO implementations
        vision = Vision.getInstance();
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            vision);
        funnel = Funnel.getInstance(Mode.REPLAY);
        intake = Intake.getInstance(Mode.REPLAY);
        shooter = Shooter.getInstance(Mode.REPLAY);
        hood = Hood.getInstance(Mode.REPLAY);
        intakeWrist = IntakeWrist.getInstance(Mode.REPLAY);
        break;
    }

    // Set up named commands for pathplanner
    NamedCommands.registerCommand(
        "WaitUntilShooterReady",
        Commands.waitUntil(() -> shooterCommand.isReady()).withTimeout(1.0));
    NamedCommands.registerCommand(
        "Shoot",
        Commands.runOnce(
            () -> {
              shooterCommand.runFunnel();
            }));
    NamedCommands.registerCommand(
        "AimNear",
        Commands.runOnce(
            () -> {
              shooterCommand.shootNear();
            }));
    NamedCommands.registerCommand(
        "AimMiddle",
        Commands.runOnce(
            () -> {
              shooterCommand.shootMiddle();
            }));
    NamedCommands.registerCommand(
        "AimTower",
        Commands.runOnce(
            () -> {
              shooterCommand.shootTower();
            }));
    NamedCommands.registerCommand(
        "AimTrench",
        Commands.runOnce(
            () -> {
              shooterCommand.shootTrench();
            }));
    NamedCommands.registerCommand(
        "AutoAim",
        Commands.runOnce(
            () -> {
              shooterCommand.setAutoShooting(true);
            }));
    NamedCommands.registerCommand(
        "StopShooter",
        Commands.runOnce(
            () -> {
              shooterCommand.stop();
            }));
    NamedCommands.registerCommand(
        "ResetShooterToDefault",
        Commands.runOnce(
            () -> {
              shooterCommand.reset();
            }));

    NamedCommands.registerCommand(
        "DeployIntake",
        Commands.runOnce(
            () -> {
                intakeWrist.isDown = true;
              intakeWrist.goToPosition(WristConstants.deployedPositionRotations);
              intake.runVoltage(IntakeConstants.fastIntakeVoltage);
            },
            intakeWrist,
            intake)
            .andThen(Commands.waitUntil(() -> intakeWrist.isDeployed()).withTimeout(3.0))
            .andThen(Commands.runOnce(() -> {
              intakeWrist.runVoltage(WristConstants.intakeDeployVoltage);
            }, intakeWrist)));
    // Commands.runOnce(() -> {
    // intakeWrist.runVoltage(WristConstants.intakeManualDeployVoltage);
    // intake.runVoltage(IntakeConstants.fastIntakeVoltage);
    // }, intakeWrist, intake));

    NamedCommands.registerCommand("FirstDeployIntake", Commands.runOnce(() -> {
    intakeWrist.isDown = true;
      intakeWrist.runVoltage(WristConstants.firstDeployVoltage);
      intake.runVoltage(IntakeConstants.fastIntakeVoltage);
    }, intakeWrist, intake).andThen(Commands.waitUntil(() -> intakeWrist.isDeployed()).withTimeout(3.0))
        .andThen(Commands.runOnce(() -> {
          intakeWrist.runVoltage(WristConstants.intakeDeployVoltage);
        }, intakeWrist)));

    NamedCommands.registerCommand(
        "LiftIntake",
        Commands.runOnce(
            () -> {
                intakeWrist.isDown = false;
              intakeWrist.goToPosition(WristConstants.stowedPositionRotations);
              // intake.runVoltage(0);
            },
            intakeWrist));
    // Commands.runOnce(() -> {
    // intakeWrist.runVoltage(WristConstants.intakeManualLiftVoltage);
    // }, intakeWrist));

    NamedCommands.registerCommand(
        "LiftIntakeForShoot",
        Commands.runOnce(
            () -> {
                intakeWrist.isDown = false;
              intakeWrist.runVoltage(WristConstants.shootingVoltage);
              intake.runVoltage(IntakeConstants.fastIntakeVoltage);
            },
            intakeWrist, intake));
    
    NamedCommands.registerCommand("LiftIntakeHalfUp", Commands.runOnce(() -> {
        intakeWrist.isDown = false;
      intakeWrist.goToPosition(WristConstants.halfUpPositionRotations);
      intake.runVoltage(IntakeConstants.fastIntakeVoltage);
    }, intakeWrist, intake));

    // DO NOT UNCOMMENT THIS, BECAUSE THEN IT WILL BE BLOCKING In PATHPLANNER WHICH
    // IS VERY BAD

    // .andThen(Commands.waitUntil(() -> intakeWrist.isStowed()))
    // .andThen(
    // Commands.runOnce(
    // () -> {
    // intakeWrist.goToPosition(WristConstants.stowedPositionRotations);
    // intake.runVoltage(0);
    // },
    // intake,
    // intake)));
    NamedCommands.registerCommand(
        "StopDrive",
        Commands.runOnce(
            () -> {
              drive.stop();
            },
            drive));
    NamedCommands.registerCommand(
        "AutoRotate",
        Commands.run(
            () -> {
              drive.autoRotate(new ChassisVelocities(), drive.rotateToHub.get());
            },
            drive).beforeStarting(
                () -> drive.rotationController.reset(
                    drive.getRotation().getRadians(), drive.getChassisVelocities().omega))
            .until(() -> drive.isAutoRotated())
            .withTimeout(1.5).andThen(Commands.runOnce(() -> {
              drive.stop();
            }, drive)));

    // Set up auto routines
    autoChooser = new LoggedNetworkChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.add(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.add(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.add(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.FORWARD));
    autoChooser.add(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.REVERSE));
    autoChooser.add(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.FORWARD));
    autoChooser.add(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.REVERSE));

    // songChooser = new LoggedNetworkChooser<>("Orchestra Song Chooser");
    // for (int song = 0; song < OrchestraUtils.allSongs.length; song++) {
    // final int song1 = song;
    // songChooser.add(
    // OrchestraUtils.allSongs[song],
    // Commands.runOnce(
    // () -> {
    // OrchestraUtils.songSelected = song1;
    // Telemetry.log(
    // "DB/String 1", "Selected: " + OrchestraUtils.allSongs[song1]);
    // }));
    // }

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * org.wpilib.driverstation.Joystick} or {@link XboxController}), and then
   * passing it to a {@link
   * org.wpilib.command2.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    shooterCommand = new ShooterCommand();
    shooter.setDefaultCommand(shooterCommand);

    // normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            drive.rotateToZoneTarget,
            () -> shooterCommand.isAutoRotating()));

    // Configure orchestra
    // configureOrchestra();

    // Assign controls in ControlMap
    ControlMap.getInstance().configurePreset1(operator, driver);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  // public void configureOrchestra() {
  // // ORCHESTRA STUFF
  // int driveMotorIds[] = { 1, 2, 3, 4, 5, 6, 7, 8 };
  // int subsystemMotorIds[] = { 13, 14, 15, 16, 17, 18, 19, 20, 21, 22 };

  // for (int id : driveMotorIds) {
  // OrchestraUtils.orchestra.addInstrument(
  // new TalonFX(id, DriveConstants.kCANBus));
  // }
  // for (int id : subsystemMotorIds) {
  // OrchestraUtils.orchestra.addInstrument(new TalonFX(id, new
  // CANBus(CANPort.CAN_S2)));
  // }
  // Telemetry.log("DB/String 0", "Playing: ");
  // Telemetry.log("DB/String 1", "Selected: " + OrchestraUtils.allSongs[0]);
  // OrchestraUtils.orchestra.loadMusic("music/" + OrchestraUtils.allSongs[0]);
  // }
}
