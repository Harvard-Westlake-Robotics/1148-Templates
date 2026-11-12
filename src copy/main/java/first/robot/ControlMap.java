package first.robot;

import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.funnel.Funnel;
import first.robot.subsystems.funnel.FunnelConstants;
import first.robot.subsystems.intake.Intake;
import first.robot.subsystems.intake.IntakeConstants;
import first.robot.subsystems.wrist.IntakeWrist;
import first.robot.subsystems.wrist.WristConstants;

import org.wpilib.command2.Commands;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.command2.button.CommandNiDsPS5Controller;
import org.wpilib.command2.button.CommandNiDsXboxController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

public class ControlMap {
    private static ControlMap instance;

    public static ControlMap getInstance() {
        if (instance == null) {
            instance = new ControlMap();
        }
        return instance;
    }

    private ControlMap() {
    }

    public void configurePreset1(CommandNiDsXboxController operator, CommandNiDsPS5Controller driver) {
        // -------- DRIVER CONTROLS ---------

        // Reset robot pose rotation to 0° when circle button is pressed
        driver
                .getHID().povUp().multiPress(2, 1)
                .onTrue(
                        Commands.runOnce(
                                () -> Drive.getInstance()
                                        .setPose(
                                                new Pose2d(
                                                        Drive.getInstance()
                                                                .getPose()
                                                                .getTranslation(),
                                                        MatchState.getAlliance()
                                                                .orElse(Alliance.BLUE) == Alliance.BLUE
                                                                        ? new Rotation2d()
                                                                        : new Rotation2d(
                                                                                Math.PI))),
                                Drive.getInstance())
                                .ignoringDisable(true));

        // Intake Controls
        
        // Full intake up
        driver
                .L1()
                .onTrue(
                        Commands.either(
                                Commands.runOnce(
                                        () -> {
                                            IntakeWrist.getInstance()
                                                    .runVoltage(WristConstants.shootingVoltage);
                                            IntakeWrist.getInstance().isDown = false;
                                            Intake.getInstance().runVoltage(
                                                    IntakeConstants.fastIntakeVoltage);
                                        },
                                        IntakeWrist.getInstance()),
                                Commands.runOnce(
                                        () -> {
                                            IntakeWrist.getInstance()
                                                    .goToPosition(WristConstants.stowedPositionRotations);
                                            IntakeWrist.getInstance().isDown = false;
                                        },
                                        IntakeWrist.getInstance()),

                                () -> RobotContainer.shooterCommand.isRunningFunnel()))
                .onFalse(
                        Commands.runOnce(
                                () -> {
                                    IntakeWrist.getInstance()
                                            .goToPosition(WristConstants.deployedPositionRotations);
                                    IntakeWrist.getInstance().isDown = true;
                                    Intake.getInstance().runVoltage(0);
                                },
                                IntakeWrist.getInstance()));

        // OLD INTAKE LIFT, REMAPPED
        driver
                .getHID().povLeft()
                        .onTrue(
                        Commands.either(
                                Commands.runOnce(
                                        () -> {
                                            IntakeWrist.getInstance()
                                                    .goToPosition(WristConstants.halfUpPositionRotations);
                                            IntakeWrist.getInstance().isDown = false;
                                            Intake.getInstance().runVoltage(IntakeConstants.fastIntakeVoltage);
                                        },
                                        IntakeWrist.getInstance()
                                ),
                                Commands.runOnce(
                                        () -> {
                                            IntakeWrist.getInstance()
                                                    .goToPosition(WristConstants.halfUpPositionRotations);
                                            IntakeWrist.getInstance().isDown = false;
                                            Intake.getInstance().runVoltage(0);
                                        },
                                        IntakeWrist.getInstance()
                                ),
                        () -> RobotContainer.shooterCommand.isRunningFunnel()
                        )
                )

                .onFalse(
                        Commands.runOnce(
                                () -> {
                                        IntakeWrist.getInstance()
                                                .goToPosition(WristConstants.deployedPositionRotations);
                                        IntakeWrist.getInstance()
                                                .isDown = true;
                                }, IntakeWrist.getInstance()));

        // Wrist manual voltage code
        // driver
        // .leftBumper()
        // .whileTrue(
        // Commands.run(
        // () -> {
        // IntakeWrist.getInstance().runVoltage(
        // WristConstants.intakeManualDeployVoltage);
        // },
        // IntakeWrist.getInstance()));
        // .whileFalse(
        // // Commands.either(
        // Commands.run(
        // () -> {
        // IntakeWrist.getInstance()
        // .runVoltage(WristConstants.shootingVoltage);
        // },
        // IntakeWrist.getInstance()));
        // Commands.run(
        // () -> {
        // IntakeWrist.getInstance()
        // .runVoltage(WristConstants.intakeManualLiftVoltage);
        // },
        // IntakeWrist.getInstance()),
        // () -> RobotContainer.shooterCommand.isRunningFunnel()));

        // Run intake (hold)
        driver.L2().onTrue(Commands.runOnce(() -> {
            Intake.getInstance().runVoltage(12);
            if (IntakeWrist.getInstance().isDown) {
                IntakeWrist.getInstance().runVoltage(WristConstants.intakeDeployVoltage);
            }
        }, Intake.getInstance(), IntakeWrist.getInstance())).onFalse(Commands.runOnce(() -> {
            Intake.getInstance().runVoltage(0);
            if (IntakeWrist.getInstance().isDown) {
                IntakeWrist.getInstance().runVoltage(0);
            }
        }, Intake.getInstance(), IntakeWrist.getInstance()));

        // Run outtake (hold)
        operator.leftTrigger().onTrue(Commands.runOnce(() -> {
            Intake.getInstance().runVoltage(IntakeConstants.reverseIntakeVoltage);
            Funnel.getInstance().runVoltageFunnel(-12);
            Funnel.getInstance().runVoltageIndexer(-12);
        }, Intake.getInstance(), Funnel.getInstance())).onFalse(Commands.runOnce(() -> {
            Intake.getInstance().runVoltage(0);
            Funnel.getInstance().runVoltageFunnel(FunnelConstants.funnelSlowVolts);
            Funnel.getInstance().runVoltageIndexer(FunnelConstants.indexerSlowVolts);
            if (IntakeWrist.getInstance().isDown) {
                IntakeWrist.getInstance().runVoltage(0);
            }
        }, Intake.getInstance(), Funnel.getInstance()));

        // Reverse intake (hold)
        // driver
        // .leftBumper()
        // .onTrue(
        // Commands.runOnce(
        // () -> {
        //
        // Intake.getInstance().runVoltage(IntakeConstants.reverseIntakeVoltage);
        // },
        // Intake.getInstance()))
        // .onFalse(
        // Commands.runOnce(
        // () -> {
        // if (IntakeWrist.getInstance()
        // .getPositionSetpointRot() ==
        // WristConstants.stowedPositionRotations) {
        // Intake.getInstance().runVoltage(0);
        // } else {
        //
        // Intake.getInstance().runVoltage(IntakeConstants.fastIntakeVoltage);
        // }
        // },
        // IntakeWrist.getInstance(),
        // Intake.getInstance()));

        // Funnel Controls
        driver
                .R1()
                // Automatic intake up
                // .onTrue(
                // Commands.waitSeconds(0.5)
                // .andThen(
                // Commands.runOnce(
                // () -> {
                // IntakeWrist.getInstance()
                // .runVoltage(WristConstants.shootingVoltage);
                // Intake.getInstance().runVoltage(IntakeConstants.fastIntakeVoltage);
                // },
                // IntakeWrist.getInstance(),
                // Intake.getInstance())
                // .andThen(Commands.waitUntil(() -> IntakeWrist.getInstance().isStowed()))
                // .andThen(
                // Commands.runOnce(
                // () -> {
                // IntakeWrist.getInstance()
                // .goToPosition(
                // WristConstants.stowedPositionRotations);
                // },
                // IntakeWrist.getInstance()))))
                .whileTrue(
                        Commands.run(
                                () -> {
                                    RobotContainer.shooterCommand.runFunnel();
                                })
                                .until(() -> RobotContainer.shooterCommand.runningFunnel))
                .onFalse(
                        Commands.runOnce(
                                () -> {
                                    RobotContainer.shooterCommand.stopFunnel();
                                    // Intake.getInstance().runVoltage(0);
                                    // IntakeWrist.getInstance().goToPosition(WristConstants.deployedPositionRotations);
                                }));
        // IntakeWrist.getInstance()));

        // Shooting Controls

        // Autorotate
        driver
                .R2()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    RobotContainer.shooterCommand
                                            .setAutoRotating(true);
                                }))
                .onFalse(
                        Commands.runOnce(
                                () -> {
                                    RobotContainer.shooterCommand
                                            .setAutoRotating(false);
                                }));

        // Current Shifting
        // driver.faceUp().onTrue(Commands.runOnce(() -> {
        // Drive.getInstance().changeCurrentLimitsAndNeutralMode(DriveConstants.kSlipCurrent.in(Amps),
        // 60, 40, 5.0,
        // NeutralModeValue.Brake);
        // }, Drive.getInstance())).onFalse(Commands.runOnce(() -> {
        // Drive.getInstance().changeCurrentLimitsAndNeutralMode(DriveConstants.kSlipCurrent.in(Amps),
        // 40, 40, 0.0,
        // NeutralModeValue.Brake);
        // }, Drive.getInstance()));

        // driver.faceDown().onTrue(Commands.runOnce(() -> {
        // Drive.getInstance().changeCurrentLimitsAndNeutralMode(DriveConstants.kSlipCurrent.in(Amps),
        // 80, 40, 2.0,
        // NeutralModeValue.Brake);
        // }, Drive.getInstance())).onFalse(Commands.runOnce(() -> {
        // Drive.getInstance().changeCurrentLimitsAndNeutralMode(DriveConstants.kSlipCurrent.in(Amps),
        // 40, 40, 0.0,
        // NeutralModeValue.Brake);
        // }, Drive.getInstance()));

        // --------------- OPERATOR CONTROLS ---------------

        // Set wrist to deployed position if we started the robot when it was down
        operator
                .getHID().povRight()
                .multiPress(2, 1)
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    IntakeWrist.getInstance().resetPosition(
                                            WristConstants.deployedPositionRotations);
                                },
                                IntakeWrist.getInstance()));

        // X-Lock
        // When any non-zero control inputs are given, this will be overridden
        operator
                .rightTrigger()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    Drive.getInstance().stopWithX();
                                },
                                Drive.getInstance()));

        // Shooter Controls

        // Autoaim
        operator
                .rightBumper()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    RobotContainer.shooterCommand
                                            .setAutoShooting(true);
                                }))
                .onFalse(
                        Commands.runOnce(
                                () -> {
                                    RobotContainer.shooterCommand.reset();
                                }));

        // Aim near
        operator
                .y()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    RobotContainer.shooterCommand.shootNear();
                                }))
                .onFalse(
                        Commands.runOnce(
                                () -> {
                                    RobotContainer.shooterCommand.reset();
                                }));

        // Aim middle
        operator
                .x()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    RobotContainer.shooterCommand.shootMiddle();
                                }))
                .onFalse(
                        Commands.runOnce(
                                () -> {
                                    RobotContainer.shooterCommand.reset();
                                }));

        // Aim tower
        operator
                .a()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    RobotContainer.shooterCommand.shootTower();
                                }))
                .onFalse(
                        Commands.runOnce(
                                () -> {
                                    RobotContainer.shooterCommand.reset();
                                }));

        // Aim trench
        operator
                .b()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    RobotContainer.shooterCommand.shootTrench();
                                }))
                .onFalse(
                        Commands.runOnce(
                                () -> {
                                    RobotContainer.shooterCommand.reset();
                                }));
        // Near pass preset
        operator
                .getHID().povLeft()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    RobotContainer.shooterCommand.nearPass();
                                }))
                .onFalse(
                        Commands.runOnce(
                                () -> {
                                    RobotContainer.shooterCommand.reset();
                                }));

        // Far pass preset
        operator
                .getHID().povDown()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    RobotContainer.shooterCommand.farPass();
                                }))
                .onFalse(
                        Commands.runOnce(
                                () -> {
                                    RobotContainer.shooterCommand.reset();
                                }));

        // Wiggle slightly up
        operator.leftBumper().onTrue(Commands.runOnce(() -> {
            IntakeWrist.getInstance().goToPosition(WristConstants.slightlyUpPositionRotations);
        }, IntakeWrist.getInstance())).onFalse(Commands.runOnce(() -> {
            IntakeWrist.getInstance().goToPosition(WristConstants.deployedPositionRotations);
            IntakeWrist.getInstance().isDown = true;
        }, IntakeWrist.getInstance()));

        // Wiggle half up
        // operator.leftTrigger().onTrue(Commands.runOnce(() -> {
        // IntakeWrist.getInstance().goToPosition(WristConstants.halfUpPositionRotations);
        // }, IntakeWrist.getInstance())).onFalse(Commands.runOnce(() -> {
        // IntakeWrist.getInstance().goToPosition(WristConstants.deployedPositionRotations);
        // IntakeWrist.getInstance().isDown = true;
        // }, IntakeWrist.getInstance()));

        // Wrist deploy with a lot of voltage to get hopper unstuck
        operator.getHID().povUp().onTrue(Commands.runOnce(() -> {
            IntakeWrist.getInstance().runVoltage(WristConstants.firstDeployVoltage);
            IntakeWrist.getInstance().isDown = true;
        }, IntakeWrist.getInstance())).onFalse(Commands.runOnce(() -> {
            IntakeWrist.getInstance().goToPosition(WristConstants.deployedPositionRotations);
        }, IntakeWrist.getInstance()));

        // ORCHESTRA STUFF II
        // operator
        // .dpadLeft()
        // .and(operator.faceUp())
        // .onTrue(
        // Commands.runOnce(
        // () -> {
        // if (OrchestraUtils.isPlaying) {
        // if (OrchestraUtils.songPlaying == OrchestraUtils.songSelected) {
        // Telemetry.log(
        // "DB/String 0",
        // "Paused: " + OrchestraUtils.allSongs[OrchestraUtils.songSelected]);
        // OrchestraUtils.orchestra.pause();
        // OrchestraUtils.isPlaying = false;
        // } else {
        // OrchestraUtils.orchestra.stop();
        // Telemetry.log(
        // "DB/String 0",
        // "Playing: " + OrchestraUtils.allSongs[OrchestraUtils.songSelected]);
        // OrchestraUtils.orchestra.loadMusic(
        // "music/" + OrchestraUtils.allSongs[OrchestraUtils.songSelected]);
        // OrchestraUtils.songPlaying = OrchestraUtils.songSelected;
        // OrchestraUtils.orchestra.play();
        // }
        // } else {
        // if (OrchestraUtils.songPlaying != OrchestraUtils.songSelected) {
        // OrchestraUtils.orchestra.loadMusic(
        // "music/" + OrchestraUtils.allSongs[OrchestraUtils.songSelected]);
        // OrchestraUtils.songPlaying = OrchestraUtils.songSelected;
        // }
        // Telemetry.log(
        // "DB/String 0",
        // "Playing: " + OrchestraUtils.allSongs[OrchestraUtils.songSelected]);
        // OrchestraUtils.isPlaying = true;
        // OrchestraUtils.orchestra.play();
        // }
        // }));
        // operator
        // .dpadLeft()
        // .and(operator.faceDown())
        // .onTrue(
        // Commands.runOnce(
        // () -> {
        // if (OrchestraUtils.isPlaying) {
        // OrchestraUtils.orchestra.stop();
        // OrchestraUtils.isPlaying = false;
        // }
        // Telemetry.log("DB/String 0", "Playing: ");
        // OrchestraUtils.songPlaying = -1;
        // }));
        // operator
        // .dpadLeft()
        // .and(operator.faceLeft())
        // .onTrue(
        // Commands.runOnce(
        // () -> {
        // OrchestraUtils.songSelected = (OrchestraUtils.songSelected - 1
        // + OrchestraUtils.allSongs.length)
        // % OrchestraUtils.allSongs.length;
        // Telemetry.log(
        // "DB/String 1",
        // "Selected: " + OrchestraUtils.allSongs[OrchestraUtils.songSelected]);
        // }));
        // operator
        // .dpadLeft()
        // .and(operator.faceRight())
        // .onTrue(
        // Commands.runOnce(
        // () -> {
        // OrchestraUtils.songSelected = (OrchestraUtils.songSelected + 1)
        // % OrchestraUtils.allSongs.length;
        // Telemetry.log(
        // "DB/String 1",
        // "Selected: " + OrchestraUtils.allSongs[OrchestraUtils.songSelected]);
        // }));
    }
}
