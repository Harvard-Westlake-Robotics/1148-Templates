// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package first.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.system.Timer;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
public class ModuleIOSim implements ModuleIO {
  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private PIDController driveController =
      new PIDController(DriveConstants.DRIVE_SIM_KP, 0, DriveConstants.DRIVE_SIM_KD);
  private PIDController turnController =
      new PIDController(DriveConstants.TURN_SIM_KP, 0, DriveConstants.TURN_SIM_KD);
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    // Create drive and turn sim models
    driveSim =
        new DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                DriveConstants.DRIVE_GEARBOX,
                constants.DriveInertia,
                constants.DriveMotorGearRatio),
            DriveConstants.DRIVE_GEARBOX,
            DriveConstants.DRIVE_SIM_STDDEVS);
    turnSim =
        new DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                DriveConstants.TURN_GEARBOX, constants.SteerInertia, constants.SteerMotorGearRatio),
            DriveConstants.TURN_GEARBOX,
            DriveConstants.DRIVE_SIM_STDDEVS);

    // Enable wrapping for turn PID
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts = driveFFVolts + driveController.calculate(driveSim.getAngularVelocity());
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(turnSim.getAngularPosition());
    } else {
      turnController.reset();
    }

    // Update simulation state
    driveSim.setInputVoltage(Math.clamp(driveAppliedVolts, -12.0, 12.0));
    turnSim.setInputVoltage(Math.clamp(turnAppliedVolts, -12.0, 12.0));
    driveSim.update(0.02);
    turnSim.update(0.02);

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePositionRad = driveSim.getAngularPosition();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocity();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveStatorCurrentAmps = Math.abs(driveSim.getCurrentDraw());
    inputs.driveSupplyCurrentAmps = Math.abs(driveSim.getCurrentDraw());

    // Update turn inputs
    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;
    inputs.turnAbsolutePosition = new Rotation2d(turnSim.getAngularPosition());
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPosition());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocity();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnStatorCurrentAmps = Math.abs(turnSim.getCurrentDraw());

    // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't
    // matter)
    inputs.odometryTimestamps = new double[] {Timer.getMonotonicTimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFFVolts =
        DriveConstants.DRIVE_SIM_KS * Math.signum(velocityRadPerSec)
            + DriveConstants.DRIVE_SIM_KV * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }

  @Override
  public void changeCurrentLimitsAndNeutralMode(
      double newStatorLimit,
      double newSupplyLimit,
      double newSupplyLowerLimit,
      double newSupplyLowerTime,
      NeutralModeValue newNeutralMode) {
    // TODO: Add TalonFX Implementation to ModuleIOSim with TalonFXSimState
  }
}
