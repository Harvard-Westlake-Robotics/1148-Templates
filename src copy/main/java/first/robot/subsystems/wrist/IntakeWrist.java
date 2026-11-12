package first.robot.subsystems.wrist;

import static org.wpilib.units.Units.Volts;

import first.robot.RobotContainer;
import first.robot.RobotContainer.Mode;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.sysid.SysIdRoutine;
import org.wpilib.driverstation.GenericHID.RumbleType;
import org.wpilib.system.Timer;

public class IntakeWrist extends SubsystemBase {
  private WristIO io;
  private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
  private final String key = "IntakeWrist";
  private static IntakeWrist instance;
  public SysIdRoutine sysId;

  // -1.0 means it is alerting
  private Timer wristAlertTimer = new Timer();
  private double wristAlertMaxTimeSeconds = 1.5;
  @AutoLogOutput
  private boolean wristAlertIsTiming = false;

  @AutoLogOutput(key = key + "/isDown")
  public boolean isDown;

  @AutoLogOutput(key = key + "/isStowed")
  private boolean isStowed = true;

  @AutoLogOutput(key = key + "/isDeployed")
  private boolean isDeployed = false;

  @AutoLogOutput private double positionSetpointRot = WristConstants.stowedPositionRotations;

  public double getPositionSetpointRot() {
    return positionSetpointRot;
  }

  public static IntakeWrist getInstance() {
    if (instance == null) {
      instance = new IntakeWrist(Mode.REAL);
    }
    return instance;
  }

  public static IntakeWrist getInstance(Mode mode) {
    if (instance == null) {
      instance = new IntakeWrist(mode);
    }
    return instance;
  }

  private IntakeWrist(Mode mode) {
    switch (mode) {
      case REAL:
        io = new WristIOTalonFX();
        break;
      case SIM:
        io = new WristIOSim();
        break;
      default:
        io = new WristIO() {};
        break;
    }
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput(key + "/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVoltage(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(key, inputs);

    isStowed =
        Math.abs(inputs.wristPositionRot - WristConstants.stowedPositionRotations)
            <= WristConstants.stowToleranceRotations;

    isDeployed =
        Math.abs(inputs.wristPositionRot - WristConstants.deployedPositionRotations)
            <= WristConstants.deployToleranceRotations;

    if (isDown && !isDeployed) {
        if (!wristAlertIsTiming) {
            wristAlertTimer.restart();
        } else if (wristAlertTimer.hasElapsed(wristAlertMaxTimeSeconds)) {
            RobotContainer.driver.getHID().setRumble(RumbleType.RIGHT_RUMBLE, 0.5);
        }
        wristAlertIsTiming = true;
    } else if (wristAlertIsTiming) {
        wristAlertIsTiming = false;
        wristAlertTimer.stop();
        wristAlertTimer.reset();
        RobotContainer.driver.getHID().setRumble(RumbleType.RIGHT_RUMBLE, 0.0);
    }
  }

  public boolean isDeployed() {
    return isDeployed;
  }

  public boolean isStowed() {
    return isStowed;
  }

  public double getPositionRot() {
    return inputs.wristPositionRot;
  }

  public void runVoltage(double volts) {
    io.runVoltage(volts);
  }

  public void goToPosition(double rotations) {
    positionSetpointRot = rotations;
    io.goToPosition(rotations);
  }

  public void resetPosition(double rotations) {
    io.resetPosition(rotations);
  }

  // /** Returns a command to run a quasistatic test in the specified direction. */
  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //   return run(() -> runVoltage(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
  // }

  // /** Returns a command to run a dynamic test in the specified direction. */
  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //   return run(() -> runVoltage(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  // }
}
