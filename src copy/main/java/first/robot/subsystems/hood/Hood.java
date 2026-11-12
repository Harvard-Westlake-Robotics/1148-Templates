package first.robot.subsystems.hood;

import static org.wpilib.units.Units.Volts;

import first.robot.RobotContainer.Mode;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.sysid.SysIdRoutine;

public class Hood extends SubsystemBase {
  private HoodIO io;
  private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final String key = "Hood";
  private static Hood instance = null;
  public SysIdRoutine sysId;

  @AutoLogOutput private double hoodAngleSetpointDeg = HoodConstants.maxAngleDegrees - 0.1;

  public static Hood getInstance() {
    if (instance == null) {
      instance = new Hood(Mode.REAL);
    }
    return instance;
  }

  public static Hood getInstance(Mode mode) {
    if (instance == null) {
      instance = new Hood(mode);
    }
    return instance;
  }

  private Hood(Mode mode) {
    switch (mode) {
      case REAL:
        io = new HoodIOTalonFX();
        break;
      case SIM:
        io = new HoodIOSim();
        break;
      default:
        io = new HoodIO() {};
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
  }

  public void runVoltage(double volts) {
    io.runVoltage(volts);
  }

  public void goToAngle(double degrees) {
    hoodAngleSetpointDeg = degrees;
    io.goToAngle(degrees);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runVoltage(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runVoltage(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  public boolean isWithinAngleThreshold() {
    return Math.abs(inputs.hoodPositionDegrees - hoodAngleSetpointDeg)
        <= HoodConstants.angleThresholdDeg;
  }
}
