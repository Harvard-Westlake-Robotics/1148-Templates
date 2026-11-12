package first.robot.subsystems.intake;

import first.robot.RobotContainer.Mode;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.SubsystemBase;

public class Intake extends SubsystemBase {
  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final String key = "Intake";
  private static Intake instance;

  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake(Mode.REAL);
    }
    return instance;
  }

  public static Intake getInstance(Mode mode) {
    if (instance == null) {
      instance = new Intake(mode);
    }
    return instance;
  }

  public Intake(Mode mode) {
    switch (mode) {
      case REAL:
        io = new IntakeIOTalonFX();
        break;
      case SIM:
        io = new IntakeIOSim();
        break;
      default:
        io = new IntakeIO() {};
        break;
    }
  }

  public void runVoltage(double volts) {
    Logger.recordOutput(key + "/TargetVoltage", volts);
    io.runVoltage(volts);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(key, inputs);
  }
}
