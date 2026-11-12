package first.robot.subsystems.funnel;

import first.robot.RobotContainer.Mode;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.SubsystemBase;

public class Funnel extends SubsystemBase {
  private FunnelIO io;
  private FunnelIOInputsAutoLogged inputs = new FunnelIOInputsAutoLogged();
  private final String key = "Funnel";
  private static Funnel instance;

  public static Funnel getInstance() {
    if (instance == null) {
      instance = new Funnel(Mode.REAL);
    }
    return instance;
  }

  public static Funnel getInstance(Mode mode) {
    if (instance == null) {
      instance = new Funnel(mode);
    }
    return instance;
  }

  private Funnel(Mode mode) {
    switch (mode) {
      case REAL:
        io = new FunnelIOTalonFX();
        break;
      case SIM:
        io = new FunnelIOSim();
        break;
      default:
        io = new FunnelIO() {};
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(key, inputs);
  }

  public void runVoltageFunnel(double volts) {
    Logger.recordOutput(key + "/FunnelVoltage", volts);
    io.runVoltageFunnel(volts);
  }

  public void runVoltageIndexer(double volts) {
    io.runVoltageIndexer(volts);
  }
}
