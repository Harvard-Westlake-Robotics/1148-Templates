package first.robot.subsystems.shooter;

import first.robot.RobotContainer.Mode;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.SubsystemBase;

public class Shooter extends SubsystemBase {
  private ShooterIO io;
  private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final String key = "Shooter";
  private static Shooter instance;

  @AutoLogOutput private double shooterVelocitySetpointRPS;

  public double getShooterVelocitySetpointRPS() {
    return shooterVelocitySetpointRPS;
  }

  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter(Mode.REAL);
    }
    return instance;
  }

  public static Shooter getInstance(Mode mode) {
    if (instance == null) {
      instance = new Shooter(mode);
    }
    return instance;
  }

  private Shooter(Mode mode) {
    switch (mode) {
      case REAL:
        io = new ShooterIOTalonFX();
        break;
      case SIM:
        io = new ShooterIOSim();
        break;
      default:
        io = new ShooterIO() {};
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(key, inputs);
  }

  public void runVoltage(double volts) {
    io.runVoltage(volts);
  }

  public void runVelocity(double velocityRPS) {
    shooterVelocitySetpointRPS = velocityRPS;
    io.runVelocity(velocityRPS);
  }

  public boolean isWithinVelocityThreshold() {
    return Math.abs(inputs.shooterVelocityRPS - shooterVelocitySetpointRPS)
        <= ShooterConstants.velocityThresholdRPS;
  }
}
