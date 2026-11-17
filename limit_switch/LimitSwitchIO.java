package limit_switch;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.wpilibj.DigitalInput;

public interface LimitSwitchIO {
  @AutoLog
  public static class LimitSwitchIOInputs {
    public bool isPressed = false;
  }

  public default void updateInputs(LimitSwitchIOInputs inputs) {}

  public default boolean isPressed() {
    return false;
  }
}