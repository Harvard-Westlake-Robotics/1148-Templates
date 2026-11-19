package LimitSwitch;

import org.littletonrobotics.junction.AutoLog;

public interface LimitSwitchIO {
  @AutoLog
  public static class LimitSwitchIOInputs {
    public boolean isPressed = false;
  }

  public default void updateInputs(LimitSwitchIOInputs inputs) {}

  public default boolean isPressed() {
    return false;
  }
}