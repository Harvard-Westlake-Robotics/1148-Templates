package LimitSwitch;

import org.wpilib.hardware.discrete.DigitalInput;

public class LimitSwitchIOREV implements LimitSwitchIO {
  private final DigitalInput limitSwitch;

  private boolean isPressed;

  public LimitSwitchIOREV(int channel) {
    this.limitSwitch = new DigitalInput(channel);
    isPressed = limitSwitch.get();
  }

  @Override
  public void updateInputs(LimitSwitchIOInputs inputs) {
    isPressed = limitSwitch.get();
    inputs.isPressed = isPressed;
  }

  @Override
  public boolean isPressed() {
    return isPressed;
  }
}