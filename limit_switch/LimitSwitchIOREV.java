package limit_switch;

import edu.wpi.first.wpilibj.DigitalInput;

public class LimitSwitchIOREV implements LimitSwitchIO {
  private final DigitalInput limitSwitch;

  private boolean isPressed;

  public LimitSwitchIOREV(int channel) {
    this.limitSwitch = new DigitalInput(channel);
    isPressed = limitSwitch.get();
  }

  public int getChannel() {
    return limitSwitch.getChannel();
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