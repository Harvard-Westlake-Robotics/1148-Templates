package first.robot.subsystems.beambreak;

import org.wpilib.hardware.discrete.DigitalInput;

public class BeamBreakIOHardware implements BeamBreakIO {
    private final DigitalInput beamBreak;

    public BeamBreakIOHardware() {
        this.beamBreak = new DigitalInput(1); // input id here
    }

    @Override
    public void updateInputs(BeamBreakIOInputs inputs) {
        inputs.value = getBeamBreak();
    }

    public Boolean getBeamBreak() {
        if (beamBreak != null) {
            return beamBreak.get();
        } else {
            return null;
        }
    }
}