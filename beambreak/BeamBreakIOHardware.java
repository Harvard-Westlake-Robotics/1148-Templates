package first.robot.subsystems.beambreak;

import org.wpilib.hardware.discrete.DigitalInput;

public class BeamBreakIOHardware implements BeamBreakIO {
    private final DigitalInput beamBreak;

    public BeamBreakIOHardware() {
        this.beamBreak = new DigitalInput(1); // input id here
    }

    @Override
    public void updateInputs(BeamBreakIOInputs inputs) {
        inputs.isBroken = getBeamBreak();
    }

    public boolean getBeamBreak() {
        return beamBreak.get();
    }
}