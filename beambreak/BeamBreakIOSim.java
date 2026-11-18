package first.robot.subsystems.beambreak;

public class BeamBreakIOSim implements BeamBreakIO {
    public BeamBreakIOSim() {}

    public void updateInputs(BeamBreakIOInputs inputs) {
        inputs.isBroken = getBeamBreak();
    }

    // placeholder function, please change if simulating beam break
    public Boolean getBeamBreak() {
        return false;
    }
}
