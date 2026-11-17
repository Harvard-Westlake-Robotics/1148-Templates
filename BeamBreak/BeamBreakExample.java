//import DigitalInput somehow
//import org.wpilib.hardware.discrete.DigitalInput;

public class BeamBreakExample {
    private final DigitalInput beamBreak;

    public BeamBreakExample() {
        this.beamBreak = new DigitalInput(1); // input id here
    }
    
    public Boolean getBeamBreak() {
        if (beamBreak != null) {
            return beamBreak.get();
        } else {
            return null;
        }
    }

    public int getBeamBreakPort() {
        if (beamBreak != null) {
            return beamBreak.getChannel();
        } else {
            return -1;
        }
    }
}