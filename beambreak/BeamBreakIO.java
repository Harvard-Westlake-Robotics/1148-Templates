package first.robot.subsystems.beambreak;

import org.littletonrobotics.junction.AutoLog;

public interface BeamBreakIO {
    @AutoLog
    public static class BeamBreakIOInputs {
        boolean isBroken = false;
    }
    
    public default void updateInputs(BeamBreakIOInputs inputs) {}

    public default Boolean getBeamBreak() {
        return null;
    }
}
