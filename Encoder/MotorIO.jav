package frc.robot.subsystems.wrists;

import org.littletonrobotics.junction.AutoLog;

public interface MotorIO {
    @AutoLog
    public static class MotorIOInputs {
        public boolean motorConnected = false;
        public boolean encoderConnected = false;
        public double motorPositionRot = 0.0;
        public double motorVelocityRPS = 0.0;
        public double motorAppliedVolts = 0.0;
    }
 
    public default void updateInputs(MotorIOInputs inputs) {
    }

    public default void runVoltage(double volts) {
    }

    public default void goToPosition(double rotations) {
    }
}