package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface MotorWithEncoderIO {
    @AutoLog
    public static class MotorWithEncoderIOInputs {
        public boolean motorConnected = false;
        public boolean encoderConnected = false;
        public double motorPositionRot = 0.0;
        public double motorVelocityRPS = 0.0;
        public double motorAppliedVolts = 0.0;
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;

    }
 
    public default void updateInputs(MotorWithEncoderIOInputs inputs) {
    }

    public default void runVoltage(double volts) {
    }

    public default void goToPosition(double rotations) {
    }
}