package first.robot.commands;

import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.DriveConstants.FieldConstants;
import first.robot.subsystems.funnel.Funnel;
import first.robot.subsystems.funnel.FunnelConstants;
import first.robot.subsystems.hood.Hood;
import first.robot.subsystems.hood.HoodConstants;
import first.robot.subsystems.shooter.Shooter;
import first.robot.subsystems.shooter.ShooterConstants;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose2d;

public class ShooterCommand extends Command {
    @AutoLogOutput
    private double shooterVelocityRPS;
    @AutoLogOutput
    private double hoodAngleDeg;
    @AutoLogOutput
    private double funnelVoltage;
    @AutoLogOutput
    private double indexerVoltage;
    @AutoLogOutput
    private double fastIndexerVoltage;
    @AutoLogOutput
    public boolean autoShooting;
    @AutoLogOutput
    public boolean autoRotating;
    @AutoLogOutput
    public boolean runningFunnel = false;
    @AutoLogOutput
    private boolean passing = false;

    // public LoggedTunableNumber tuneHoodAngleDeg =
    // new LoggedTunableNumber("ShooterCommand/tuneHoodAngleDeg",
    // HoodConstants.hoodAngleShootTrench);
    // public LoggedTunableNumber tuneShooterVelocityRPS =
    // new LoggedTunableNumber("ShooterCommand/tuneShooterVelocityRPS",
    // ShooterConstants.trenchRPS);
    // public LoggedTunableNumber tuneIndexerVoltage =
    // new LoggedTunableNumber(
    // "ShooterCommand/tuneIndexerVoltage", FunnelConstants.indexerTrenchVolts);

    public ShooterCommand() {
        addRequirements(Shooter.getInstance(), Hood.getInstance(), Funnel.getInstance());
        shooterVelocityRPS = ShooterConstants.defaultRPS;
        hoodAngleDeg = HoodConstants.maxAngleDegrees;
        funnelVoltage = FunnelConstants.funnelSlowVolts;
        indexerVoltage = FunnelConstants.indexerSlowVolts;
        fastIndexerVoltage = FunnelConstants.indexerMiddleVolts;
        autoShooting = false;
        autoRotating = false;
        passing = false;

        Logger.recordOutput("ShooterCommand/CommandRunning", false);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Logger.recordOutput("ShooterCommand/CommandRunning", true);
        if (autoShooting) {
            autoAngleAndVelocity();
        }

        if (runningFunnel && shooterVelocityRPS != ShooterConstants.defaultRPS) {
            if (passing) {
                funnelVoltage = FunnelConstants.funnelPassingVolts;
                indexerVoltage = FunnelConstants.indexerPassingVolts;
            } else {
                funnelVoltage = FunnelConstants.funnelFastVolts;
                indexerVoltage = fastIndexerVoltage;
            }
        } else {
            funnelVoltage = FunnelConstants.funnelSlowVolts;
            indexerVoltage = FunnelConstants.indexerSlowVolts;
        }

        // Checking to make sure we're still in range to continue running auto shooting
        // (Assuming we're still auto shooting of course)
        // if (!Drive.getInstance().isInShootingRange()) {
        // autoShooting = false;
        // }

        Hood.getInstance().goToAngle(hoodAngleDeg);
        Shooter.getInstance().runVelocity(shooterVelocityRPS);
        Funnel.getInstance().runVoltageFunnel(funnelVoltage);
        Funnel.getInstance().runVoltageIndexer(indexerVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("ShooterCommand/CommandRunning", false);
    }

    public void autoAngleAndVelocity() {
        Pose2d pos = Drive.getInstance()
                .getPose()
                .plus(
                        Drive.getInstance()
                                .getChassisVelocities()
                                .toTwist2d(ShooterConstants.sotmPhaseShiftTimeSeconds)
                                .exp());
        Pose2d hubPos = MatchState.getAlliance().isPresent() && MatchState.getAlliance().get() == Alliance.RED
                ? FieldConstants.RED_HUB_POS
                : FieldConstants.BLUE_HUB_POS;

        if (Drive.getInstance().isInShootingRange()) {
            passing = false;
            double distFromHub = pos.getTranslation().getDistance(hubPos.getTranslation());

            double v = ShooterConstants.shooterVelocityMap.get(distFromHub);
            Logger.recordOutput("ShooterCommand/UnclampedVelocity", v);
            double a = ShooterConstants.hoodAngleMap.get(distFromHub);
            Logger.recordOutput("ShooterCommand/UnclampedAngle", a);
            double i = ShooterConstants.indexerVoltageMap.get(distFromHub);
            Logger.recordOutput("ShooterCommand/UnclampedIndexerVoltage", i);

            shooterVelocityRPS = Math.clamp(v, ShooterConstants.minVelocityRPS, ShooterConstants.maxVelocityRPS);

            hoodAngleDeg = Math.clamp(a, HoodConstants.minAngleDegrees, HoodConstants.maxAngleDegrees);

            fastIndexerVoltage = Math.clamp(i, 0.0, 12.0);

            // // Optimize until close enough

            // double t = 0;

            // int iterations = 0;
            // do {
            //     t = ShooterConstants.timeOfFlightMap.get(distFromHub);

            //     ChassisVelocities velocities = Drive.getInstance().getChassisVelocities();

            //     // Assume 0 angular velocity affecting the balls, because if there is angular
            //     // velocity it would mess up the shot, since the balls at the edges of the
            //     // shooter would collide with the balls at the center
            //     Pose2d newPos = pos.plus(new Transform2d(velocities.vx * t,
            //             velocities.vy * t, new Rotation2d(velocities.omega *
            //                     t)));
            //     distFromHub = newPos.getTranslation().getDistance(hubPos.getTranslation());
            //     iterations++;
            //     if (iterations >= 10)
            //         break;
            // } while (Math
            //         .abs(t - ShooterConstants.timeOfFlightMap
            //                 .get(distFromHub)) > ShooterConstants.sotmOptimizationTimeThresholdSeconds);

            // Logger.recordOutput("ShooterCommand/TimeOfFlight", ShooterConstants.timeOfFlightMap.get(distFromHub));

        } else {
            passing = true;
            double distFromCorner = pos.getTranslation().getDistance(
                Drive.getInstance().getCornerPos());

            double v = ShooterConstants.shooterPassingVelocityMap.get(distFromCorner);
            Logger.recordOutput("ShooterCommand/UnclampedVelocity", v);
            double a = HoodConstants.minAngleDegrees;
            Logger.recordOutput("ShooterCommand/UnclampedAngle", a);
            double i = FunnelConstants.indexerPassingVolts;
            Logger.recordOutput("ShooterCommand/UnclampedIndexerVoltage", i);

            shooterVelocityRPS = Math.clamp(v, ShooterConstants.minVelocityRPS, ShooterConstants.maxVelocityRPS);

            hoodAngleDeg = Math.clamp(a, HoodConstants.minAngleDegrees, HoodConstants.maxAngleDegrees);

            fastIndexerVoltage = Math.clamp(i, 0.0, 12.0);
        }
    }

    public boolean isAutoShooting() {
        return autoShooting;
    }

    public void setAutoShooting(boolean autoShoot) {
        autoShooting = autoShoot;
    }

    public boolean isAutoRotating() {
        return autoRotating;
    }

    public void setAutoRotating(boolean autoRotate) {
        if (autoRotate && !autoRotating) {
            Drive.getInstance().rotationController.reset(Drive.getInstance().getRotation().getRadians());
        }
        autoRotating = autoRotate;
    }

    // Funnel Commands
    public boolean isRunningFunnel() {
        return runningFunnel;
    }

    public void runFunnel() {
        if (passing
                || isReady()) {
            runningFunnel = true;
        } else {
            runningFunnel = false;
        }
    }

    public void stopFunnel() {
        runningFunnel = false;
    }

    @AutoLogOutput
    public boolean isReady() {
        return Shooter.getInstance().isWithinVelocityThreshold()
                && Hood.getInstance().isWithinAngleThreshold()
                && Shooter.getInstance().getShooterVelocitySetpointRPS() != ShooterConstants.defaultRPS
                && Shooter.getInstance().getShooterVelocitySetpointRPS() != 0;
    }

    public void shootNear() {
        passing = false;
        setHoodAngle(71.0);
        setShooterVelocity(40.5);
        setIndexerVoltage(5.0);
    }

    public void shootMiddle() {
        passing = false;
        setHoodAngle(HoodConstants.hoodAngleShootMiddle);
        setShooterVelocity(ShooterConstants.middleRPS);
        setIndexerVoltage(FunnelConstants.indexerMiddleVolts);
    }

    public void shootTower() {
        passing = false;
        setHoodAngle(HoodConstants.hoodAngleShootTower);
        setShooterVelocity(ShooterConstants.towerRPS);
        setIndexerVoltage(FunnelConstants.indexerTowerVolts);
        // setHoodAngle(tuneHoodAngleDeg.get());
        // setShooterVelocity(tuneShooterVelocityRPS.get());
        // setIndexerVoltage(tuneIndexerVoltage.get());
    }

    public void shootTrench() {
        passing = false;
        setHoodAngle(HoodConstants.hoodAngleShootTrench);
        setShooterVelocity(ShooterConstants.trenchRPS);
        setIndexerVoltage(FunnelConstants.indexerTrenchVolts);
    }

    public void nearPass() {
        passing = true;
        setHoodAngle(HoodConstants.minAngleDegrees);
        setShooterVelocity(ShooterConstants.passingNeutralRPS);
        setIndexerVoltage(FunnelConstants.indexerPassingVolts);
        // if (Drive.getInstance().isInShootingRange()) {
        // // Can't pass
        // return;
        // } else if (Drive.getInstance().isInOpponentShootingRange()) {
        // hoodAngleDeg = HoodConstants.hoodAnglePassOpponentZone;
        // shooterVelocityMPS = ShooterConstants.passingOpponentMPS;
        // } else {
        // hoodAngleDeg = HoodConstants.hoodAnglePassNeutralZone;
        // shooterVelocityMPS = ShooterConstants.passingNeutralMPS;
        // }
    }

    public void farPass() {
        passing = true;
        setHoodAngle(HoodConstants.minAngleDegrees);
        setShooterVelocity(ShooterConstants.passingOpponentRPS);
        setIndexerVoltage(FunnelConstants.indexerPassingVolts);
    }

    public void reset() {
        setHoodAngle(HoodConstants.maxAngleDegrees);
        setShooterVelocity(
                Drive.getInstance().isInOpponentShootingRange() ? 0 : ShooterConstants.defaultRPS);
        stopFunnel();
        passing = false;
        autoShooting = false;
    }

    public void stop() {
        setHoodAngle(HoodConstants.maxAngleDegrees);
        setShooterVelocity(0);
        stopFunnel();
        passing = false;
        autoShooting = false;
    }

    public void setHoodAngle(double angleDeg) {
        hoodAngleDeg = angleDeg;
    }

    public void setShooterVelocity(double velocityMPS) {
        shooterVelocityRPS = velocityMPS;
    }

    public void setIndexerVoltage(double volts) {
        fastIndexerVoltage = volts;
    }
}
