// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package first.robot;

import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.DriveConstants;
import first.robot.subsystems.drive.NetworkCommunicator;
import first.robot.subsystems.vision.Vision;

import static org.wpilib.units.Units.Amps;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import first.robot.util.wpilog.WPILOGWriter;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.driverstation.MatchState;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.tunable.Tunables;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.limelightvision.IMUMode;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    public Robot() {
        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata(
                "GitDirty",
                switch (BuildConstants.DIRTY) {
                    case 0 -> "All changes committed";
                    case 1 -> "Uncommitted changes";
                    default -> "Unknown";
                });

        // Set up data receivers & replay source
        switch (RobotContainer.currentMode) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs" or
                // "/home/systemcore/logs" if USB unmounted)
                // File usb = new File("/u");
                // if (usb.isDirectory() && usb.canWrite()) {
                    Logger.addDataReceiver(new WPILOGWriter("/u/logs"));
                // }
                Logger.addDataReceiver(new WPILOGWriter("/home/systemcore/logs"));
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case SIM:
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(
                        new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // home/lvuser/logs
                break;
        }

        // TODO: PUT IN REAL VALUES
        // LoggedPowerDistribution.getInstance(0, 2, ModuleType.REV);

        // Start AdvantageKit logger
        Logger.start();

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();
        // try {
        // Elastic.selectTab("Autonomous");
        // } catch (Throwable t) {
        // t.printStackTrace();
        // Throwable cause = t.getCause();
        // while (cause != null) {
        // System.err.println("Caused by:");
        // cause.printStackTrace();
        // cause = cause.getCause();
        // }
        // }

        // SmartDashboard.putData(new PowerDistribution(1));
        Tunables.publish("CommandScheduler", CommandScheduler.getInstance());

        NetworkCommunicator.getInstance().setIsAuto(true);
    }

    /** This function is called periodically during all modes. */
    @Override
    public void robotPeriodic() {
        // Optionally switch the thread to high priority to improve loop
        // timing (see the template project documentation for details)
        // Threads.setCurrentThreadPriority(true, 99);

        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands, removing
        // finished or interrupted commands, and running subsystem periodic() methods.
        // This must be called from the robot's periodic block in order for anything in
        // the Command-based framework to work.
        CommandScheduler.getInstance().run();

        // Return to non-RT thread priority (do not modify the first argument)
        // Threads.setCurrentThreadPriority(false, 10);
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        Vision.getInstance().setIMUMode(IMUMode.EXTERNAL_SEED_INTERNAL);
        Drive.getInstance().setVisionActive(true);
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        // Elastic.selectTab("Autonomous");
        Drive.getInstance().setVisionActive(true);
        autonomousCommand = robotContainer.getAutonomousCommand();
        Vision.getInstance().setIMUMode(IMUMode.EXTERNAL);

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
        NetworkCommunicator.getInstance().setIsAuto(true);
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        NetworkCommunicator.getInstance().setIsAuto(false);
        Vision.getInstance().recordMatch(true);
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        // Elastic.selectTab("Teleop");
        Drive.getInstance().setVisionActive(true);
        Vision.getInstance().setIMUMode(IMUMode.EXTERNAL);
        NetworkCommunicator.getInstance().setIsAuto(false);

        Drive.getInstance().changeCurrentLimitsAndNeutralMode(DriveConstants.kSlipCurrent.in(Amps), 40, 40, 2.0,
                NeutralModeValue.Brake);

        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        RobotContainer.shooterCommand.reset();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        double matchTime = MatchState.getMatchTime();
        double shiftTime = matchTime;
        String shift = "";
        if (matchTime >= 130) {
            shiftTime -= 130;
            shift = "Transition";
        } else if (matchTime >= 105) {
            shiftTime -= 105;
            shift = "Shift 1";
        } else if (matchTime >= 80) {
            shiftTime -= 80;
            shift = "Shift 2";
        } else if (matchTime >= 55) {
            shiftTime -= 55;
            shift = "Shift 3";
        } else if (matchTime >= 30) {
            shiftTime -= 30;
            shift = "Shift 4";
        } else if (matchTime >= 0) {
            shift = "Endgame";
        }

        Telemetry.log("ShiftTime", shiftTime);
        Telemetry.log("Shift", shift);
    }

    @Override
    public void teleopExit() {
        Vision.getInstance().recordMatch(false);
    }

    /** This function is called once when utility mode is enabled. */
    @Override
    public void utilityInit() {
        // Cancels all running commands at the start of utility mode.
        CommandScheduler.getInstance().cancelAll();
        NetworkCommunicator.getInstance().setIsAuto(false);
    }

    /** This function is called periodically during utility mode. */
    @Override
    public void utilityPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
    }
}
