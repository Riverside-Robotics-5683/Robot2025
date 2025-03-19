// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ravenrobotics.robot;

import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The main robot class for running a robot (outside of Main).
 */
public class Robot extends LoggedRobot {

    @SuppressWarnings("unused")
    private PowerDistribution pdp; // I added the unused warning suppression to fix a resource leakage warning on

    // line 33 if it wasn't being created in a variable.

    private final RobotContainer robotContainer;

    /**
     * Creates an instance of the Robot class.
     */
    public Robot() {
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME); // Record the project name in the log file
        // metadata.
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE); // Record the build date of the project in the log
        // file metadata.
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA); // Record the sha1 hash of the latest Git commit in the log
        // file metadata.
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE); // Record the date of the latest Git commit in the log
        // file metadata.
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH); // Record the current Git branch in the log file
        // metadata.

        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed"); // If all changes have been committed, record the
                // metadata in the log file.
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncommitted changes"); // If there are uncommitted changes, record the
                // metadata in the log file.
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown"); // If GVersion can't find Git, record the metadata in the log
                // file.
                break;
        }

        // If the robot is real.
        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter()); // Make AdvantageKit write data to the log file.
            Logger.addDataReceiver(new NT4Publisher()); // Make AdvantageKit publish data to NetworkTables (driver dashboard).
            pdp = new PowerDistribution(1, ModuleType.kRev); // Create a PowerDistribution object to automatically enable
            // power monitoring (nice).
        } else { // If running inside a simulation (replaying a log file).
            setUseTiming(false); // Instead of running at normal robot speeds, run at the (much faster) speed of
            // the laptop to get the new data processed quickly.
            String log = LogFileUtil.findReplayLog(); // Get the path of the log file.
            Logger.setReplaySource(new WPILOGReader(log)); // Set the source for replay input data as the log file.
            Logger.addDataReceiver(
                new WPILOGWriter(LogFileUtil.addPathSuffix(log, "_replay"))
            ); // Write the replayed data
            // into a new log file with
            // the suffix "_replay".
        }

        Logger.start(); // Start logging data.

        robotContainer = new RobotContainer(); //Initialize the RobotContainer to setup everything.

        PathfindingCommand.warmupCommand().schedule();
    }

    @Override
    public void robotPeriodic() {
        // Run the CommandScheduler every 20ms so we can use Commands.
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        var command = robotContainer.getAutoCommand();

        if (command != null) {
            command.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        robotContainer.teleopSetup();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void testInit() {
        var command = robotContainer.getTestCommand();

        command.schedule();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}
