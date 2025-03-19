package ravenrobotics.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import ravenrobotics.robot.subsystems.drive.DriveSubsystem;

/**
 * Command for autonomous movement of the robot for a specified duration.
 * Moves the robot at a constant speed for a set amount of time.
 */
public class MoveAuto extends Command {

    /** Drive subsystem instance for controlling robot movement */
    private final DriveSubsystem driveSubsystem;

    /** Speed settings for the robot movement (x, y, rotation) */
    private final ChassisSpeeds speed;

    /** Duration of movement in seconds */
    private final double time;

    /** Flag to track completion of the command */
    private boolean isDone = false;

    /**
     * Constructs a new MoveAuto command.
     *
     * @param speed Forward speed of the robot in meters per second
     * @param time Duration to run the command in seconds
     */
    public MoveAuto(double speed, double time) {
        this.driveSubsystem = DriveSubsystem.getInstance();

        // Configure to move forward only (no sideways or rotational movement)
        this.speed = new ChassisSpeeds(speed, 0, 0);
        this.time = time;

        // Register this command as requiring the drive subsystem
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        // Apply the specified speed to the drive subsystem
        driveSubsystem.run(speed);

        // Wait for the specified time
        Timer.delay(time);

        // Mark the command as complete
        isDone = true;
    }

    @Override
    public void end(boolean isInteruupted) {
        // Stop the robot by applying zero speed
        driveSubsystem.run(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return isDone;
    }
}
