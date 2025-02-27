package ravenrobotics.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import ravenrobotics.robot.subsystems.drive.DriveSubsystem;

public class MoveAuto extends Command {

    private final DriveSubsystem driveSubsystem;

    private final ChassisSpeeds speed;
    private final double time;

    private boolean isDone = false;

    public MoveAuto(double speed, double time) {
        this.driveSubsystem = DriveSubsystem.getInstance();

        this.speed = new ChassisSpeeds(speed, 0, 0);
        this.time = time;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.run(speed);

        Timer.delay(time);

        isDone = true;
    }

    @Override
    public void end(boolean isInteruupted) {
        driveSubsystem.run(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return isDone;
    }
}
