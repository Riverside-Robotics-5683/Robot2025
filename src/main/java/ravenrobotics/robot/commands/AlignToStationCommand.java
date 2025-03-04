package ravenrobotics.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;
import java.util.Optional;
import org.photonvision.targeting.PhotonTrackedTarget;
import ravenrobotics.robot.subsystems.drive.DriveSubsystem;
import ravenrobotics.robot.subsystems.vision.VisionSubsystem;

public class AlignToStationCommand extends Command {

    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    private final PIDController sideController = new PIDController(
        5.0,
        0.0,
        0.0
    );
    private final PIDController rangeController = new PIDController(
        5.0,
        0.0,
        0.0
    );
    private final PIDController yawController = new PIDController(
        5.0,
        0.0,
        0.0
    );

    private boolean isDone = false;

    public AlignToStationCommand() {
        this.driveSubsystem = DriveSubsystem.getInstance();
        this.visionSubsystem = VisionSubsystem.getInstance();

        addRequirements(this.driveSubsystem, this.visionSubsystem);
    }

    @Override
    public void initialize() {
        visionSubsystem.setCoralToAprilTags();
    }

    @Override
    public void execute() {
        Optional<List<PhotonTrackedTarget>> results;

        try {
            results = visionSubsystem.getCoralAprilTags();
        } catch (IllegalAccessException e) {
            return;
        }

        List<PhotonTrackedTarget> aprilTags;

        if (results.isPresent()) {
            aprilTags = results.get();
        } else {
            return;
        }

        if (aprilTags.isEmpty()) {
            return;
        }

        PhotonTrackedTarget biggestTag = aprilTags.get(0);

        Transform3d tagPosition = biggestTag.getBestCameraToTarget();
        Rotation3d tagRotation = tagPosition.getRotation();

        double sideCommand = sideController.calculate(
            tagPosition.getMeasureY().in(Units.Meter),
            0
        );

        double rangeCommand = rangeController.calculate(
            tagPosition.getMeasureX().in(Units.Meter),
            0.2
        );
        double yawCommand = yawController.calculate(tagRotation.getZ(), 0);

        if (sideCommand < 0.1 && rangeCommand < 0.1 && yawCommand < 0.1) {
            isDone = true;
        }

        driveSubsystem.run(
            new ChassisSpeeds(rangeCommand, sideCommand, yawCommand)
        );
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.run(new ChassisSpeeds());
        visionSubsystem.setCoralToCoral();
    }

    @Override
    public boolean isFinished() {
        return isDone;
    }
}
