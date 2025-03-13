package ravenrobotics.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;
import ravenrobotics.robot.subsystems.drive.DriveSubsystem;
import ravenrobotics.robot.subsystems.vision.VisionSubsystem;

public class AlignToReefCommand extends Command {

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

    public AlignToReefCommand() {
        this.driveSubsystem = DriveSubsystem.getInstance();
        this.visionSubsystem = VisionSubsystem.getInstance();

        addRequirements(this.driveSubsystem, this.visionSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Ye");
    }

    @Override
    public void execute() {
        var results = visionSubsystem.getLeftAprilTags();

        List<PhotonTrackedTarget> aprilTags;

        if (results.isPresent()) {
            aprilTags = results.get();
        } else {
            return;
        }

        if (aprilTags.isEmpty()) {
            return;
        }

        PhotonTrackedTarget biggestTag = aprilTags.get(aprilTags.size() - 1);

        Transform3d tagPosition = biggestTag.getBestCameraToTarget();
        Rotation3d tagRotation = tagPosition.getRotation();

        System.out.println(
            tagPosition.getX() +
            "\n" +
            tagPosition.getY() +
            "\n" +
            tagRotation.getZ()
        );

        double sideCommand = sideController.calculate(
            tagPosition.getMeasureY().in(Units.Meter),
            0
        );

        double rangeCommand = rangeController.calculate(
            tagPosition.getMeasureX().in(Units.Meter),
            0.5
        );
        double yawCommand =
            yawController.calculate(tagRotation.getZ(), 0) * 0.5;

        // if (sideCommand < 0.1 && rangeCommand < 0.1 && yawCommand < 0.1) {
        //     isDone = true;
        // }

        System.out.println("Running drive command");

        driveSubsystem.run(new ChassisSpeeds(rangeCommand, sideCommand, 0));
    }

    @Override
    public boolean isFinished() {
        return isDone;
    }
}
