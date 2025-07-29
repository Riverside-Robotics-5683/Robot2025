package ravenrobotics.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.AutoBuilderException;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import ravenrobotics.robot.subsystems.drive.DriveSubsystem;
import ravenrobotics.robot.subsystems.elevator.ElevatorSubsystem;
import ravenrobotics.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import ravenrobotics.robot.subsystems.intake.IntakeSubsystem;

public class AutoFactory {

    public static final Map<
        PathfindingDestinations,
        Integer
    > BLUE_DESTINATIONS = new HashMap<>(8);
    public static final Map<PathfindingDestinations, Integer> RED_DESTINATIONS =
        new HashMap<>(8);

    private static final PathConstraints pathfindingConstraints =
        new PathConstraints(
            1.5,
            2.5,
            Units.degreesToRadians(540),
            Units.degreesToRadians(720)
        );

    private static final AprilTagFieldLayout fieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    // All the comments are relative to PathPlanner's view (top-down field view), but the assignments are relative to each side.
    static {
        // Top Left
        BLUE_DESTINATIONS.put(PathfindingDestinations.kReef1, 19);
        // Top Right
        BLUE_DESTINATIONS.put(PathfindingDestinations.kReef2, 20);
        // Middle Right
        BLUE_DESTINATIONS.put(PathfindingDestinations.kReef3, 21);
        // Bottom Right
        BLUE_DESTINATIONS.put(PathfindingDestinations.kReef4, 22);
        // Bottom Left
        BLUE_DESTINATIONS.put(PathfindingDestinations.kReef5, 17);
        // Middle Left
        BLUE_DESTINATIONS.put(PathfindingDestinations.kReef6, 18);

        // Top
        BLUE_DESTINATIONS.put(PathfindingDestinations.kCoralTop, 13);
        // Bottom
        BLUE_DESTINATIONS.put(PathfindingDestinations.kCoralBottom, 12);

        // Bottom Left
        RED_DESTINATIONS.put(PathfindingDestinations.kReef1, 11);
        // Middle Left
        RED_DESTINATIONS.put(PathfindingDestinations.kReef2, 10);
        // Top Left
        RED_DESTINATIONS.put(PathfindingDestinations.kReef3, 9);
        // Top Right
        RED_DESTINATIONS.put(PathfindingDestinations.kReef4, 8);
        // Middle Right
        RED_DESTINATIONS.put(PathfindingDestinations.kReef5, 7);
        // Bottom Right
        RED_DESTINATIONS.put(PathfindingDestinations.kReef6, 6);

        // Bottom
        RED_DESTINATIONS.put(PathfindingDestinations.kCoralTop, 1);
        // Coral
        RED_DESTINATIONS.put(PathfindingDestinations.kCoralBottom, 2);
    }

    public enum StartPosition {
        kLeft,
        kCenter,
        kRight,
    }

    public enum PathfindingDestinations {
        kReef1,
        kReef2,
        kReef3,
        kReef4,
        kReef5,
        kReef6,
        kCoralTop,
        kCoralBottom,
    }

    public enum CoralStation {
        kLeft,
        kRight,
    }

    public static Command getPathfindingCommand(
        PathfindingDestinations destination
    ) {
        Optional<Alliance> allianceOptional = DriverStation.getAlliance();

        Pose2d targetPose = new Pose2d();

        if (allianceOptional.isPresent()) {
            System.out.println(allianceOptional.get().toString());
            switch (allianceOptional.get()) {
                case Blue:
                    targetPose = fieldLayout
                        .getTagPose(BLUE_DESTINATIONS.get(destination))
                        .get()
                        .toPose2d();
                    break;
                case Red:
                    targetPose = fieldLayout
                        .getTagPose(RED_DESTINATIONS.get(destination))
                        .get()
                        .toPose2d();
                    break;
            }
        } else {
            targetPose = fieldLayout
                .getTagPose(BLUE_DESTINATIONS.get(destination))
                .get()
                .toPose2d();
        }

        Translation2d poseTrasnform;

        if (
            destination == PathfindingDestinations.kCoralTop ||
            destination == PathfindingDestinations.kCoralBottom
        ) {
            poseTrasnform = new Translation2d(0.2, 0.0);
        } else {
            poseTrasnform = new Translation2d(0.5, 0.0);
        }

        Rotation2d transformAngle = targetPose
            .getRotation()
            .rotateBy(Rotation2d.kCW_Pi_2);

        poseTrasnform = poseTrasnform.rotateBy(transformAngle);

        targetPose = targetPose.transformBy(
            new Transform2d(poseTrasnform, transformAngle)
        );

        Logger.recordOutput("AutoFactory/targetPose", targetPose);

        if (!AutoBuilder.isConfigured()) {
            throw new AutoBuilderException(
                "Configure the AutoBuilder before running this method!"
            );
        }

        return AutoBuilder.pathfindToPose(targetPose, pathfindingConstraints);
    }
    // public static SequentialCommandGroup createMultiCoralAuto(
    //     StartPosition startPosition,
    //     PathfindingDestinations firstReefPosition,
    //     int coralToScore,
    //     CoralStation coralStation,
    //     ElevatorPosition reefLevel
    // ) {
    //     SequentialCommandGroup autoGroup = new SequentialCommandGroup();

    //     Optional<Alliance> activeAllianceOptional = DriverStation.getAlliance();

    //     Alliance activeAlliance;

    //     if (activeAllianceOptional.isPresent()) {
    //         activeAlliance = activeAllianceOptional.get();
    //     } else {
    //         activeAlliance = DriverStation.Alliance.Blue;
    //     }

    //     Pose2d startPose = new Pose2d();

    //     if (activeAlliance == DriverStation.Alliance.Blue) {
    //         switch (startPosition) {
    //             case kLeft:
    //                 startPose = new Pose2d(7.58, 6.7, Rotation2d.kCCW_Pi_2);
    //                 break;
    //             case kCenter:
    //                 startPose = new Pose2d(7.58, 4.0, Rotation2d.kCCW_Pi_2);
    //                 break;
    //             case kRight:
    //                 startPose = new Pose2d(7.58, 1.6, Rotation2d.kCCW_Pi_2);
    //                 break;
    //         }
    //     } else {
    //         switch (startPosition) {
    //             case kLeft:
    //                 startPose = new Pose2d(9.97, 1.6, Rotation2d.kCCW_Pi_2);
    //                 break;
    //             case kCenter:
    //                 startPose = new Pose2d(9.97, 4.0, Rotation2d.kCCW_Pi_2);
    //                 break;
    //             case kRight:
    //                 startPose = new Pose2d(9.97, 6.7, Rotation2d.kCCW_Pi_2);
    //                 break;
    //         }
    //     }

    //     autoGroup.addCommands(
    //         DriveSubsystem.getInstance().resetPose2dCommand(startPose)
    //     );

    //     if (coralToScore == 0) {
    //         autoGroup.addCommands(new MoveAuto(Units.feetToMeters(4), 2));

    //         return autoGroup;
    //     } else if (coralToScore == 1) {
    //         autoGroup.addCommands(
    //             getPathfindingCommand(firstReefPosition),
    //             new AlignToReefCommand(
    //                 activeAlliance == Alliance.Red
    //                     ? RED_DESTINATIONS.get(firstReefPosition)
    //                     : BLUE_DESTINATIONS.get(firstReefPosition)
    //             ).alongWith(
    //                 ElevatorSubsystem.getInstance()
    //                     .setElevatorPosition(reefLevel)
    //             ),
    //             reefLevel == ElevatorPosition.L1
    //                 ? IntakeSubsystem.getInstance().outtakeCoralL1()
    //                 : IntakeSubsystem.getInstance().outtakeCoral()
    //         );

    //         return autoGroup;
    //     }
    // }
}
