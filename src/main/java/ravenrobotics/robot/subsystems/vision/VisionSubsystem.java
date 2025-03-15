package ravenrobotics.robot.subsystems.vision;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.AutoBuilderException;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import ravenrobotics.robot.subsystems.drive.DriveSubsystem;

public class VisionSubsystem extends SubsystemBase {

    /** Field layout for AprilTag detection and pose estimation */
    private final AprilTagFieldLayout fieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    /** Camera for the left side vision processing */
    private final PhotonCamera leftLocalizer = new PhotonCamera(
        "leftLocalizer"
    );
    /** Camera for the right side vision processing */
    private final PhotonCamera rightLocalizer = new PhotonCamera(
        "rightLocalizer"
    );

    private final PhotonCamera coralCamera = new PhotonCamera("coralCamera");

    /** Transform offset for the left camera position relative to robot center */
    private final Transform3d leftOffset = new Transform3d(
        new Translation3d(0.2667, 0.0, 0.3302),
        new Rotation3d(-90, 90, 0)
    );
    /** Transform offset for the right camera position relative to robot center */
    private final Transform3d rightOffset = new Transform3d(
        new Translation3d(-0.3175, 0.0, 0.7112),
        new Rotation3d(90, 90, 0)
    );

    private final PhotonPoseEstimator leftEstimator = new PhotonPoseEstimator(
        fieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        leftOffset
    );
    private final PhotonPoseEstimator rightEstimator = new PhotonPoseEstimator(
        fieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        rightOffset
    );

    private List<PhotonPipelineResult> leftResults = new ArrayList<>();
    private List<PhotonPipelineResult> rightResults = new ArrayList<>();
    private List<PhotonPipelineResult> coralResults = new ArrayList<>();

    public enum AlignmentTarget {
        kReef,
        kCoralStation,
    }

    /** Singleton instance of the vision subsystem */
    private static VisionSubsystem instance;

    /**
     * Gets an instance of VisionSubsystem.
     *
     * @return The VisionSubsystem instance.
     */
    public static VisionSubsystem getInstance() {
        // If an instance doesn't exist, create one
        if (instance == null) {
            instance = new VisionSubsystem();
        }

        return instance;
    }

    public Optional<List<PhotonTrackedTarget>> getLeftAprilTags() {
        if (leftResults.isEmpty()) {
            return Optional.empty();
        }

        return Optional.ofNullable(
            leftResults.get(leftResults.size() - 1).getTargets()
        );
    }

    public Optional<List<PhotonTrackedTarget>> getVisibleCoral()
        throws IllegalAccessException {
        if (coralCamera.getPipelineIndex() != 0) {
            throw new IllegalAccessException(
                "coralCamera must be set to the coral pipeline!"
            );
        }

        if (coralResults.isEmpty()) {
            return Optional.empty();
        }

        return Optional.ofNullable(
            coralResults.get(coralResults.size() - 1).getTargets()
        );
    }

    public Optional<List<PhotonTrackedTarget>> getCoralAprilTags()
        throws IllegalAccessException {
        if (coralCamera.getPipelineIndex() != 1) {
            throw new IllegalAccessException(
                "coralCamera must be set to the AprilTag pipeline!"
            );
        }

        if (coralResults.isEmpty()) {
            return Optional.empty();
        }

        return Optional.ofNullable(
            coralResults.get(coralResults.size() - 1).getTargets()
        );
    }

    public void setCoralToCoral() {
        coralCamera.setPipelineIndex(0);
    }

    public void setCoralToAprilTags() {
        coralCamera.setPipelineIndex(1);
    }

    public boolean isCoralCameraAprilTags() {
        return coralCamera.getPipelineIndex() == 1;
    }

    /**
     * Creates a command to automatically align the robot with a specified target.
     *
     * <p>This method finds AprilTags associated with the requested target type (reef or coral station),
     * calculates the optimal position for alignment, and generates a path-following command to
     * navigate there.
     *
     * @param target The type of game element to align with (Reef or Coral Station)
     * @param currentPose The robot's current position on the field
     * @return An Optional containing the path-following command if a valid alignment target was found,
     *         or empty if no valid targets were detected
     */
    public Optional<Command> getAlignmentCommand(
        AlignmentTarget target,
        Pose2d currentPose
    ) {
        // Set coral camera to detect AprilTags
        setCoralToAprilTags();

        Optional<List<PhotonTrackedTarget>> aprilTagsOptional =
            Optional.empty();

        int errorCounter = 0;

        // Get AprilTag data from the appropriate camera based on target type
        switch (target) {
            case kReef:
                // For reef alignment, use the left camera
                aprilTagsOptional = getLeftAprilTags();
                break;
            case kCoralStation:
                // For coral station alignment, use the coral camera
                // Try up to 10 times to get valid data
                while (aprilTagsOptional.isEmpty()) {
                    try {
                        aprilTagsOptional = getCoralAprilTags();
                    } catch (IllegalAccessException e) {
                        errorCounter++;

                        // Exit after 10 failed attempts
                        if (errorCounter > 10) {
                            return Optional.empty();
                        }

                        // Brief pause between attempts
                        Timer.delay(0.1);
                    }
                }
                break;
        }

        // If no AprilTags were detected, return empty
        if (aprilTagsOptional.isEmpty()) {
            return Optional.empty();
        }

        List<PhotonTrackedTarget> targets = aprilTagsOptional.get();

        Pose3d targetPose = null;
        double targetScore = Double.MAX_VALUE;

        // Process each detected AprilTag
        for (PhotonTrackedTarget tag : targets) {
            // Check if this tag is valid for the selected target type
            boolean isValidTag = (target == AlignmentTarget.kReef)
                ? isReefTag(tag.getFiducialId())
                : isCoralStationTag(tag.getFiducialId());

            if (!isValidTag) {
                continue;
            }

            // Get the tag's position in field coordinates
            Optional<Pose3d> tagPoseOptional = fieldLayout.getTagPose(
                tag.getFiducialId()
            );

            if (tagPoseOptional.isEmpty()) {
                continue;
            }

            Pose3d tagPose = tagPoseOptional.get();

            // Calculate a position 25cm in front of the tag
            // and rotated 90 degrees relative to the tag's orientation
            Pose3d potentialPose = new Pose3d(
                tagPose.getX() - 0.25 * Math.cos(tagPose.getRotation().getZ()),
                tagPose.getY() - 0.25 * Math.sin(tagPose.getRotation().getZ()),
                tagPose.getZ(),
                new Rotation3d(
                    0,
                    0,
                    tagPose.getRotation().getZ() + Math.PI * 0.5
                )
            );

            // Score this position based on distance from current robot position
            // Lower scores (shorter distances) are preferred
            double score = potentialPose
                .getTranslation()
                .getDistance(
                    new Translation3d(currentPose.getX(), currentPose.getY(), 0)
                );

            // Keep track of the best (closest) alignment position
            if (score < targetScore) {
                targetPose = potentialPose;
                targetScore = score;
            }
        }

        // If no valid alignment position was found, return empty
        if (targetPose == null) {
            return Optional.empty();
        }

        // Create a path from current position to the target position
        List<Waypoint> pathWaypoints = PathPlannerPath.waypointsFromPoses(
            currentPose,
            targetPose.toPose2d()
        );

        // Define path constraints (speed, acceleration, etc.)
        PathConstraints pathConstraints = new PathConstraints(
            2.0, // Max velocity (m/s)
            2.5, // Max acceleration (m/s²)
            Math.PI, // Max angular velocity (rad/s)
            2.0 * Math.PI // Max angular acceleration (rad/s²)
        );

        // Create the path with the calculated end position and rotation
        PathPlannerPath alignmentPath = new PathPlannerPath(
            pathWaypoints,
            pathConstraints,
            null,
            new GoalEndState(0.0, targetPose.getRotation().toRotation2d())
        );

        // Generate and return the path-following command
        try {
            return Optional.of(AutoBuilder.followPath(alignmentPath));
        } catch (AutoBuilderException e) {
            return Optional.empty();
        }
    }

    private boolean isReefTag(int id) {
        switch (id) {
            case 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22:
                return true;
            default:
                return false;
        }
    }

    private boolean isCoralStationTag(int id) {
        switch (id) {
            case 1, 2, 12, 13:
                return true;
            default:
                return false;
        }
    }

    @Override
    public void periodic() {
        leftResults = leftLocalizer.getAllUnreadResults();
        rightResults = rightLocalizer.getAllUnreadResults();

        coralResults = coralCamera.getAllUnreadResults();

        List<EstimatedRobotPose> estimatedPoses = new ArrayList<>();

        if (!leftResults.isEmpty()) {
            Optional<EstimatedRobotPose> leftEstimatedPose =
                leftEstimator.update(leftResults.get(leftResults.size() - 1));

            if (leftEstimatedPose.isPresent()) {
                estimatedPoses.add(leftEstimatedPose.get());
            }
        }

        if (!rightResults.isEmpty()) {
            Optional<EstimatedRobotPose> rightEstimatedPose =
                rightEstimator.update(
                    rightResults.get(rightResults.size() - 1)
                );

            if (rightEstimatedPose.isPresent()) {
                estimatedPoses.add(rightEstimatedPose.get());
            }
        }

        if (!estimatedPoses.isEmpty()) {
            DriveSubsystem.getInstance()
                .submitVisionMeasurements(estimatedPoses);
        }
    }
}
