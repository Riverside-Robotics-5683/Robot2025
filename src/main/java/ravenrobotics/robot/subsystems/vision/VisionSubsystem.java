package ravenrobotics.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
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

    private EstimatedRobotPose leftEstimatedPose;
    private EstimatedRobotPose rightEstimatedPose;

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

    /**
     * Retrieves AprilTags detected by the left camera.
     *
     * @return An Optional containing a list of PhotonTrackedTargets if available,
     *         or empty if no results exist
     */
    public Optional<List<PhotonTrackedTarget>> getLeftAprilTags() {
        if (leftResults.isEmpty()) {
            return Optional.empty();
        }

        return Optional.ofNullable(
            leftResults.get(leftResults.size() - 1).getTargets()
        );
    }

    /**
     * Retrieves coral targets detected by the coral camera.
     *
     * @return An Optional containing a list of PhotonTrackedTargets if available,
     *         or empty if no results exist
     * @throws IllegalAccessException If the coral camera is not set to the coral pipeline
     */
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

    /**
     * Retrieves AprilTags detected by the coral camera.
     *
     * @return An Optional containing a list of PhotonTrackedTargets if available,
     *         or empty if no results exist
     * @throws IllegalAccessException If the coral camera is not set to the AprilTag pipeline
     */
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

    /**
     * Sets the coral camera pipeline to detect coral.
     */
    public void setCoralToCoral() {
        coralCamera.setPipelineIndex(0);
    }

    /**
     * Sets the coral camera pipeline to detect AprilTags.
     */
    public void setCoralToAprilTags() {
        coralCamera.setPipelineIndex(1);
    }

    public AprilTagFieldLayout getFieldLayout() {
        return fieldLayout;
    }

    /**
     * Checks if the coral camera is currently set to detect AprilTags.
     *
     * @return true if the coral camera is set to the AprilTag pipeline, false otherwise
     */
    public boolean isCoralCameraAprilTags() {
        return coralCamera.getPipelineIndex() == 1;
    }

    /**
     * Calculates the mean (average) estimated pose between left and right camera pose estimations.
     *
     * @return An Optional containing the interpolated Pose3d if at least one valid pose exists,
     *         or an empty Optional if no valid poses are available
     */
    public Optional<Pose3d> getMeanEstimatedPose() {
        // Extract poses from EstimatedRobotPose objects, handling null cases
        Pose3d leftPose = leftEstimatedPose != null
            ? leftEstimatedPose.estimatedPose
            : null;

        Pose3d rightPose = rightEstimatedPose != null
            ? rightEstimatedPose.estimatedPose
            : null;

        // If left pose is missing, return right pose (or empty if also null)
        if (leftPose == null) {
            return Optional.ofNullable(rightPose);
        }

        // If right pose is missing, return left pose
        if (rightPose == null) {
            return Optional.ofNullable(leftPose);
        }

        // Both poses exist, return their interpolation (50% blend)
        return Optional.of(
            leftEstimatedPose.estimatedPose.interpolate(
                rightEstimatedPose.estimatedPose,
                0.5
            )
        );
    }

    @Override
    public void periodic() {
        // Collect all unread vision processing results from the cameras
        leftResults = leftLocalizer.getAllUnreadResults();
        rightResults = rightLocalizer.getAllUnreadResults();
        coralResults = coralCamera.getAllUnreadResults();

        // Create a list to store valid pose estimations
        List<EstimatedRobotPose> estimatedPoses = new ArrayList<>();

        // Process results from left camera if available
        if (!leftResults.isEmpty()) {
            // Update pose estimator with most recent result from left camera
            Optional<EstimatedRobotPose> leftEstimatedPoseOptional =
                leftEstimator.update(leftResults.get(leftResults.size() - 1));

            // Add valid pose estimation to the collection
            if (leftEstimatedPoseOptional.isPresent()) {
                leftEstimatedPose = leftEstimatedPoseOptional.get();
                estimatedPoses.add(leftEstimatedPose);
                // Log the estimated pose.
                Logger.recordOutput(
                    "Vision/leftEstimatedPose",
                    leftEstimatedPose.estimatedPose
                );
            }
        }

        // Process results from right camera if available
        if (!rightResults.isEmpty()) {
            // Update pose estimator with most recent result from right camera
            Optional<EstimatedRobotPose> rightEstimatedPoseOptional =
                rightEstimator.update(
                    rightResults.get(rightResults.size() - 1)
                );

            // Add valid pose estimation to the collection
            if (rightEstimatedPoseOptional.isPresent()) {
                rightEstimatedPose = rightEstimatedPoseOptional.get();
                estimatedPoses.add(rightEstimatedPose);
                Logger.recordOutput(
                    "Vision/rightEstimatedPose",
                    rightEstimatedPose.estimatedPose
                );
            }
        }

        // If we have valid pose estimations, submit them to the drive subsystem
        if (!estimatedPoses.isEmpty()) {
            DriveSubsystem.getInstance()
                .submitVisionMeasurements(estimatedPoses);
        }
    }
}
