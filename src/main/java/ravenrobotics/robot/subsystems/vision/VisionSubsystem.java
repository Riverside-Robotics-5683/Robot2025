package ravenrobotics.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

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

    /** Transform offset for the left camera position relative to robot center */
    private final Transform3d leftOffset = new Transform3d(
        new Translation3d(0.5, 0.0, 0.0),
        new Rotation3d(0, 0, 0)
    );
    /** Transform offset for the right camera position relative to robot center */
    private final Transform3d rightOffset = new Transform3d(
        new Translation3d(-0.5, 0.0, 0.0),
        new Rotation3d(0, 0, 0)
    );

    /** Coral camera for object detection. */
    private final PhotonCamera coralCamera = new PhotonCamera("coralCamera");

    /** Pose estimator using the left camera */
    private final PhotonPoseEstimator leftEstimator = new PhotonPoseEstimator(
        fieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        leftOffset
    );
    /** Pose estimator using the right camera */
    private final PhotonPoseEstimator rightEstimator = new PhotonPoseEstimator(
        fieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        rightOffset
    );

    private List<PhotonPipelineResult> leftResults;
    private List<PhotonPipelineResult> rightResults;

    /** Most recent pose estimate from the left camera */
    private EstimatedRobotPose leftPose;
    /** Most recent pose estimate from the right camera */
    private EstimatedRobotPose rightPose;

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
     * Gets the estimated pose from the left camera.
     *
     * @return The estimated robot pose.
     */
    public EstimatedRobotPose getLeftEstimatedPose() {
        return leftPose;
    }

    /**
     * Gets the estimated pose from the right camera.
     *
     * @return The estimated robot pose.
     */
    public EstimatedRobotPose getRightEstimatedPose() {
        return rightPose;
    }

    /**
     * Gets the detected AprilTags from the left camera.
     *
     * @return The detected AprilTags.
     */
    public PhotonPipelineResult getIntakeAprilTags() {
        // Make sure there are actually results before trying to return anything.
        if (!leftResults.isEmpty()) {
            return leftResults.get(0);
        }

        return null;
    }

    /**
     * Get the detected coral from the coral camera.
     *
     * @return The coral that has been detected.
     */
    public PhotonPipelineResult getDetectedCoral() {
        var results = coralCamera.getAllUnreadResults();

        if (!results.isEmpty()) {
            return results.get(0);
        }

        return null;
    }

    @Override
    public void periodic() {
        // Get all unread results from both cameras
        leftResults = leftLocalizer.getAllUnreadResults();
        rightResults = rightLocalizer.getAllUnreadResults();

        // Process left camera results if available
        if (!leftResults.isEmpty()) {
            var result = leftResults.get(0);

            // If AprilTags are detected
            if (result.hasTargets()) {
                var pose = leftEstimator.update(result);

                // Update the left pose estimate if valid
                if (pose.isPresent()) {
                    leftPose = pose.get();
                }
            }
        }

        // Process right camera results if available
        if (!rightResults.isEmpty()) {
            var result = rightResults.get(0);

            // If AprilTags are detected
            if (result.hasTargets()) {
                var pose = rightEstimator.update(result);

                // Update the right pose estimate if valid
                if (pose.isPresent()) {
                    rightPose = pose.get();
                }
            }
        }
    }
}
