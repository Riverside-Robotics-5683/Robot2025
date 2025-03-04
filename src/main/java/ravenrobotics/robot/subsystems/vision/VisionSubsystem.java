package ravenrobotics.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
