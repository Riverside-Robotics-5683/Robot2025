package ravenrobotics.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import ravenrobotics.robot.Constants.VisionConstants;

/**
 * Class for estimating the position of a piece of coral relative to the camera.
 */
public class CoralPoseEstimator {

    /** Camera calibration matrix for 3D pose estimation */
    private final Mat calibrationMatrix = new Mat(3, 3, CvType.CV_64F);

    /** Distortion coefficients for correcting lens distortion */
    private final MatOfDouble distortionCoefficients = new MatOfDouble(
        1,
        8,
        CvType.CV_64F
    );

    /** Diameter of the coral in meters (approximately 4 inches) */
    private final double CORAL_DIAMTER = 0.1016;

    /** Length of the coral in meters (approximately 11.8 inches) */
    private final double CORAL_LENGTH = 0.3;

    /** Number of points used to approximate coral's circular cross-section */
    private final int NUM_CIRCLE_POINTS = 16;

    /** Yaw threshold in degrees to determine if coral is viewed from the end */
    private final double END_VIEW_THRESHOLD = 45.0;

    /**
     * Constructor for CoralPoseEstimator.
     *
     * @param coralCamera The PhotonCamera used for detecting coral targets
     */
    public CoralPoseEstimator(PhotonCamera coralCamera) {
        // Get camera matrix from the PhotonCamera or use default values
        Matrix<N3, N3> cameraMatrix;
        var matrixOptional = coralCamera.getCameraMatrix();

        if (matrixOptional.isPresent()) {
            cameraMatrix = matrixOptional.get();
        } else {
            // Use default camera matrix if none is provided
            cameraMatrix = new Matrix<>(
                Nat.N3(),
                Nat.N3(),
                VisionConstants.DEFAULT_CAMERA_MATRIX
            );
        }

        // Get distortion coefficients from the PhotonCamera or use default values
        Matrix<N8, N1> distortionMatrix;
        var distOptional = coralCamera.getDistCoeffs();

        if (distOptional.isPresent()) {
            distortionMatrix = distOptional.get();
        } else {
            // Use default distortion matrix if none is provided
            distortionMatrix = new Matrix<>(
                Nat.N8(),
                Nat.N1(),
                VisionConstants.DEFAULT_DISTORTION_MATRIX
            );
        }

        // Copy camera matrix to OpenCV format
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                calibrationMatrix.put(row, col, cameraMatrix.get(row, col));
            }
        }

        // Copy distortion coefficients to OpenCV format
        for (int col = 0; col < 8; col++) {
            distortionCoefficients.put(0, col, distortionMatrix.get(col, 0));
        }
    }

    /**
     * Estimates the 3D position of a detected coral relative to the camera.
     * Uses PnP (Perspective-n-Point) algorithm to solve for position.
     *
     * @param coral The tracked target representing the coral
     * @return Optional containing the Translation3d position if successful, empty otherwise
     */
    public Optional<Translation3d> estimateCoralPosition(
        PhotonTrackedTarget coral
    ) {
        try {
            // Determine if the coral is being viewed from the end or side
            boolean isEndView = isEndView(coral);

            // Create 3D model points based on viewing angle
            MatOfPoint3f coralPoints = isEndView
                ? createEndViewPoints()
                : createSideViewPoints();

            // Project the 3D model points to 2D image points
            MatOfPoint2f imagePoints = isEndView
                ? getEndViewImagePoints(coral)
                : getSideViewImagePoints(coral);

            // Rotation and translation vectors to be filled by solvePnP
            Mat rvec = new Mat();
            Mat tvec = new Mat();

            // Initial guess for rotation based on target orientation
            Mat initialRvec = getInitialRotationGuess(coral, isEndView);
            Mat initialTvec = new Mat(3, 1, CvType.CV_64F);

            // Estimate initial distance to provide better starting point
            double roughDistance = estimateRoughDistance(coral, isEndView);
            initialTvec.put(0, 0, new double[] { 0, 0, roughDistance });

            // Solve the PnP problem to get object pose relative to camera
            boolean solved = Calib3d.solvePnP(
                coralPoints,
                imagePoints,
                calibrationMatrix,
                distortionCoefficients,
                rvec,
                tvec,
                true // Use initial guess
            );

            if (!solved) {
                return Optional.empty();
            }

            // Extract translation vector to get position
            double[] translation = tvec.get(0, 0);

            // Create Translation3d object with estimated position
            Translation3d position = new Translation3d(
                translation[0],
                translation[1],
                translation[2]
            );

            // Clean up OpenCV resources
            coralPoints.release();
            imagePoints.release();
            rvec.release();
            tvec.release();
            initialRvec.release();
            initialTvec.release();

            return Optional.of(position);
        } catch (Exception e) {
            e.printStackTrace();
            return Optional.empty();
        }
    }

    /**
     * Determines if the coral is being viewed from its end or its side.
     * Uses a combination of aspect ratio and yaw angle to make the determination.
     *
     * @param target The tracked target representing the coral
     * @return true if coral is viewed from the end, false if viewed from the side
     */
    private boolean isEndView(PhotonTrackedTarget target) {
        // Get the detected corners.
        var corners = target.getDetectedCorners();

        if (corners.isEmpty()) {
            // Use the yaw to detect if the camera is viewing the coral side-on or end-on.
            double yaw = Math.abs(target.getYaw());

            return yaw < END_VIEW_THRESHOLD;
        }

        double minX = Double.MAX_VALUE;
        double maxX = Double.MIN_VALUE;
        double minY = Double.MAX_VALUE;
        double maxY = Double.MIN_VALUE;

        // Find the bounding box.
        for (TargetCorner corner : corners) {
            minX = Math.min(minX, corner.x);
            maxX = Math.max(maxX, corner.x);
            minY = Math.min(minY, corner.y);
            maxY = Math.min(maxY, corner.y);
        }

        double width = maxX - minX;
        double height = maxY - minY;

        // Find the aspect ratio.
        double aspectRatio = width / height;

        // End view should have aspect ratio close to 1 (circle)
        boolean aspectRatioSuggestsEndView = Math.abs(aspectRatio - 1.0) < 0.2;

        double yaw = Math.abs(target.getYaw());
        boolean angleIndicatesEndView = yaw < END_VIEW_THRESHOLD;

        return aspectRatioSuggestsEndView || angleIndicatesEndView;
    }

    /**
     * Creates 3D model points for a coral viewed from its side.
     * Models the coral as a cylinder with circular ends.
     *
     * @return MatOfPoint3f containing the 3D coordinates of model points
     */
    private MatOfPoint3f createSideViewPoints() {
        List<Point3> points = new ArrayList<>();

        double radius = CORAL_DIAMTER / 2.0;

        // Create points around the circumference at both ends of the cylinder
        for (int i = 0; i < NUM_CIRCLE_POINTS; i++) {
            double angle = (2.0 * Math.PI * i) / NUM_CIRCLE_POINTS;

            double x = radius * Math.cos(angle);
            double y = radius * Math.sin(angle);

            // Add point at front face (z=0)
            points.add(new Point3(x, y, 0));
            // Add corresponding point at back face (z=length)
            points.add(new Point3(x, y, CORAL_LENGTH));
        }

        // Add additional key points at 90-degree intervals
        for (int i = 0; i < 4; i++) {
            double angle = (2.0 * Math.PI * i) / 4;

            double x = radius * Math.cos(angle);
            double y = radius * Math.sin(angle);

            points.add(new Point3(x, y, 0));
            points.add(new Point3(x, y, CORAL_LENGTH));
        }

        MatOfPoint3f objectPoints = new MatOfPoint3f();
        objectPoints.fromList(points);

        return objectPoints;
    }

    /**
     * Creates 3D model points for a coral viewed from its end.
     * Models the coral as a cylinder with the main focus on the circular end face.
     *
     * @return MatOfPoint3f containing the 3D coordinates of model points
     */
    private MatOfPoint3f createEndViewPoints() {
        List<Point3> points = new ArrayList<>();

        double radius = CORAL_DIAMTER / 2.0;

        // Create points around the circumference of the end face
        for (int i = 0; i < NUM_CIRCLE_POINTS; i++) {
            double angle = (2.0 * Math.PI * i) / NUM_CIRCLE_POINTS;

            double x = radius * Math.cos(angle);
            double y = radius * Math.sin(angle);

            points.add(new Point3(x, y, 0));
        }

        // Add center point of circle
        points.add(new Point3(0, 0, 0));

        // Add depth information with points slightly inside the cylinder
        double depthOffset = CORAL_LENGTH * 0.25;

        // Add points at a smaller radius inside the cylinder for depth perception
        for (int i = 0; i < 4; i++) {
            double angle = (2.0 * Math.PI * i) / 4;

            double x = radius * 0.8 * Math.cos(angle);
            double y = radius * 0.8 * Math.sin(angle);

            points.add(new Point3(x, y, depthOffset));
        }

        MatOfPoint3f objectPoints = new MatOfPoint3f();
        objectPoints.fromList(points);

        return objectPoints;
    }

    /**
     * Projects 3D model points to 2D image points for a side view of the coral.
     * Creates an elliptical projection representing the cylindrical shape.
     *
     * @param target The tracked target representing the coral
     * @return MatOfPoint2f containing the 2D image points
     */
    private MatOfPoint2f getSideViewImagePoints(PhotonTrackedTarget target) {
        List<Point> points = new ArrayList<>();

        var corners = target.getDetectedCorners();

        // Find bounding box of the detected object
        double minX = Double.MAX_VALUE;
        double maxX = Double.MIN_VALUE;
        double minY = Double.MAX_VALUE;
        double maxY = Double.MIN_VALUE;

        for (var corner : corners) {
            minX = Math.min(minX, corner.x);
            maxX = Math.max(maxX, corner.x);
            minY = Math.min(minY, corner.y);
            maxY = Math.min(maxY, corner.y);
        }

        // Calculate center and dimensions
        double centerX = (minX + maxX) / 2.0;
        double centerY = (minY + maxY) / 2.0;

        double width = maxX - minX;
        double height = maxY - minY;

        // Calculate radii for front face (larger ellipse)
        double frontRadiusX = width / 2.0;
        double frontRadiusY = height / 2.0;

        // Calculate radii for back face (smaller ellipse due to perspective)
        double perspectiveScale = 0.85;
        double backRadiusX = frontRadiusX * perspectiveScale;
        double backRadiusY = frontRadiusY * perspectiveScale;
        double backCenterX = centerX + (width * 0.05); // Slight shift for perspective

        // Generate points around front face (ellipse)
        for (int i = 0; i < NUM_CIRCLE_POINTS; i++) {
            double angle = (2.0 * Math.PI * i) / NUM_CIRCLE_POINTS;

            double x = centerX + frontRadiusX * Math.cos(angle);
            double y = centerY + frontRadiusY * Math.sin(angle);

            points.add(new Point(x, y));
        }

        // Generate points around back face (smaller ellipse)
        for (int i = 0; i < NUM_CIRCLE_POINTS; i++) {
            double angle = (2.0 * Math.PI * i) / NUM_CIRCLE_POINTS;

            double x = backCenterX + backRadiusX * Math.cos(angle);
            double y = centerY + backRadiusY * Math.sin(angle);

            points.add(new Point(x, y));
        }

        // Add key points at 90-degree intervals on both front and back faces
        for (int i = 0; i < 4; i++) {
            double angle = (2.0 * Math.PI * i) / 4;

            // Front face point
            double frontX = centerX + frontRadiusX * Math.cos(angle);
            double frontY = centerY + frontRadiusY * Math.sin(angle);
            points.add(new Point(frontX, frontY));

            // Back face point
            double backX = backCenterX + backRadiusX * Math.cos(angle);
            double backY = centerY + backRadiusY * Math.sin(angle);
            points.add(new Point(backX, backY));
        }

        MatOfPoint2f imagePoints = new MatOfPoint2f();
        imagePoints.fromList(points);

        return imagePoints;
    }

    /**
     * Projects 3D model points to 2D image points for an end view of the coral.
     * Creates a circular projection with additional inner points for depth perception.
     *
     * @param target The tracked target representing the coral
     * @return MatOfPoint2f containing the 2D image points
     */
    private MatOfPoint2f getEndViewImagePoints(PhotonTrackedTarget target) {
        List<Point> points = new ArrayList<>();

        var corners = target.getDetectedCorners();

        // Find bounding box of the detected object
        double minX = Double.MAX_VALUE;
        double maxX = Double.MIN_VALUE;
        double minY = Double.MAX_VALUE;
        double maxY = Double.MIN_VALUE;

        for (var corner : corners) {
            minX = Math.min(minX, corner.x);
            maxX = Math.max(maxX, corner.x);
            minY = Math.min(minY, corner.y);
            maxY = Math.max(maxY, corner.y);
        }

        // Calculate center and radius
        double centerX = (minX + maxX) / 2.0;
        double centerY = (minY + maxY) / 2.0;

        // Use the smaller dimension for radius to ensure circle fits in bounding box
        double radius = Math.min((maxX - minX) / 2.0, (maxY - minY) / 2.0);

        // Generate points around the circumference
        for (int i = 0; i < NUM_CIRCLE_POINTS; i++) {
            double angle = (2.0 * Math.PI * i) / NUM_CIRCLE_POINTS;

            double x = centerX + radius * Math.cos(angle);
            double y = centerY + radius * Math.sin(angle);

            points.add(new Point(x, y));
        }

        // Add center point
        points.add(new Point(centerX, centerY));

        // Add inner points at 90-degree intervals for depth perception
        double innerRadius = radius * 0.8;
        for (int i = 0; i < 4; i++) {
            double angle = (2.0 * Math.PI * i) / 4;

            double x = centerX + innerRadius * Math.cos(angle);
            double y = centerY + innerRadius * Math.sin(angle);

            points.add(new Point(x, y));
        }

        MatOfPoint2f imagePoints = new MatOfPoint2f();
        imagePoints.fromList(points);

        return imagePoints;
    }

    /**
     * Creates an initial rotation guess for the PnP solver based on target orientation.
     * Provides different rotation matrices for end view vs side view.
     *
     * @param target The tracked target representing the coral
     * @param isEndView Whether the coral is viewed from its end or side
     * @return Mat containing the initial rotation vector guess
     */
    private Mat getInitialRotationGuess(
        PhotonTrackedTarget target,
        boolean isEndView
    ) {
        Mat rvec = new Mat(3, 1, CvType.CV_64F);

        if (isEndView) {
            // For end view, use yaw and pitch with no roll
            double yaw = Math.toRadians(target.getYaw());
            double pitch = Math.toRadians(target.getPitch());

            rvec.put(0, 0, new double[] { pitch, yaw, 0 });
        } else {
            // For side view, add 90 degrees roll to account for cylindrical orientation
            double yaw = Math.toRadians(target.getYaw());
            double pitch = Math.toRadians(target.getPitch());

            rvec.put(0, 0, new double[] { pitch, yaw, Math.PI / 2 });
        }

        return rvec;
    }

    /**
     * Estimates the approximate distance to the coral based on its apparent size.
     * Uses the pinhole camera model: distance = (actual_size * focal_length) / apparent_size
     *
     * @param target The tracked target representing the coral
     * @param isEndView Whether the coral is viewed from its end or side
     * @return Estimated distance in meters
     */
    private double estimateRoughDistance(
        PhotonTrackedTarget target,
        boolean isEndView
    ) {
        var corners = target.getDetectedCorners();

        // Find bounding box of the detected object
        double minX = Double.MAX_VALUE;
        double maxX = Double.MIN_VALUE;
        double minY = Double.MAX_VALUE;
        double maxY = Double.MIN_VALUE;

        for (var corner : corners) {
            minX = Math.min(minX, corner.x);
            maxX = Math.max(maxX, corner.x);
            minY = Math.min(minY, corner.y);
            maxY = Math.min(maxY, corner.y);
        }

        double width = maxX - minX;
        double height = maxY - minY;

        // Get focal length from calibration matrix
        double fx = calibrationMatrix.get(0, 0)[0];

        // Use largest dimension of target in pixels
        double apparentSize = Math.max(width, height);

        // Select appropriate actual size based on view orientation
        double actualSize = isEndView
            ? CORAL_DIAMTER // End view shows the diameter
            : Math.max(CORAL_DIAMTER, CORAL_LENGTH); // Side view shows length or diameter

        // Apply pinhole camera formula: distance = (actual_size * focal_length) / apparent_size
        return (actualSize * fx) / apparentSize;
    }
}
