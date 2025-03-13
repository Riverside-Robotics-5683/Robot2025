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

    private final Mat calibrationMatrix = new Mat(3, 3, CvType.CV_64F);
    private final MatOfDouble distortionCoefficients = new MatOfDouble(
        1,
        8,
        CvType.CV_64F
    );

    private final double CORAL_DIAMTER = 0.1016;
    private final double CORAL_LENGTH = 0.3;
    private final int NUM_CIRCLE_POINTS = 16;
    private final double END_VIEW_THRESHOLD = 45.0;

    public CoralPoseEstimator(PhotonCamera coralCamera) {
        Matrix<N3, N3> cameraMatrix;
        var matrixOptional = coralCamera.getCameraMatrix();

        if (matrixOptional.isPresent()) {
            cameraMatrix = matrixOptional.get();
        } else {
            cameraMatrix = new Matrix<>(
                Nat.N3(),
                Nat.N3(),
                VisionConstants.DEFAULT_CAMERA_MATRIX
            );
        }

        Matrix<N8, N1> distortionMatrix;
        var distOptional = coralCamera.getDistCoeffs();

        if (distOptional.isPresent()) {
            distortionMatrix = distOptional.get();
        } else {
            distortionMatrix = new Matrix<>(
                Nat.N8(),
                Nat.N1(),
                VisionConstants.DEFAULT_DISTORTION_MATRIX
            );
        }

        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                calibrationMatrix.put(row, col, cameraMatrix.get(row, col));
            }
        }

        for (int col = 0; col < 8; col++) {
            distortionCoefficients.put(0, col, distortionMatrix.get(col, 0));
        }
    }

    public Optional<Translation3d> estimateCoralPosition(
        PhotonTrackedTarget coral
    ) {
        try {
            boolean isEndView = isEndView(coral);

            MatOfPoint3f coralPoints = isEndView
                ? createEndViewPoints()
                : createSideViewPoints();

            MatOfPoint2f imagePoints = isEndView
                ? getEndViewImagePoints(coral)
                : getSideViewImagePoints(coral);

            Mat rvec = new Mat();
            Mat tvec = new Mat();

            Mat initialRvec = getInitialRotationGuess(coral, isEndView);
            Mat initialTvec = new Mat(3, 1, CvType.CV_64F);

            double roughDistance = estimateRoughDistance(coral, isEndView);
            initialTvec.put(0, 0, new double[] { 0, 0, roughDistance });

            boolean solved = Calib3d.solvePnP(
                coralPoints,
                imagePoints,
                calibrationMatrix,
                distortionCoefficients,
                rvec,
                tvec,
                true
            );

            if (!solved) {
                return Optional.empty();
            }

            double[] translation = tvec.get(0, 0);

            Translation3d position = new Translation3d(
                translation[0],
                translation[1],
                translation[2]
            );

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

        boolean aspectRatioSuggestsEndView = Math.abs(aspectRatio - 1.0) < 0.2;

        double yaw = Math.abs(target.getYaw());
        boolean angleIndicatesEndView = yaw < END_VIEW_THRESHOLD;

        return aspectRatioSuggestsEndView || angleIndicatesEndView;
    }

    private MatOfPoint3f createSideViewPoints() {
        List<Point3> points = new ArrayList<>();

        double radius = CORAL_DIAMTER / 2.0;

        for (int i = 0; i < NUM_CIRCLE_POINTS; i++) {
            double angle = (2.0 * Math.PI * i) / NUM_CIRCLE_POINTS;

            double x = radius * Math.cos(angle);
            double y = radius * Math.sin(angle);

            points.add(new Point3(x, y, 0));
            points.add(new Point3(x, y, CORAL_LENGTH));
        }

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

    private MatOfPoint3f createEndViewPoints() {
        List<Point3> points = new ArrayList<>();

        double radius = CORAL_DIAMTER / 2.0;

        for (int i = 0; i < NUM_CIRCLE_POINTS; i++) {
            double angle = (2.0 * Math.PI * i) / NUM_CIRCLE_POINTS;

            double x = radius * Math.cos(angle);
            double y = radius * Math.sin(angle);

            points.add(new Point3(x, y, 0));
        }

        points.add(new Point3(0, 0, 0));

        double depthOffset = CORAL_LENGTH * 0.25;

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

    private MatOfPoint2f getSideViewImagePoints(PhotonTrackedTarget target) {
        List<Point> points = new ArrayList<>();

        var corners = target.getDetectedCorners();

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

        double centerX = (minX + maxX) / 2.0;
        double centerY = (minY + maxY) / 2.0;

        double width = maxX - minX;
        double height = maxY - minY;

        double frontRadiusX = width / 2.0;
        double frontRadiusY = height / 2.0;

        double perspectiveScale = 0.85;
        double backRadiusX = frontRadiusX * perspectiveScale;
        double backRadiusY = frontRadiusY * perspectiveScale;
        double backCenterX = centerX + (width * 0.05);

        for (int i = 0; i < NUM_CIRCLE_POINTS; i++) {
            double angle = (2.0 * Math.PI * i) / NUM_CIRCLE_POINTS;

            double x = centerX + frontRadiusX * Math.cos(angle);
            double y = centerY + frontRadiusY * Math.sin(angle);

            points.add(new Point(x, y));
        }

        for (int i = 0; i < NUM_CIRCLE_POINTS; i++) {
            double angle = (2.0 * Math.PI * i) / NUM_CIRCLE_POINTS;

            double x = backCenterX + backRadiusX * Math.cos(angle);
            double y = centerY + backRadiusY * Math.sin(angle);

            points.add(new Point(x, y));
        }

        for (int i = 0; i < 4; i++) {
            double angle = (2.0 * Math.PI * i) / 4;

            double frontX = centerX + frontRadiusX * Math.cos(angle);
            double frontY = centerY + frontRadiusY * Math.sin(angle);
            points.add(new Point(frontX, frontY));

            double backX = backCenterX + backRadiusX * Math.cos(angle);
            double backY = centerY + backRadiusY * Math.sin(angle);
            points.add(new Point(backX, backY));
        }

        MatOfPoint2f imagePoints = new MatOfPoint2f();
        imagePoints.fromList(points);

        return imagePoints;
    }

    private MatOfPoint2f getEndViewImagePoints(PhotonTrackedTarget target) {
        List<Point> points = new ArrayList<>();

        var corners = target.getDetectedCorners();

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

        double centerX = (minX + maxX) / 2.0;
        double centerY = (minY + maxY) / 2.0;

        double radius = Math.min((maxX - minX) / 2.0, (maxY - minY) / 2.0);

        for (int i = 0; i < NUM_CIRCLE_POINTS; i++) {
            double angle = (2.0 * Math.PI * i) / NUM_CIRCLE_POINTS;

            double x = centerX + radius * Math.cos(angle);
            double y = centerY + radius * Math.sin(angle);

            points.add(new Point(x, y));
        }

        points.add(new Point(centerX, centerY));

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

    private Mat getInitialRotationGuess(
        PhotonTrackedTarget target,
        boolean isEndView
    ) {
        Mat rvec = new Mat(3, 1, CvType.CV_64F);

        if (isEndView) {
            double yaw = Math.toRadians(target.getYaw());
            double pitch = Math.toRadians(target.getPitch());

            rvec.put(0, 0, new double[] { pitch, yaw, 0 });
        } else {
            double yaw = Math.toRadians(target.getYaw());
            double pitch = Math.toRadians(target.getPitch());

            rvec.put(0, 0, new double[] { pitch, yaw, Math.PI / 2 });
        }

        return rvec;
    }

    private double estimateRoughDistance(
        PhotonTrackedTarget target,
        boolean isEndView
    ) {
        var corners = target.getDetectedCorners();

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

        double fx = calibrationMatrix.get(0, 0)[0];

        double apparentSize = Math.max(width, height);
        double actualSize = isEndView
            ? CORAL_DIAMTER
            : Math.max(CORAL_DIAMTER, CORAL_LENGTH);

        return (actualSize * fx) / apparentSize;
    }
}
