package ravenrobotics.robot.subsystems.drive;

import static ravenrobotics.robot.Constants.KinematicsConstants.SWERVE_KINEMATICS;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import ravenrobotics.robot.Constants.AutoConstants;
import ravenrobotics.robot.Constants.KinematicsConstants;
import ravenrobotics.robot.Constants.SwerveModuleKeys;
import ravenrobotics.robot.subsystems.vision.VisionSubsystem;

/**
 * Subsystem for controlling the drive base.
 */
public class DriveSubsystem extends SubsystemBase {

    private final SwerveModule[] swerveModules = new SwerveModule[4]; // The swerve modules for controlling motors.
    // Swerve module inputs for AdvantageKit.
    private SwerveModuleInputsAutoLogged[] swerveModuleInputs =
        new SwerveModuleInputsAutoLogged[4];

    private final IMU imu; // The IMU for getting heading and orientation.
    private IMUInputsAutoLogged imuInputs = new IMUInputsAutoLogged(); // The IMU inputs for AdvantageKit.

    private SwerveModulePosition[] swerveModulePositions =
        new SwerveModulePosition[4]; // The swerve module positions.
    private SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4]; // The swerve module states (speed).

    // Pose estimator combining drive odometry and vision measurements for more
    // accurate odometry.
    private SwerveDrivePoseEstimator3d estimatedPose;

    private Field2d fieldWidget;

    private static DriveSubsystem instance; // The static instance for the getInstance method.

    private DriveSubsystem() {
        for (int i = 0; i < 4; i++) {
            // Create all of the swerve modules.
            swerveModules[i] = new SwerveModule(i);

            swerveModulePositions[i] = new SwerveModulePosition();
            swerveModuleStates[i] = new SwerveModuleState();

            swerveModuleInputs[i] = new SwerveModuleInputsAutoLogged();
        }

        imu = new IMU();

        // Create the IMU inputs.
        imuInputs = new IMUInputsAutoLogged();

        // Create the initial state of the pose estimator.
        estimatedPose = new SwerveDrivePoseEstimator3d(
            SWERVE_KINEMATICS,
            imuInputs.imuOrientation,
            swerveModulePositions,
            new Pose3d()
        );

        // Configure PathPlanner to use this subsystem.
        AutoBuilder.configure(
            this::getPose2d, // Get the robot's current position.
            this::resetPose2d, // Reset the robot's position.
            this::getSpeeds, // Get the robot's current speeds.
            (speeds, feedforwards) -> run(speeds), // Set the robot's speeds.
            new PPHolonomicDriveController(
                // PID control for translation and rotation.
                new PIDConstants(5.0, 0.0, 0.0),
                new PIDConstants(5.0, 0.0, 0.0)
            ),
            AutoConstants.ROBOT_CONFIG, // The robot's config from PathPlanner settings.
            () -> {
                var alliance = DriverStation.getAlliance(); // Get the alliance.
                if (alliance.isPresent()) {
                    // If the alliance is red, return true to flip the path.
                    // If the alliance is not red, return false to keep the path the same.
                    return alliance.get() == Alliance.Red;
                }
                // If the alliance is invalid, don't flip the path to be safe.
                return false;
            },
            // Use this subsystem as part of the requirements for the auto commands.
            this
        );

        // Reset the rotation because it starts off angled for some reason.
        estimatedPose.resetRotation(new Rotation3d());

        fieldWidget = new Field2d();
    }

    /**
     * Get the active DriveSubsystem instance.
     *
     * @return The active instance.
     */
    public static DriveSubsystem getInstance() {
        if (instance == null) {
            // If the instance hasn't been created yet, create it.
            instance = new DriveSubsystem();
        }

        return instance; // Return the instance.
    }

    /**
     * Set the target states for the drive base swerve modules.
     *
     * @param targetSpeeds The target ChassisSpeeds.
     */
    public void run(ChassisSpeeds targetSpeeds) {
        Logger.recordOutput("Drive/TargetSpeeds", targetSpeeds); // Log the raw target speeds.

        // Perform 2nd-order kinematics on the given speeds to reduce drift.
        targetSpeeds = ChassisSpeeds.discretize(
            targetSpeeds,
            KinematicsConstants.DISCRETIZE_SECONDS
        );
        Logger.recordOutput("Drive/CorrectedSpeeds", targetSpeeds); // Log the corrected target speeds.

        // Convert the target ChassisSpeeds to individual SwerveModuleStates.
        SwerveModuleState[] targetStates =
            SWERVE_KINEMATICS.toSwerveModuleStates(targetSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(
            targetStates,
            KinematicsConstants.MAX_MODULE_SPEED
        );

        for (int i = 0; i < 4; i++) { // For each swerve module:
            // Log the given state for the module.
            Logger.recordOutput(
                "Drive/" + SwerveModuleKeys.fromInt(i) + "Module/TargetState",
                targetStates[i]
            );
            SwerveModuleState optimizedState =
                swerveModules[i].setModuleState(targetStates[i]); // Send the state to the
            // module.
            // Log the optimized state.
            Logger.recordOutput(
                "Drive/" +
                SwerveModuleKeys.fromInt(i) +
                "Module/OptimziedState",
                optimizedState
            );
        }
    }

    /**
     * Get the robot's current heading.
     *
     * @return The heading as a Rotation2d.
     */
    public Rotation2d getHeading() {
        return imuInputs.imuHeading;
    }

    public Rotation2d getRawHeading() {
        return Rotation2d.fromDegrees(imuInputs.imuRawYaw);
    }

    public Command resetPose2dCommand(Pose2d newPose) {
        return this.runOnce(() -> {
                estimatedPose.resetPose(new Pose3d(newPose));
            });
    }

    public Field2d getFieldWidget() {
        return fieldWidget;
    }

    /**
     * Get the robot's current heading from the drive base and PhotonVision.
     *
     * @return The fused heading as a Rotation2d.
     */
    public Rotation2d getFusedHeading() {
        return estimatedPose
            .getEstimatedPosition()
            .getRotation()
            .toRotation2d();
    }

    /**
     * Get the estimated position of the robot.
     *
     * @return The position as a Pose2d.
     */
    public Pose2d getPose2d() {
        return estimatedPose.getEstimatedPosition().toPose2d();
    }

    public Pose3d getPose3d() {
        return estimatedPose.getEstimatedPosition();
    }

    public double getXAccel() {
        return imuInputs.imuRawXAccel;
    }

    /**
     * Reset the estimated position of the robot.
     *
     * @param pose The position of the robot as a Pose2d.
     */
    public void resetPose2d(Pose2d pose) {
        var allianceOptional = DriverStation.getAlliance();

        Rotation2d rotation = pose.getRotation();

        if (allianceOptional.isPresent()) {
            if (allianceOptional.get() == Alliance.Blue) {
                rotation = rotation.rotateBy(Rotation2d.kPi);
            }
        }

        estimatedPose.resetPose(
            new Pose3d(
                pose.getX(),
                pose.getY(),
                0,
                new Rotation3d(pose.getRotation())
            )
        );

        imu.setHeading(rotation.getDegrees());
    }

    /**
     * Get the current speeds of the robot.
     *
     * @return The speeds of the robot as a ChassisSpeeds.
     */
    public ChassisSpeeds getSpeeds() {
        // Use the swerve kinematics object to perform forward kinematics to convert the
        // module states to a ChassisSpeed.
        return SWERVE_KINEMATICS.toChassisSpeeds(swerveModuleStates);
    }

    /**
     * Builds a command to update the robot's current heading.
     *
     * @param heading The new heading for the IMU.
     * @return The Command to schedule.
     */
    public Command setHeading(double heading) {
        return this.runOnce(() -> {
                imu.setHeading(heading);
            });
    }

    /**
     * Builds a command to reset the robot's current heading to zero.
     *
     * @return The Command to schedule.
     */
    public Command resetHeading() {
        return this.runOnce(() -> {
                imu.resetHeading();
                estimatedPose.resetRotation(new Rotation3d());
            });
    }

    public Command resetHeading(double angle) {
        return this.runOnce(() -> {
                imu.setHeading(angle);
                estimatedPose.resetRotation(new Rotation3d(0, 0, angle));
            });
    }

    public void submitVisionMeasurements(
        List<EstimatedRobotPose> visionMeasurements
    ) {
        for (EstimatedRobotPose measurement : visionMeasurements) {
            estimatedPose.addVisionMeasurement(
                measurement.estimatedPose,
                measurement.timestampSeconds
            );
        }
    }

    public Command resetPoseEstimationToVision() {
        return this.runOnce(() -> {
                var poseOptional = VisionSubsystem.getInstance()
                    .getMeanEstimatedPose();

                if (poseOptional.isPresent()) {
                    estimatedPose.resetPose(poseOptional.get());
                }
            });
    }

    @Override
    public void periodic() {
        for (int i = 0; i < 4; i++) { // For each swerve module:
            swerveModules[i].updateInputs(swerveModuleInputs[i]); // Update the inputs for the module.
            // Process the inputs for the module.
            Logger.processInputs(
                "Drive/" + SwerveModuleKeys.fromInt(i) + "Module",
                swerveModuleInputs[i]
            );

            swerveModulePositions[i] = swerveModuleInputs[i].modulePosition; // Update the module's position.
            swerveModuleStates[i] = swerveModuleInputs[i].currentState; // Update the module's state.
        }

        imu.updateInputs(imuInputs); // Update the IMU inputs.
        Logger.processInputs("Drive/IMU", imuInputs); // Process the IMU inputs.

        // Update the pose estimator with the updated inputs.
        //estimatedPose.update(imuInputs.imuOrientation, swerveModulePositions);
        estimatedPose.updateWithTime(
            Logger.getTimestamp(),
            imuInputs.imuOrientation,
            swerveModulePositions
        );

        // Record the updated pose to the log.
        Logger.recordOutput(
            "Drive/Position",
            estimatedPose.getEstimatedPosition()
        );

        fieldWidget.setRobotPose(getPose2d());

        SmartDashboard.putData("Field", fieldWidget);
    }

    @Override
    public void simulationPeriodic() {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].updateSimDevices(); // Update module simulations.
            swerveModules[i].updateInputs(swerveModuleInputs[i]); // Process simulated inputs.

            // Process simulated inputs.
            Logger.processInputs(
                "Drive/IMU" + SwerveModuleKeys.fromInt(i) + "Module",
                swerveModuleInputs[i]
            );

            swerveModulePositions[i] = swerveModuleInputs[i].modulePosition;
            swerveModuleStates[i] = swerveModuleInputs[i].currentState;
        }
    }
}
