package ravenrobotics.robot.subsystems.drive;

import static ravenrobotics.robot.Constants.KinematicsConstants.SWERVE_KINEMATICS;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.robot.Constants.KinematicsConstants;
import ravenrobotics.robot.Constants.SwerveModuleKeys;

/**
 * Subsystem for controlling the drive base.
 */
public class DriveSubsystem extends SubsystemBase {
  private final SwerveModule[] swerveModules = new SwerveModule[4]; // The swerve modules for controlling motors.
  // Swerve module inputs for AdvantageKit.
  private SwerveModuleInputsAutoLogged[] swerveModuleInputs = new SwerveModuleInputsAutoLogged[4];

  private final IMU imu = new IMU(); // The IMU for getting heading and orientation.
  private IMUInputsAutoLogged imuInputs = new IMUInputsAutoLogged(); // The IMU inputs for AdvantageKit.

  private SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4]; // The swerve module positions.
  private SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4]; // The swerve module states (speed).

  // Pose estimator combining drive odometry and vision measurements for more
  // accurate odometry.
  private SwerveDrivePoseEstimator3d estimatedPose = new SwerveDrivePoseEstimator3d(
      KinematicsConstants.SWERVE_KINEMATICS, imuInputs.imuOrientation, swerveModulePositions, new Pose3d());

  private static DriveSubsystem instance; // The static instance for the getInstance method.

  private DriveSubsystem() {
    for (int i = 0; i < 4; i++) {
      // Create all of the swerve modules.
      swerveModules[i] = new SwerveModule(i);
    }
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
    targetSpeeds = ChassisSpeeds.discretize(targetSpeeds, KinematicsConstants.DISCRETIZE_SECONDS);
    Logger.recordOutput("Drive/CorrectedSpeeds", targetSpeeds); // Log the corrected target speeds.

    // Convert the target ChassisSpeeds to individual SwerveModuleStates.
    SwerveModuleState[] targetStates = SWERVE_KINEMATICS.toSwerveModuleStates(targetSpeeds);

    for (int i = 0; i < 4; i++) { // For each swerve module:
      // Log the given state for the module.
      Logger.recordOutput("Drive/" + SwerveModuleKeys.fromInt(i) + "Module/TargetState", targetStates[i]);
      SwerveModuleState optimizedState = swerveModules[i].setModuleState(targetStates[i]); // Send the state to the
                                                                                           // module.
      // Log the optimized state.
      Logger.recordOutput("Drive/" + SwerveModuleKeys.fromInt(i) + "Module/OptimziedState", optimizedState);
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
    });
  }

  @Override
  public void periodic() {
    for (int i = 0; i < 4; i++) { // For each swerve module:
      swerveModules[i].updateInputs(swerveModuleInputs[i]); // Update the inputs for the module.
      // Process the inputs for the module.
      Logger.processInputs("Drive/" + SwerveModuleKeys.fromInt(i) + "Module", swerveModuleInputs[i]);

      swerveModulePositions[i] = swerveModuleInputs[i].modulePosition; // Update the module's position.
      swerveModuleStates[i] = swerveModuleInputs[i].currentState; // Update the module's state.
    }

    imu.updateInputs(imuInputs); // Update the IMU inputs.
    Logger.processInputs("Drive/IMU", imuInputs); // Process the IMU inputs.

    // Update the pose estimator with the updated inputs.
    estimatedPose.update(imuInputs.imuOrientation, swerveModulePositions);
  }

  @Override
  public void simulationPeriodic() {
    for (int i = 0; i < 4; i++) {
      swerveModules[i].updateSimDevices(); // Update module simulations.
      swerveModules[i].updateInputs(swerveModuleInputs[i]); // Process simulated inputs.
      
      //Process simulated inputs.
      Logger.processInputs("Drive/IMU" + SwerveModuleKeys.fromInt(i) + "Module", swerveModuleInputs[i]);
      
      swerveModulePositions[i] = swerveModuleInputs[i].modulePosition;
      swerveModuleStates[i] = swerveModuleInputs[i].currentState;
    }
  }
}
