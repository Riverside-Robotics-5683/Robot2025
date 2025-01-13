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

public class DriveSubsystem extends SubsystemBase {
  private final SwerveModule[] swerveModules = new SwerveModule[4];
  private SwerveModuleInputsAutoLogged[] swerveInputs = new SwerveModuleInputsAutoLogged[4];

  private final IMU imu = new IMU();
  private IMUInputsAutoLogged imuInputs = new IMUInputsAutoLogged();

  private SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
  private SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];

  private SwerveDrivePoseEstimator3d estimatedPose = new SwerveDrivePoseEstimator3d(
      KinematicsConstants.SWERVE_KINEMATICS, imuInputs.imuOrientation, swerveModulePositions, new Pose3d());

  private static DriveSubsystem instance;

  public static DriveSubsystem getInstance() {
    if (instance == null) {
      instance = new DriveSubsystem();
    }

    return instance;
  }

  private DriveSubsystem() {
    for (int i = 0; i < 4; i++) {
      swerveModules[i] = new SwerveModule(i);
    }
  }

  public void run(ChassisSpeeds targetSpeeds) {
    Logger.recordOutput("Drive/TargetSpeeds", targetSpeeds);
    targetSpeeds = ChassisSpeeds.discretize(targetSpeeds, KinematicsConstants.DISCRETIZE_SECONDS); 
    SwerveModuleState[] targetStates = SWERVE_KINEMATICS.toSwerveModuleStates(targetSpeeds);
    Logger.recordOutput("Drive/DiscretizedSpeeds", targetSpeeds);

    for (int i = 0; i < 4; i++) {
      Logger.recordOutput("Drive/" + SwerveModuleKeys.fromInt(i) + "Module/TargetState", targetStates[i]);
      SwerveModuleState optimizedState = swerveModules[i].setModuleState(targetStates[i]);
      Logger.recordOutput("Drive/" + SwerveModuleKeys.fromInt(i) + "Module/OptimziedState", optimizedState);
    }
  }

  public Rotation2d getHeading() {
    return imuInputs.imuHeading;
  }

  public Command setHeading(double heading) {
    return this.runOnce(() -> {
      imu.setHeading(heading);
    });
  }

  public Command resetHeading() {
    return this.runOnce(() -> {
      imu.resetHeading();
    });
  }

  @Override
  public void periodic() {
    for (int i = 0; i < 4; i++) {
      swerveModules[i].updateInputs(swerveInputs[i]);
      Logger.processInputs("Drive/" + SwerveModuleKeys.fromInt(i) + "Module", swerveInputs[i]);
    }

    imu.updateInputs(imuInputs);
    Logger.processInputs("Drive/IMU", imuInputs);
  }
}
