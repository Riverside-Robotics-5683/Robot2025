package ravenrobotics.robot;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import ravenrobotics.robot.Constants.KinematicsConstants;
import ravenrobotics.robot.Constants.SwerveModuleConstants;

public class Configs {
  // Configs for swerve motors.
  public static SparkFlexConfig swerveDriveConfig = new SparkFlexConfig();
  public static SparkMaxConfig swerveAngleConfig = new SparkMaxConfig();

  // Config for the IMU.
  public static Pigeon2Configuration imuConfig = new Pigeon2Configuration();

  static {
    swerveDriveConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(SwerveModuleConstants.DRIVE_LIMIT);
    swerveAngleConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(SwerveModuleConstants.ANGLE_LIMIT);

    swerveDriveConfig.encoder
        .positionConversionFactor(KinematicsConstants.DRIVE_CONVERSION_FACTOR)
        .velocityConversionFactor(KinematicsConstants.DRIVE_CONVERSION_FACTOR / 60);
    swerveAngleConfig.encoder
        .positionConversionFactor(KinematicsConstants.ANGLE_CONVERSION_FACTOR)
        .velocityConversionFactor(KinematicsConstants.ANGLE_CONVERSION_FACTOR / 60);

    swerveDriveConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0, 0, 0)
        .velocityFF(1.0 / KinematicsConstants.DRIVE_FREE_WHEEL_SPEED)
        .outputRange(-1, 1);
    swerveAngleConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(5, 0, 0)
        .outputRange(-1, 1)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, KinematicsConstants.ANGLE_CONVERSION_FACTOR);

    imuConfig.Pigeon2Features.withEnableCompass(false).withDisableNoMotionCalibration(false)
        .withDisableTemperatureCompensation(false);
  }
}
