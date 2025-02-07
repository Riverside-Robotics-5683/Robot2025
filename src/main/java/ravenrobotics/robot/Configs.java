package ravenrobotics.robot;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import ravenrobotics.robot.Constants.ElevatorConstants;
import ravenrobotics.robot.Constants.KinematicsConstants;
import ravenrobotics.robot.Constants.SwerveModuleConstants;

public class Configs {

    // Configs for swerve motors.
    public static SparkFlexConfig swerveDriveConfig = new SparkFlexConfig();
    public static SparkMaxConfig swerveAngleConfig = new SparkMaxConfig();

    public static SparkFlexConfig elevatorConfig = new SparkFlexConfig();

    public static SparkFlexConfig flipperConfig = new SparkFlexConfig();
    public static SparkFlexConfig sliderConfig = new SparkFlexConfig();
    public static SparkFlexConfig rollerConfig = new SparkFlexConfig();

    // Config for the IMU.
    public static Pigeon2Configuration imuConfig = new Pigeon2Configuration();

    static {
        swerveDriveConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(SwerveModuleConstants.DRIVE_LIMIT)
            .closedLoopRampRate(1.5);
        swerveAngleConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(SwerveModuleConstants.ANGLE_LIMIT);

        swerveDriveConfig.encoder
            .positionConversionFactor(
                KinematicsConstants.DRIVE_CONVERSION_FACTOR
            )
            .velocityConversionFactor(
                KinematicsConstants.DRIVE_CONVERSION_FACTOR / 60.0
            );
        swerveAngleConfig.absoluteEncoder
            .positionConversionFactor(
                KinematicsConstants.ANGLE_CONVERSION_FACTOR
            )
            .velocityConversionFactor(
                KinematicsConstants.ANGLE_CONVERSION_FACTOR / 60.0
            )
            .inverted(true);

        swerveDriveConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.05, 0.001, 0)
            .velocityFF(1 / KinematicsConstants.DRIVE_FREE_WHEEL_SPEED)
            .outputRange(-1, 1);
        swerveAngleConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(1, 0, 0)
            .outputRange(-1, 1)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(
                0,
                KinematicsConstants.ANGLE_CONVERSION_FACTOR
            );

        imuConfig.Pigeon2Features.withEnableCompass(false)
            .withDisableNoMotionCalibration(false)
            .withDisableTemperatureCompensation(false);

        elevatorConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(ElevatorConstants.ELEAVTOR_LIMIT);
        elevatorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(1.0, 0.0, 0.0)
            .outputRange(-1, 1);

        flipperConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(20)
            .closedLoopRampRate(1.0);
        flipperConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(1.0, 0.0, 0.0)
            .outputRange(-1, 1);

        sliderConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(15)
            .closedLoopRampRate(1.0);
        sliderConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(1.0, 0.0, 0.0)
            .outputRange(-1, 1);

        rollerConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(15);
    }
}
