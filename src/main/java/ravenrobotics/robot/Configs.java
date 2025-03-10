package ravenrobotics.robot;

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
    public static SparkMaxConfig rollerConfig = new SparkMaxConfig();

    public static SparkFlexConfig climberConfig = new SparkFlexConfig();

    static {
        swerveDriveConfig
            .idleMode(IdleMode.kBrake)
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
            .pid(0.005, 0, 0)
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

        elevatorConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ElevatorConstants.ELEAVTOR_LIMIT)
            .closedLoopRampRate(0.5);
        elevatorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(1.0, 0, 0)
            .outputRange(-0.25, 1);
        elevatorConfig.encoder.velocityConversionFactor(1.0 / 60.0);

        elevatorConfig.closedLoop.maxMotion
            .maxVelocity(3200)
            .maxAcceleration(6000);

        flipperConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        flipperConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.5, 0, 0.2)
            .outputRange(-1, 1);

        rollerConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(15);

        rollerConfig.encoder
            .positionConversionFactor(44.0 / 36.0)
            .velocityConversionFactor(44.0 / 36.0);

        climberConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(30)
            .closedLoopRampRate(1.0);
        climberConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(1.0, 0.0, 0.0)
            .outputRange(-1, 1);
    }
}
