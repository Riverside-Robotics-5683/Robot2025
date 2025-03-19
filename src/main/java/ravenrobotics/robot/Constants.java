package ravenrobotics.robot;

import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import java.io.IOException;
import ravenrobotics.robot.util.DriverProfile;

/**
 * Top level class for storing robot constants.
 */
public class Constants {

    public static class DSConstants {

        public static final int DRIVE_CONTROLLER = 0; // Drive controller port.
        public static final int SYSTEMS_CONTROLLER = 1; // Systems controller port.

        public static final DriverProfile DEFAULT_PROFILE = new DriverProfile(
            "Default", // Profile name
            0.05, // X-axis deadband.
            0.05, // Y-axis deadband.
            0.05, // Z-axis deadband.
            75, // X-axis acceleration.
            75, // Y-axis acceleration.
            75
        ); // Z-axis acceleration.
    }

    /**
     * Constants for swerve modules.
     */
    public static class SwerveModuleConstants {

        // Front Left CAN IDs.
        public static final int FL_DRIVE = 1; // Drive motor.
        public static final int FL_ANGLE = 2; // Angle motor.

        // Front Right CAN IDs.
        public static final int FR_DRIVE = 3; // Drive motor.
        public static final int FR_ANGLE = 4; // Angle motor.

        // Rear Left CAN IDs.
        public static final int RL_DRIVE = 5; // Drive motor.
        public static final int RL_ANGLE = 6; // Angle motor.

        // Rear Right CAN IDs.
        public static final int RR_DRIVE = 7; // Drive motor.
        public static final int RR_ANGLE = 8; // Angle motor.

        public static final int DRIVE_LIMIT = 40; // Drive motor current limit.
        public static final int ANGLE_LIMIT = 20; // Angle motor current limit.

        public static final double FL_OFFSET = -Math.PI / 2; // Front Left offset.
        public static final double FR_OFFSET = 0; // Front Right offset.
        public static final double RL_OFFSET = Math.PI; // Rear Left offset.
        public static final double RR_OFFSET = Math.PI / 2; // Rear Right offset.
    }

    /**
     * Shorthand keys for the different swerve modules.
     */
    public enum SwerveModuleKeys {
        /**
         * The front left module.
         */
        FL,
        /**
         * The front right module.
         */
        FR,
        /**
         * The rear left module.
         */
        RL,
        /**
         * The rear right module.
         */
        RR;

        /**
         * Convert an integer in the range 0-3 to a SwerveModuleKey.
         *
         * @param id The int to convert.
         * @return The corresponding SwerveModuleKey.
         */
        public static SwerveModuleKeys fromInt(int id) {
            switch (id) {
                case 0:
                    return FL;
                case 1:
                    return FR;
                case 2:
                    return RL;
                case 3:
                    return RR;
                default:
                    throw new IllegalArgumentException(
                        "ID must be in range 0-3."
                    );
            }
        }
    }

    /**
     * Constants for the IMU.
     */
    public static class IMUConstants {

        // The IMU's CAN ID.
        public static final int IMU = 1;
    }

    /**
     * Consants for kinematics operations.
     */
    public static class KinematicsConstants {

        public static final double WHEEL_DIAMETER = Units.inchesToMeters(3); // Wheel diameter.
        public static final double WHEEL_CIRCUMFERENCE =
            WHEEL_DIAMETER * Math.PI; // Wheel circumference.

        public static final double DRIVE_GEAR_RATIO =
            (45.0 * 22.0) / (14.0 * 15.0); // Gear ratio between the drive motor
        // and the wheel.

        // The ratio between the drive motor and the wheel.
        public static final double DRIVE_CONVERSION_FACTOR =
            WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO;

        public static final double ANGLE_CONVERSION_FACTOR = Math.PI * 2; // Angle motor conversion factor.

        // The motor's free speed in rotations per second.
        public static final double DRIVE_FREE_MOTOR_SPEED = 6784.0 / 60.0;

        // Drive wheel free speed (feedforwards).
        public static final double DRIVE_FREE_WHEEL_SPEED =
            (DRIVE_FREE_MOTOR_SPEED * WHEEL_CIRCUMFERENCE) / DRIVE_GEAR_RATIO;

        // Robot length and width.
        public static final double TRACK_WIDTH = Units.inchesToMeters(27);
        public static final double WHEEL_BASE = Units.inchesToMeters(27);

        // Swerve drive kinematics for converting between ChassisSpeeds and
        // SwerveModuleStates.
        public static final SwerveDriveKinematics SWERVE_KINEMATICS =
            new SwerveDriveKinematics(
                new Translation2d(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0), // FL
                new Translation2d(TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0), // FR
                new Translation2d(-TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0), // RL
                new Translation2d(-TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0)
            ); // RR

        public static final double DISCRETIZE_SECONDS = 0.02; // 2nd order kinematics

        public static final double MAX_MODULE_SPEED = Units.feetToMeters(16); // Max module speed
        public static final double ELEVATOR_MODULE_SPEED = Units.feetToMeters(
            8
        );
    }

    /**
     * Constants for the elevator.
     */
    public static class ElevatorConstants {

        // The CAN ID of the left motor driving the elevator.
        public static final int ELEVATOR_LEFT = 9;
        // The CAN ID of the right motor driving the elevator.
        public static final int ELEVATOR_RIGHT = 10;

        // The current limit for both elevator motors.
        public static final int ELEAVTOR_LIMIT = 40;
        public static final double SPEED_LIMIT = 0;

        public static final double ELEVATOR_L4 = 89.99;
        public static final double ELEVATOR_L3 = 60;
        public static final double ELEVATOR_L2 = 35.5;
        public static final double ELEVATOR_L1 = 30.208;
        public static final double ELEVATOR_FEED = 1.91;
    }

    /**
     * Constants for the intake.
     */
    public static class IntakeConstants {

        // The CAN ID of the motor for intaking/outaking game pieces.
        public static final int ROLLERS = 11;
    }

    public static class ClimberConstants {

        public static final int CLIMBER = 13;
    }

    /**
     * Constants for autonomous.
     */
    public static class AutoConstants {

        // The physical robot config for calculating autonomous paths.
        public static RobotConfig ROBOT_CONFIG;

        static {
            try {
                // Try to load the robot config from the settings from PathPlanner.
                ROBOT_CONFIG = RobotConfig.fromGUISettings();
                //Generic error handling for now, probably won't change :).
            } catch (org.json.simple.parser.ParseException e) {
                e.printStackTrace();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    public static class VisionConstants {

        public static double[] DEFAULT_CAMERA_MATRIX = {
            650.0,
            0.0,
            320.0,
            0.0,
            650.0,
            240.0,
            0.0,
            0.0,
            1.0,
        };

        public static double[] DEFAULT_DISTORTION_MATRIX = {
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        };
    }
}
