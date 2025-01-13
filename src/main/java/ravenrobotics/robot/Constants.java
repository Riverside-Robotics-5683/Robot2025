package ravenrobotics.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import ravenrobotics.robot.util.DriverProfile;

public class Constants {
  public static class DSConstants {
    public static final int DRIVE_CONTROLLER = 0;

    public static final DriverProfile DEFAULT_PROFILE = new DriverProfile(
        "Default",
        0.04,
        0.04,
        0.04,
        1.5,
        1.5,
        1.0);
  }

  public static class SwerveModuleConstants {
    public static final int FL_DRIVE = 0;
    public static final int FL_ANGLE = 1;

    public static final int FR_DRIVE = 2;
    public static final int FR_ANGLE = 3;

    public static final int RL_DRIVE = 4;
    public static final int RL_ANGLE = 5;

    public static final int RR_DRIVE = 6;
    public static final int RR_ANGLE = 7;

    public static final int DRIVE_LIMIT = 40;
    public static final int ANGLE_LIMIT = 30;

    public static final double FL_OFFSET = -Math.PI / 2;
    public static final double FR_OFFSET = 0;
    public static final double RL_OFFSET = Math.PI;
    public static final double RR_OFFSET = Math.PI / 2;
  }

  public enum SwerveModuleKeys {
    FL,
    FR,
    RL,
    RR;

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
          throw new IllegalArgumentException("ID must be in range 0-3.");
      }
    }
  }

  public static class IMUConstants {
    public static final int IMU = 8;
  }

  public static class KinematicsConstants {
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(3);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final double DRIVE_CONVERSION_FACTOR = (45.0 * 22) / (14 * 15);
    public static final double ANGLE_CONVERSION_FACTOR = Math.PI * 2;

    public static final double DRIVE_FREE_WHEEL_SPEED = (6784 * WHEEL_CIRCUMFERENCE) / DRIVE_CONVERSION_FACTOR;

    public static final double TRACK_WIDTH = Units.inchesToMeters(25);
    public static final double WHEEL_BASE = Units.inchesToMeters(25);

    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0),
        new Translation2d(TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0),
        new Translation2d(-TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0),
        new Translation2d(-TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0));

    public static final double DISCRETIZE_SECONDS = 0.02;

    public static final double MAX_MODULE_SPEED = Units.feetToMeters(20);
  }
}
