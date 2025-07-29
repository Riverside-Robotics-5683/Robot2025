package ravenrobotics.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

/**
 * Class for storing inputs from the IMU.
 */
@AutoLog
public class IMUInputs {
  /**
   * The IMU's current heading.
   */
  public Rotation2d imuHeading = new Rotation2d();
  /**
   * The IMU's current orientation.
   */
  public Rotation3d imuOrientation = new Rotation3d();

  /**
   * The IMU's measured roll.
   */
  public double imuRawRoll = 0.0;
  /**
   * The IMU's measured pitch.
   */
  public double imuRawPitch = 0.0;
  /**
   * The IMU's measured yaw.
   */
  public double imuRawYaw = 0.0;

  /**
   * The IMU's measured x-axis acceleration
   */
  public double imuRawXAccel = 0.0;
  /**
   * The IMU's measured y-axis acceleration.
   */
  public double imuRawYAccel = 0.0;
  /**
   * The IMU's measured z-axis acceleration.
   */
  public double imuRawZAccel = 0.0;
}
