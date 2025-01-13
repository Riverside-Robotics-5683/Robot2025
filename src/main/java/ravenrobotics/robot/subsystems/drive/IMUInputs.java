package ravenrobotics.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

@AutoLog
public class IMUInputs {
  public Rotation2d imuHeading = new Rotation2d();
  public Rotation3d imuOrientation = new Rotation3d();

  public double imuRawRoll = 0.0;
  public double imuRawPitch = 0.0;
  public double imuRawYaw = 0.0;

  public double imuRawXAccel = 0.0;
  public double imuRawYAccel = 0.0;
  public double imuRawZAccel = 0.0;
}
