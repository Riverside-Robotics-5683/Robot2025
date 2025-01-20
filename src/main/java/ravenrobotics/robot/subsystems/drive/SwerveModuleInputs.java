package ravenrobotics.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Class for storing inputs from swerve modules.
 */
@AutoLog
public class SwerveModuleInputs {
  /**
   * The drive motor position in meters.
   */
  public double drivePositionMeters = 0.0;
  /**
   * The drive motor velocity in m/s.
   */
  public double driveVelocityMetersPerSecond = 0.0;
  /**
   * The drive motor voltage.
   */
  public double driveVoltage = 0.0;
  /**
   * The drive motor current draw in amps.
   */
  public double driveAmps = 0.0;

  /**
   * The angle motor position as a Rotation2d.
   */
  public Rotation2d anglePosition = new Rotation2d();
  /**
   * The angle motor velocity
   */
  public double angleVelocityMetersPerSecond = 0.0;
  /**
   * The angle motor voltage.
   */
  public double angleVoltage = 0.0;
  /**
   * The angle motor current draw in amps.
   */
  public double angleAmps = 0.0;

  /**
   * The target module state.
   */
  public SwerveModuleState targetState = new SwerveModuleState();
  /**
   * The current module state.
   */
  public SwerveModuleState currentState = new SwerveModuleState();
  /**
   * The current module position.
   */
  public SwerveModulePosition modulePosition = new SwerveModulePosition();
}
