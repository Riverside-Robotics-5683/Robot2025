package ravenrobotics.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

@AutoLog
public class SwerveModuleInputs {
  public double drivePositionMeters = 0.0;
  public double driveVelocityMetersPerSecond = 0.0;
  public double driveVoltage = 0.0;
  public double driveAmps = 0.0;

  public Rotation2d anglePosition = new Rotation2d();
  public double angleVelocityMetersPerSecond = 0.0;
  public double angleVoltage = 0.0;
  public double angleAmps = 0.0;

  public SwerveModuleState targetState = new SwerveModuleState();
  public SwerveModuleState currentState = new SwerveModuleState();
  public SwerveModulePosition modulePosition = new SwerveModulePosition();
}
