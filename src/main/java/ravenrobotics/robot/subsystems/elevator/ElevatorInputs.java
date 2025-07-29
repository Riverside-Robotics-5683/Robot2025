package ravenrobotics.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/**
 * Class for storing elevator inputs for logging.
 */
@AutoLog
public class ElevatorInputs {
    public double leftMotorPosition = 0.0;
    public double leftMotorVelocity = 0.0;

    public double leftMotorVoltage = 0.0;
    public double leftMotorCurrent = 0.0;

    public double rightMotorPosition = 0.0;
    public double rightMotorVelocity = 0.0;

    public double rightMotorVoltage = 0.0;
    public double rightMotorCurrent = 0.0;

    public double targetPosition = 0.0;
}
