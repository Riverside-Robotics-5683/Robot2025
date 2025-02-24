package ravenrobotics.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class IntakeInputs {

    public double flipperTargetPosition = 0.0;

    public double topFlipperPosition = 0.0;
    public double topFlipperVelocity = 0.0;

    public double topFlipperVoltage = 0.0;
    public double topFlipperCurrent = 0.0;

    public double bottomFlipperPosition = 0.0;
    public double bottomFlipperVelocity = 0.0;

    public double bottomFlipperVoltage = 0.0;
    public double bottomFlipperCurrent = 0.0;
}
