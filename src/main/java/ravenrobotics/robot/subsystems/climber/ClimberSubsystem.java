package ravenrobotics.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.robot.Configs;
import ravenrobotics.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

    private final SparkFlex climberMotor = new SparkFlex(
        ClimberConstants.CLIMBER,
        MotorType.kBrushless
    );

    private final RelativeEncoder climberEncoder = climberMotor.getEncoder();

    private final SparkClosedLoopController climberController =
        climberMotor.getClosedLoopController();

    private boolean isHolding = false;

    private static ClimberSubsystem instance;

    /**
     * Gets the active ClimberSubsystem.
     *
     * @return The ClimberSubsystem instance.
     */
    public static ClimberSubsystem getInstance() {
        // If the instance has not been created yet, create it.
        if (instance == null) {
            instance = new ClimberSubsystem();
        }

        return instance;
    }

    private ClimberSubsystem() {
        climberMotor.configure(
            Configs.climberConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    /**
     * Sets the motor power if the climber is not in hold mode.
     *
     * @param power The power level to set the motor to
     */
    private void _setPower(double power) {
        if (!isHolding) {
            climberMotor.set(power);
        }
    }

    /**
     * Sets the climber to maintain its current position using closed loop control.
     */
    private void _setHold() {
        climberController.setReference(
            climberEncoder.getPosition(),
            ControlType.kPosition
        );
        isHolding = true;
    }

    /**
     * Disables position hold mode.
     */
    private void _cancelHold() {
        isHolding = false;
    }

    /**
     * Sets the power level for the climber motor.
     *
     * @param power The power level to set
     * @return The command to schedule.
     */
    public Command setPower(double power) {
        return this.run(() -> this._setPower(power)).finallyDo(() ->
                this._setPower(0)
            );
    }

    /**
     * Sets the climber to hold its current position.
     *
     * @return The command to schedule.
     */
    public Command setHold() {
        return this.runOnce(() -> this._setHold());
    }

    /**
     * Cancels the hold position mode of the climber.
     *
     * @return The command to schedule.
     */
    public Command cancelHold() {
        return this.runOnce(() -> this._cancelHold());
    }
}
