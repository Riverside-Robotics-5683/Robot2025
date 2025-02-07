package ravenrobotics.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    /** Motor controlling the flipper mechanism for intake angle adjustment */
    private final SparkFlex flipperMotor = new SparkFlex(
        IntakeConstants.FLIPPER,
        SparkFlex.MotorType.kBrushless
    );
    /** Motor controlling the slider mechanism for intake extension/retraction */
    private final SparkFlex sliderMotor = new SparkFlex(
        IntakeConstants.SLIDER,
        SparkFlex.MotorType.kBrushless
    );
    /** Motor controlling the rollers for intake/outtake */
    private final SparkFlex rollerMotor = new SparkFlex(
        IntakeConstants.ROLLERS,
        SparkFlex.MotorType.kBrushless
    );

    /** Closed loop controller for the flipper motor */
    private final SparkClosedLoopController flipperController =
        flipperMotor.getClosedLoopController();

    /** Closed loop controller for the slider motor */
    private final SparkClosedLoopController sliderController =
        sliderMotor.getClosedLoopController();

    /** Closed loop controller for the roller motor */
    private final SparkClosedLoopController rollerController =
        rollerMotor.getClosedLoopController();

    private static IntakeSubsystem instance;

    /**
     * Gets the active IntakeSubsystem
     *
     * @return The IntakeSubsystem instance
     */
    public static IntakeSubsystem getInstance() {
        // If no instance exists, create a new one.
        if (instance == null) {
            instance = new IntakeSubsystem();
        }

        return instance;
    }

    /**
     * The different possible slider positiions.
     */
    public enum IntakeSliderPosition {
        /**
         * Fully out.
         */
        OPEN,
        /**
         * Open enough to clear the elevator.
         */
        HALF_OPEN,
        /**
         * Fully retracted.
         */
        CLOSED,
    }

    /**
     * The different angles for the intake.
     */
    public enum IntakeAngle {
        /**
         * The angle for L1.
         */
        L1,
        /**
         * The angle for L2.
         */
        L2,
        /**
         * The angle for L3.
         */
        L3,
        /**
         * The angle for L4.
         */
        L4,
        /**
         * Default (flat) angle.
         */
        DEFAULT,
    }

    /**
     * Sets the slider position.
     *
     * @param position The desired position setpoint
     */
    private void setSliderPosition(double position) {
        sliderController.setReference(position, ControlType.kPosition);
    }

    /**
     * Sets the intake angle.
     *
     * @param position The desired angle setpoint
     */
    private void setIntakeAngle(double position) {
        flipperController.setReference(position, ControlType.kPosition);
    }

    /**
     * Sets the power of the intake rollers.
     *
     * @param power The power level.
     */
    private void setRollerPower(double power) {
        rollerMotor.set(power);
    }

    /**
     * Sets the position of the intake slider.
     *
     * @param position The desired slider position.
     * @return The Command to be scheduled.
     */
    public Command setSliderPosition(IntakeSliderPosition position) {
        switch (position) {
            // For OPEN position, set to the open slider constant
            case OPEN:
                return this.runOnce(() ->
                        setSliderPosition(IntakeConstants.INTAKE_SLIDER_OPEN)
                    );
            // For HALF_OPEN position, set to the half open slider constant
            case HALF_OPEN:
                return this.runOnce(() ->
                        setSliderPosition(
                            IntakeConstants.INTAKE_SLIDER_HALF_OPEN
                        )
                    );
            // For CLOSED position, set to the closed slider constant
            case CLOSED:
                return this.runOnce(() ->
                        setSliderPosition(IntakeConstants.INTAKE_SLIDER_CLOSED)
                    );
            // If no valid position specified, default to closed position
            default:
                return this.runOnce(() ->
                        setSliderPosition(IntakeConstants.INTAKE_SLIDER_CLOSED)
                    );
        }
    }

    /**
     * Sets the angle of the intake.
     *
     * @param angle The desired angle position.
     * @return The Command to be scheduled.
     */
    public Command setSliderPosition(IntakeAngle angle) {
        // Switch based on the provided angle enum to set the appropriate intake angle
        switch (angle) {
            // For L1 position, set to the L1 angle constant
            case L1:
                return this.runOnce(() ->
                        setIntakeAngle(IntakeConstants.INTAKE_ANGLE_L1)
                    );
            // For L2 position, set to the L2 angle constant
            case L2:
                return this.runOnce(() ->
                        setIntakeAngle(IntakeConstants.INTAKE_ANGLE_L2)
                    );
            // For L3 position, set to the L3 angle constant
            case L3:
                return this.runOnce(() ->
                        setIntakeAngle(IntakeConstants.INTAKE_ANGLE_L3)
                    );
            // For L4 position, set to the L4 angle constant
            case L4:
                return this.runOnce(() ->
                        setIntakeAngle(IntakeConstants.INTAKE_ANGLE_L4)
                    );
            // For DEFAULT position, set to the default angle constant
            case DEFAULT:
                return this.runOnce(() ->
                        setIntakeAngle(IntakeConstants.INTAKE_ANGLE_DEFAULT)
                    );
            // If no valid angle specified, default to the default angle constant
            default:
                return this.runOnce(() ->
                        setIntakeAngle(IntakeConstants.INTAKE_ANGLE_DEFAULT)
                    );
        }
    }

    /**
     * Sets the power of the intake rollers using a Command.
     *
     * @param power The power level to set the rollers to.
     * @return The Command to be scheduled.
     */
    public Command setRollers(double power) {
        return this.runOnce(() -> setRollerPower(power));
    }
}
