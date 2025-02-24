package ravenrobotics.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.littletonrobotics.junction.Logger;
import ravenrobotics.robot.Configs;
import ravenrobotics.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    /** Motor controlling the flipper mechanism for intake angle adjustment */
    private final SparkFlex topFlipperMotor = new SparkFlex(
        IntakeConstants.FLIPPER_TOP,
        SparkFlex.MotorType.kBrushless
    );

    private final SparkFlex bottomFlipperMotor = new SparkFlex(
        IntakeConstants.FLIPPER_BOTTOM,
        MotorType.kBrushless
    );

    private final RelativeEncoder topFlipperEncoder =
        topFlipperMotor.getEncoder();
    private final RelativeEncoder bottomFlipperEncoder =
        bottomFlipperMotor.getEncoder();

    /** Motor controlling the rollers for intake/outtake */
    private final SparkMax rollerMotor = new SparkMax(
        IntakeConstants.ROLLERS,
        SparkFlex.MotorType.kBrushless
    );

    private final RelativeEncoder rollerEncoder = rollerMotor.getEncoder();

    /** Closed loop controller for the flipper motor */
    private final SparkClosedLoopController flipperController =
        topFlipperMotor.getClosedLoopController();

    private IntakeInputsAutoLogged intakeInputs = new IntakeInputsAutoLogged();

    private double targetPosition = 0.0;

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

    private IntakeSubsystem() {
        topFlipperMotor.configure(
            Configs.flipperConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
        bottomFlipperMotor.configure(
            Configs.flipperConfig.follow(topFlipperMotor),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        rollerMotor.configure(
            Configs.rollerConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
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
     * Sets the intake angle.
     *
     * @param position The desired angle setpoint
     */
    private void setIntakeAngle(double position) {
        targetPosition = position;
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

    public boolean isCoralInIntake() {
        return (
            rollerMotor.getAppliedOutput() > 10 &&
            rollerEncoder.getVelocity() < 800
        );
    }

    public void setIntakePower(double power) {
        topFlipperMotor.set(power);
    }

    /**
     * Sets the angle of the intake.
     *
     * @param angle The desired angle position.
     * @return The Command to be scheduled.
     */
    public Command setIntakeAngle(IntakeAngle angle) {
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
        return this.runOnce(() -> setRollerPower(power)).finallyDo(() ->
                setRollerPower(0)
            );
    }

    public Command setIntake(double power) {
        return this.runEnd(
                () -> setIntakePower(power),
                () -> setIntakePower(0)
            );
    }

    public Command intakeCoral() {
        return this.runOnce(() -> setRollerPower(-0.1))
            .andThen(new WaitUntilCommand(() -> isCoralInIntake()))
            .finallyDo(() -> setRollerPower(0));
    }

    public Command outtakeCoral() {
        return this.runOnce(() -> setRollerPower(0.2))
            .andThen(new WaitCommand(1.5))
            .finallyDo(() -> setRollerPower(0));
    }

    @Override
    public void periodic() {
        intakeInputs.flipperTargetPosition = targetPosition;

        intakeInputs.topFlipperPosition = topFlipperEncoder.getPosition();
        intakeInputs.topFlipperVelocity = topFlipperEncoder.getVelocity();

        intakeInputs.topFlipperVoltage = topFlipperMotor.getBusVoltage();
        intakeInputs.topFlipperCurrent = topFlipperMotor.getOutputCurrent();

        intakeInputs.bottomFlipperPosition = bottomFlipperEncoder.getPosition();
        intakeInputs.bottomFlipperVelocity = bottomFlipperEncoder.getVelocity();

        intakeInputs.bottomFlipperVoltage = bottomFlipperMotor.getBusVoltage();
        intakeInputs.bottomFlipperCurrent =
            bottomFlipperMotor.getOutputCurrent();

        Logger.processInputs("Intake", intakeInputs);
    }
}
