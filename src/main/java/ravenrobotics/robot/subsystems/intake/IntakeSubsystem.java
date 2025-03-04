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
import ravenrobotics.robot.subsystems.elevator.ElevatorSubsystem;

public class IntakeSubsystem extends SubsystemBase {

    /** Motor controlling the flipper mechanism for intake angle adjustment */
    private final SparkFlex flipperMotor = new SparkFlex(
        IntakeConstants.FLIPPER,
        MotorType.kBrushless
    );

    private final RelativeEncoder flipperEncoder = flipperMotor.getEncoder();

    /** Motor controlling the rollers for intake/outtake */
    private final SparkMax rollerMotor = new SparkMax(
        IntakeConstants.ROLLERS,
        SparkFlex.MotorType.kBrushless
    );

    private final RelativeEncoder rollerEncoder = rollerMotor.getEncoder();

    /** Closed loop controller for the flipper motor */
    private final SparkClosedLoopController flipperController =
        flipperMotor.getClosedLoopController();

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
        flipperMotor.configure(
            Configs.flipperConfig,
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
         * The angle for the coral station.
         */
        FEED,
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
            rollerMotor.getOutputCurrent() > 10 &&
            Math.abs(rollerEncoder.getVelocity()) < 800
        );
    }

    public void setIntakePower(double power) {
        flipperMotor.set(power);
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
            case FEED:
                return this.runOnce(() ->
                        setIntakeAngle(IntakeConstants.INTAKE_ANGLE_FEED)
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
            .andThen(new WaitCommand(0.5))
            .andThen(new WaitUntilCommand(() -> isCoralInIntake()))
            .finallyDo(() -> setRollerPower(0));
    }

    public Command outtakeCoral() {
        if (!ElevatorSubsystem.getInstance().isElevatorAtL1()) {
            return this.runOnce(() -> setRollerPower(0.2))
                .andThen(new WaitCommand(1.5))
                .finallyDo(() -> setRollerPower(0));
        } else {
            System.out.println("L1 Out");
            return outtakeCoralL1();
        }
    }

    public Command outtakeCoralL1() {
        return this.runOnce(() -> setRollerPower(0.1))
            .andThen(new WaitCommand(1.5))
            .finallyDo(() -> setRollerPower(0));
    }

    @Override
    public void periodic() {
        intakeInputs.flipperTargetPosition = targetPosition;

        intakeInputs.flipperPosition = flipperEncoder.getPosition();
        intakeInputs.flipperVelocity = flipperEncoder.getVelocity();

        intakeInputs.flipperVoltage = flipperMotor.getBusVoltage();
        intakeInputs.flipperCurrent = flipperMotor.getOutputCurrent();

        intakeInputs.rollerVelocity = rollerEncoder.getVelocity();

        intakeInputs.rollerVoltage = rollerMotor.getBusVoltage();
        intakeInputs.rollerCurrent = rollerMotor.getOutputCurrent();

        Logger.processInputs("Intake", intakeInputs);
    }
}
