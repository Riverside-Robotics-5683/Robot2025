package ravenrobotics.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.littletonrobotics.junction.Logger;
import ravenrobotics.robot.Configs;
import ravenrobotics.robot.Constants.IntakeConstants;
import ravenrobotics.robot.util.CommandLogger;

public class IntakeSubsystem extends SubsystemBase {

    /** Motor controlling the rollers for intake/outtake */
    private final SparkFlex rollerMotor = new SparkFlex(
        IntakeConstants.ROLLERS,
        SparkFlex.MotorType.kBrushless
    );

    // Encoder to track the roller motor's position and velocity
    private final RelativeEncoder rollerEncoder = rollerMotor.getEncoder();

    // Sensor for detecting when the coral is/isn't in the intake.
    private final DigitalInput coralSensor = new DigitalInput(0);

    // Logged inputs for telemetry and debugging
    private IntakeInputsAutoLogged intakeInputs = new IntakeInputsAutoLogged();

    private Command activeCommand;

    // Singleton instance
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

    // Private constructor for singleton pattern
    private IntakeSubsystem() {
        rollerMotor.configure(
            Configs.rollerConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
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
     * Determines if a coral (game piece) is in the intake.
     *
     * @return true if coral is detected in the intake
     */
    public boolean isCoralInIntake() {
        // The sensor pulls high by default, which the DigitalOutput class interprets as true, hence the logic below :)
        return coralSensor.get() ? false : true;
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

    /**
     * Creates a command to intake a coral (game piece)
     *
     * @return Command that runs the rollers to intake a coral
     */
    public Command intakeCoral() {
        activeCommand = CommandLogger.logCommand(
            (this.runOnce(() -> setRollerPower(-0.1))
                    .andThen(new WaitUntilCommand(() -> isCoralInIntake()))
                    .andThen(
                        this.runOnce(() -> {
                                rollerMotor.stopMotor();
                                rollerMotor.set(0.1);
                            })
                            .andThen(new WaitCommand(0.15))
                            .andThen(
                                this.runOnce(() -> rollerMotor.stopMotor())
                            )
                    )).withDeadline(new WaitCommand(2)),
            "Intake Coral"
        );

        return activeCommand;
    }

    public Command stopIntakeCommand() {
        return this.runOnce(() -> {
                if (activeCommand != null) {
                    activeCommand.cancel();
                }
                rollerMotor.stopMotor();
            });
    }

    /**
     * Creates a command to outtake a coral (game piece)
     * Uses different parameters based on elevator position
     *
     * @return Command that runs the rollers to outtake a coral
     */
    public Command outtakeCoral() {
        activeCommand = CommandLogger.logCommand(
            (this.runOnce(() -> setRollerPower(-1))
                    .andThen(new WaitCommand(0.1))
                    .andThen(new WaitUntilCommand(() -> !isCoralInIntake()))
                    .andThen(
                        this.runOnce(() -> rollerMotor.stopMotor())
                    )).withDeadline(new WaitCommand(2)),
            "Outtake Coral"
        );

        return activeCommand;
    }

    /**
     * Creates a command to outtake a coral when at level 1
     *
     * @return Command that runs rollers to outtake a coral at level 1
     */
    public Command outtakeCoralL1() {
        return this.runOnce(() -> setRollerPower(-1))
            .andThen(new WaitCommand(1.5))
            .finallyDo(() -> setRollerPower(0));
    }

    @Override
    public void periodic() {
        intakeInputs.rollerVelocity = rollerEncoder.getVelocity();

        intakeInputs.rollerVoltage = rollerMotor.getBusVoltage();
        intakeInputs.rollerCurrent = rollerMotor.getOutputCurrent();

        intakeInputs.coralSensor = isCoralInIntake();

        Logger.processInputs("Intake", intakeInputs);
    }
}
