package ravenrobotics.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.littletonrobotics.junction.Logger;
import ravenrobotics.robot.Configs;
import ravenrobotics.robot.Constants.IntakeConstants;
import ravenrobotics.robot.subsystems.elevator.ElevatorSubsystem;

public class IntakeSubsystem extends SubsystemBase {

    /** Motor controlling the rollers for intake/outtake */
    private final SparkFlex rollerMotor = new SparkFlex(
        IntakeConstants.ROLLERS,
        SparkFlex.MotorType.kBrushless
    );

    private final RelativeEncoder rollerEncoder = rollerMotor.getEncoder();

    private IntakeInputsAutoLogged intakeInputs = new IntakeInputsAutoLogged();

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

    public boolean isCoralInIntake() {
        return (
            rollerMotor.getOutputCurrent() > 10 &&
            Math.abs(rollerEncoder.getVelocity()) < 800
        );
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

    public Command intakeCoral() {
        return this.runOnce(() -> setRollerPower(-0.5))
            .andThen(new WaitCommand(0.5))
            .finallyDo(() -> setRollerPower(0));
    }

    public Command outtakeCoral() {
        if (!ElevatorSubsystem.getInstance().isElevatorAtL1()) {
            return this.runOnce(() -> setRollerPower(-1))
                .andThen(new WaitCommand(1.5))
                .finallyDo(() -> setRollerPower(0));
        } else {
            System.out.println("L1 Out");
            return outtakeCoralL1();
        }
    }

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

        Logger.processInputs("Intake", intakeInputs);
    }
}
