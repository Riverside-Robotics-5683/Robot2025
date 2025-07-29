package ravenrobotics.robot.subsystems.elevator;

import static ravenrobotics.robot.Configs.elevatorConfig;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import ravenrobotics.robot.Constants.ElevatorConstants;
import ravenrobotics.robot.util.CommandLogger;

/**
 * Elevator subsystem or the Elevator
 */
public class ElevatorSubsystem extends SubsystemBase {

    // Elevator motors
    private final SparkFlex leftMotor = new SparkFlex(
        ElevatorConstants.ELEVATOR_LEFT,
        MotorType.kBrushless
    );
    private final SparkFlex rightMotor = new SparkFlex(
        ElevatorConstants.ELEVATOR_RIGHT,
        MotorType.kBrushless
    );

    // Encoders to track elevator position and velocity
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    // Controller for closed-loop control of the elevator
    private final SparkClosedLoopController elevatorController;
    // Feedforward controller to compensate for gravity and friction
    private final ElevatorFeedforward elevatorFeedforward =
        new ElevatorFeedforward(0.42516, 1.2719, 0.1016);

    // Target position for the elevator in encoder units
    private double targetPosition;

    private Command activeCommand;

    // Structure to log elevator data
    private ElevatorInputsAutoLogged elevatorInputs;

    // System identification routine for tuning
    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(null, null, null, state ->
            Logger.recordOutput("SysIDState", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            (Voltage voltage) -> {
                leftMotor.set(
                    voltage.in(Units.Volts) /
                    RobotController.getBatteryVoltage()
                );
            },
            null,
            this
        )
    );

    // Singleton instance of the elevator subsystem
    private static ElevatorSubsystem instance;

    /**
     * Gets the singleton instance of the ElevatorSubsystem
     * @return The elevator subsystem instance
     */
    public static ElevatorSubsystem getInstance() {
        if (instance == null) {
            // If instance hasn't been made yet, make it
            instance = new ElevatorSubsystem();
        }
        return instance; // Return instance
    }

    /**
     * Private constructor to enforce singleton pattern
     */
    private ElevatorSubsystem() {
        // Get encoders from motors
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        elevatorController = leftMotor.getClosedLoopController();

        // Configure motors with appropriate parameters
        leftMotor.configure(
            elevatorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
        rightMotor.configure(
            elevatorConfig.follow(leftMotor),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        // Reset encoder positions to zero
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

        elevatorInputs = new ElevatorInputsAutoLogged();
    }

    /**
     * Creates a command to run a quasistatic system identification test
     * @param direction Direction to move the elevator during the test
     * @return Command for the test
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    /**
     * Creates a command to run a dynamic system identification test
     * @param direction Direction to move the elevator during the test
     * @return Command for the test
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    /**
     * Set the elevator position
     * @param position Target position in encoder units
     */
    private void setPosition(double position) {
        // if (position < targetPosition) {
        //     elevatorController.setReference(position, ControlType.kPosition);
        // } else {
        //     elevatorController.setReference(
        //         position,
        //         ControlType.kMAXMotionPositionControl
        //     );
        // }

        // Calculate feedforward term to compensate for gravity
        double arbFF = elevatorFeedforward.calculate(leftEncoder.getVelocity());

        Logger.recordOutput("Elevator/LeftCommandedPosition", position);

        // Set the reference position with feedforward
        elevatorController.setReference(
            position,
            ControlType.kMAXMotionPositionControl,
            ClosedLoopSlot.kSlot0,
            arbFF,
            ArbFFUnits.kVoltage
        );

        // Update the target position
        targetPosition = position;
    }

    /**
     * Enum defining preset elevator positions
     */
    public enum ElevatorPosition {
        CLOSED, // Fully retracted
        L1, // Level 1
        L2, // Level 2
        L3, // Level 3
        L4, // Level 4
        FEED, // Feeding position
    }

    /**
     * Sets the elevator to a preset position
     * @param position The preset position to move to
     */
    private void setPosition(ElevatorPosition position) {
        switch (position) {
            case CLOSED:
                setPosition(0);
                break;
            case L1:
                setPosition(ElevatorConstants.ELEVATOR_L1);
                break;
            case L2:
                setPosition(ElevatorConstants.ELEVATOR_L2);
                break;
            case L3:
                setPosition(ElevatorConstants.ELEVATOR_L3);
                break;
            case L4:
                setPosition(ElevatorConstants.ELEVATOR_L4);
                break;
            case FEED:
                setPosition(ElevatorConstants.ELEVATOR_FEED);
                break;
        }
    }

    /**
     * Sets the raw power to the elevator motors
     * @param power Power value (-1.0 to 1.0)
     */
    private void setPower(double power) {
        leftMotor.set(power);
    }

    /**
     * Checks if the elevator is extended beyond level 2
     * @return True if elevator is past L2 position
     */
    public boolean isElevatorPastLimit() {
        return leftEncoder.getPosition() > ElevatorConstants.ELEVATOR_L2;
        // Test if elevator is past limit, and add a limit
    }

    /**
     * Checks if the elevator is at the L1 position
     * @return True if elevator is between 0 and L2
     */
    public boolean isElevatorAtL1() {
        return (
            leftEncoder.getPosition() > 0 &&
            leftEncoder.getPosition() < ElevatorConstants.ELEVATOR_L2
        );
    }

    /**
     * Checks if the elevator is at the target position within tolerance
     * @param tolerance Maximum allowable error
     * @return True if elevator is at target position
     */
    public boolean isElevatorAtPosition(double tolerance) {
        var currentPosition = leftEncoder.getPosition();
        var difference = Math.abs(currentPosition - targetPosition);
        return difference <= tolerance;
    }

    /**
     * Checks if the elevator is at target position with default tolerance
     * @return True if elevator is at target position
     */
    public boolean isElevatorAtPosition() {
        return isElevatorAtPosition(0.1);
    }

    /**
     * Creates a command to set the elevator to a preset position and wait until it arrives
     * @param position The preset position to move to
     * @return Command to move to position
     */
    public Command setElevatorPosition(ElevatorPosition position) {
        activeCommand = CommandLogger.logCommand(
            this.runOnce(() -> setPosition(position)).andThen(
                    new WaitUntilCommand(() -> isElevatorAtPosition(2))
                        .raceWith(new WaitCommand(5))
                        .andThen(rawElevatorPower(0))
                ),
            "Elevator " + position.toString()
        );

        return activeCommand;
    }

    public Command stopElevatorCommmand() {
        return this.runOnce(() -> {
                if (activeCommand != null) {
                    activeCommand.cancel();
                }

                leftMotor.stopMotor();
            });
    }

    /**
     * Creates a command to set a raw power to the elevator momentarily
     * @param power Power value (-1.0 to 1.0)
     * @return Command to apply power
     */
    public Command rawElevatorPower(double power) {
        return this.runOnce(() -> setPower(power)).finallyDo(() -> setPower(0));
    }

    /**
     * Creates a command to control the elevator with a power supplier
     * @param power Supplier of power values
     * @return Command to continuously apply power
     */
    public Command rawElevatorPower(DoubleSupplier power) {
        return this.run(() -> setPower(power.getAsDouble()));
    }

    @Override
    public void periodic() {
        // Update logged data from left motor
        elevatorInputs.leftMotorPosition = leftEncoder.getPosition();
        elevatorInputs.leftMotorVelocity = leftEncoder.getVelocity();

        elevatorInputs.leftMotorVoltage =
            leftMotor.get() * RobotController.getBatteryVoltage();
        elevatorInputs.leftMotorCurrent = leftMotor.getOutputCurrent();

        // Update logged data from right motor
        elevatorInputs.rightMotorPosition = rightEncoder.getPosition();
        elevatorInputs.rightMotorVelocity = rightEncoder.getVelocity();

        elevatorInputs.rightMotorVoltage =
            rightMotor.get() * RobotController.getBatteryVoltage();
        elevatorInputs.rightMotorCurrent = rightMotor.getOutputCurrent();

        // Save target position for logging
        elevatorInputs.targetPosition = targetPosition;

        // Send all inputs to logger
        Logger.processInputs("/Elevator", elevatorInputs);
    }
}
