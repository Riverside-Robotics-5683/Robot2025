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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import ravenrobotics.robot.Constants.ElevatorConstants;

/**
 * Elevator subsystem or the Elevator
 */
public class ElevatorSubsystem extends SubsystemBase {

    //Elevator motors
    private final SparkFlex leftMotor = new SparkFlex(
        ElevatorConstants.ELEVATOR_LEFT,
        MotorType.kBrushless
    );
    private final SparkFlex rightMotor = new SparkFlex(
        ElevatorConstants.ELEVATOR_RIGHT,
        MotorType.kBrushless
    );

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final SparkClosedLoopController elevatorController;
    private final ElevatorFeedforward elevatorFeedforward =
        new ElevatorFeedforward(0.42516, 1.2719, 0.1016);

    private double targetPosition;

    private ElevatorInputsAutoLogged elevatorInputs;

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

    private static ElevatorSubsystem instance;

    public static ElevatorSubsystem getInstance() {
        if (instance == null) {
            //if instance hasn't been made yet, make it
            instance = new ElevatorSubsystem();
        }
        return instance; //return instance
    }

    private ElevatorSubsystem() {
        //get encoder
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        elevatorController = leftMotor.getClosedLoopController();

        //configuring motors
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

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

        elevatorInputs = new ElevatorInputsAutoLogged();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    /**
     * Set the elevator position
     * @param position
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

        double arbFF = elevatorFeedforward.calculate(leftEncoder.getVelocity());

        elevatorController.setReference(
            position,
            ControlType.kMAXMotionPositionControl,
            ClosedLoopSlot.kSlot0,
            arbFF,
            ArbFFUnits.kVoltage
        );

        targetPosition = position;
    }

    public enum ElevatorPosition {
        CLOSED,
        L1,
        L2,
        L3,
        L4,
        FEED,
        //setting the positions
    }

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

    private void setPower(double power) {
        leftMotor.set(power);
    }

    public boolean isElevatorPastLimit() {
        return leftEncoder.getPosition() > ElevatorConstants.ELEVATOR_L2;
        //test if elevator is past limit, and add a limit
    }

    public boolean isElevatorAtL1() {
        return (
            leftEncoder.getPosition() > 0 &&
            leftEncoder.getPosition() < ElevatorConstants.ELEVATOR_L2
        );
    }

    public boolean isElevatorAtPosition(double tolerance) {
        var currentPosition = leftEncoder.getPosition();
        var difference = Math.abs(currentPosition - targetPosition);
        return difference <= tolerance;
    }

    public boolean isElevatorAtPosition() {
        return isElevatorAtPosition(0.1);
    }

    public Command setElevatorPosition(ElevatorPosition position) {
        return this.runOnce(() -> setPosition(position)).andThen(
                new WaitUntilCommand(() -> isElevatorAtPosition(2))
            );
    }

    public Command rawElevatorPower(double power) {
        return this.runOnce(() -> setPower(power)).finallyDo(() -> setPower(0));
    }

    public Command rawElevatorPower(DoubleSupplier power) {
        return this.run(() -> setPower(power.getAsDouble()));
    }

    @Override
    public void periodic() {
        elevatorInputs.leftMotorPosition = leftEncoder.getPosition();
        elevatorInputs.leftMotorVelocity = leftEncoder.getVelocity();

        elevatorInputs.leftMotorVoltage =
            leftMotor.get() * RobotController.getBatteryVoltage();
        elevatorInputs.leftMotorCurrent = leftMotor.getOutputCurrent();

        elevatorInputs.rightMotorPosition = rightEncoder.getPosition();
        elevatorInputs.rightMotorVelocity = rightEncoder.getVelocity();

        elevatorInputs.rightMotorVoltage =
            rightMotor.get() * RobotController.getBatteryVoltage();
        elevatorInputs.rightMotorCurrent = rightMotor.getOutputCurrent();

        elevatorInputs.targetPosition = targetPosition;

        Logger.processInputs("/Elevator", elevatorInputs);
    }
    //unc status
}
