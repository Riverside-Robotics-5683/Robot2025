package ravenrobotics.robot.subsystems.elevator;

import static ravenrobotics.robot.Configs.elevatorConfig;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.robot.Constants.ElevatorConstants;

/**
 * Elevator subsystem or the Elevator
 */
public class ElevatorSubsystem extends SubsystemBase{

    //Elevator motors
    private final SparkFlex leftMotor = new SparkFlex(ElevatorConstants.ELEVATOR_LEFT, MotorType.kBrushless);
    private final SparkFlex rightMotor = new SparkFlex(ElevatorConstants.ELEVATOR_RIGHT, MotorType.kBrushless);

    private final RelativeEncoder leftEncoder;
    private final SparkClosedLoopController elevatorController;

    private static ElevatorSubsystem instance;

    public static ElevatorSubsystem getInstance(){
        if (instance == null){
            //if instance hasn't been made yet, make it
            instance = new ElevatorSubsystem();

        } 
        return instance; //return instance
    }

    private ElevatorSubsystem(){
         //get encoder
        leftEncoder = leftMotor.getEncoder();
        elevatorController = leftMotor.getClosedLoopController();

        //configuring motors
        leftMotor.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(elevatorConfig.follow(ElevatorConstants.ELEVATOR_LEFT), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    } 

     /**
      * Get the elevator subsystem position
      * @param position
      */
    public void setPosition(double position){
        elevatorController.setReference(position, ControlType.kPosition);
    }
}