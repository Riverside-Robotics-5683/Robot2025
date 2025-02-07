package ravenrobotics.robot.subsystems.elevator;

import static ravenrobotics.robot.Configs.elevatorConfig;

import com.revrobotics.AbsoluteEncoder;
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

    private final AbsoluteEncoder leftEncoder;
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
        leftEncoder = leftMotor.getAbsoluteEncoder();
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

    public enum ElevatorPosition{
        CLOSED,
        L1,
        L2,
        L3,
        L4,
        FEED

        //setting the positions
    }
    public void setPosition (ElevatorPosition position){
    
        switch (position) {
            case CLOSED:
             setPosition(ElevatorPosition.CLOSED);
            break;

            case L1:
             setPosition (ElevatorPosition.L1);
            break;

            case L2:
             setPosition(ElevatorPosition.L2);
            break;

            case L3:
             setPosition(ElevatorPosition.L3);
            break;

            case L4:
             setPosition(ElevatorPosition.L4);
            break;

            case FEED:
             setPosition(ElevatorPosition.FEED);
            break;
            default: 
        }

    }
    
    public boolean isElevatorPastLimit(){

    return leftEncoder.getPosition() >=ElevatorConstants.SPEED_LIMIT;

    //test if elevator is past limit, and add a limit
    }

//unc status
}