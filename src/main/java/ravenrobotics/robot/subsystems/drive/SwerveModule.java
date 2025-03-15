package ravenrobotics.robot.subsystems.drive;

import static ravenrobotics.robot.Configs.swerveAngleConfig;
import static ravenrobotics.robot.Configs.swerveDriveConfig;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import ravenrobotics.robot.Constants.KinematicsConstants;
import ravenrobotics.robot.Constants.SwerveModuleConstants;
import ravenrobotics.robot.Robot;

/**
 * Class for controlling a swerve module.
 */
public class SwerveModule {

    private final SparkFlex driveMotor; // Drive motor (drives the wheel).
    private final SparkMax angleMotor; // Angle motor (changes the direction the wheel is pointing).

    private SparkFlexSim driveSim; // Drive motor simulation.
    private SparkMaxSim angleSim; // Angle motor simulation.

    private FlywheelSim drivePhysicsSim; // Physics simulation for the drive motor.
    private FlywheelSim anglePhysicsSim; // Physics simulation for the angle motor.

    private final RelativeEncoder driveEncoder; // The drive motor encoder.
    private final AbsoluteEncoder angleEncoder; // The angle motor encoder.

    private final SparkClosedLoopController driveController; // The Feedforward controller for the drive motor.
    private final SparkClosedLoopController angleController; // The PID controller for the drive motor.

    private final Rotation2d moduleOffset; // The module's offset around the chassis (used for modifying the angle motor
    // position value).
    private SwerveModuleState targetState; // The target state of the module.

    /**
     * Class for controlling a swerve module.
     *
     * @param id The ID of the swerve module. 0 is FL, 1 is FR, 2 is RL, 3 is RR.
     */
    public SwerveModule(int id) {
        switch (id) {
            case 0: // Create the FL motors and offset.
                driveMotor = new SparkFlex(
                    SwerveModuleConstants.FL_DRIVE,
                    MotorType.kBrushless
                );
                angleMotor = new SparkMax(
                    SwerveModuleConstants.FL_ANGLE,
                    MotorType.kBrushless
                );
                moduleOffset = Rotation2d.fromRadians(
                    SwerveModuleConstants.FL_OFFSET
                );
                break;
            case 1: // Create the FR motors and offset.
                driveMotor = new SparkFlex(
                    SwerveModuleConstants.FR_DRIVE,
                    MotorType.kBrushless
                );
                angleMotor = new SparkMax(
                    SwerveModuleConstants.FR_ANGLE,
                    MotorType.kBrushless
                );
                moduleOffset = Rotation2d.fromRadians(
                    SwerveModuleConstants.FR_OFFSET
                );
                break;
            case 2: // Create the RL motors and offset.
                driveMotor = new SparkFlex(
                    SwerveModuleConstants.RL_DRIVE,
                    MotorType.kBrushless
                );
                angleMotor = new SparkMax(
                    SwerveModuleConstants.RL_ANGLE,
                    MotorType.kBrushless
                );
                moduleOffset = Rotation2d.fromRadians(
                    SwerveModuleConstants.RL_OFFSET
                );
                break;
            case 3: // Create the RR motors and offset.
                driveMotor = new SparkFlex(
                    SwerveModuleConstants.RR_DRIVE,
                    MotorType.kBrushless
                );
                angleMotor = new SparkMax(
                    SwerveModuleConstants.RR_ANGLE,
                    MotorType.kBrushless
                );
                moduleOffset = Rotation2d.fromRadians(
                    SwerveModuleConstants.RR_OFFSET
                );
                break;
            default: // Throw an error because no other modules exist.
                throw new IllegalArgumentException("ID must be in range 0-3.");
        }

        if (Robot.isSimulation()) { // If the code is running off-robot.
            driveSim = new SparkFlexSim(driveMotor, DCMotor.getNeoVortex(1)); // Create the drive sim.
            angleSim = new SparkMaxSim(angleMotor, DCMotor.getNeo550(1)); // Create the angle sim.

            // Create the physics simulations.
            drivePhysicsSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                    DCMotor.getNeoVortex(1),
                    0.025,
                    KinematicsConstants.DRIVE_CONVERSION_FACTOR
                ),
                DCMotor.getNeoVortex(1)
            );
            anglePhysicsSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                    DCMotor.getNeo550(1),
                    0.004,
                    KinematicsConstants.ANGLE_CONVERSION_FACTOR
                ),
                DCMotor.getNeo550(1)
            );
        }

        driveEncoder = driveMotor.getEncoder(); // Get the drive encoder.
        angleEncoder = angleMotor.getAbsoluteEncoder(); // Get the angle encoder.

        // Get the PID/Feedforward controllers.
        driveController = driveMotor.getClosedLoopController();
        angleController = angleMotor.getClosedLoopController();

        // Configure the motors using the configs from Configs.
        driveMotor.configure(
            swerveDriveConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
        angleMotor.configure(
            swerveAngleConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        driveEncoder.setPosition(0); // Reset the drive encoder position

        // Initialize the target state holder.
        targetState = new SwerveModuleState(
            0,
            new Rotation2d(angleEncoder.getPosition())
        );
    }

    /**
     * Updates the given SwerveModuleInputs for processing.
     *
     * @param inputs The SwerveModuleInputs.
     */
    public void updateInputs(SwerveModuleInputs inputs) {
        // Update the drive motor inputs.
        inputs.drivePositionMeters = driveEncoder.getPosition();
        inputs.driveVelocityMetersPerSecond = driveEncoder.getVelocity();
        inputs.driveVoltage = driveMotor.getBusVoltage();
        inputs.driveAmps = driveMotor.getOutputCurrent();

        SwerveModuleState moduleState = getModuleState(); // Get the module state for later use.

        // Update the angle motor inputs.
        inputs.anglePosition = moduleState.angle;
        inputs.angleVelocityMetersPerSecond = moduleState.speedMetersPerSecond;
        inputs.angleVoltage = angleMotor.getBusVoltage();
        inputs.angleAmps = angleMotor.getOutputCurrent();

        // Update odometry inputs.
        inputs.targetState = targetState;
        inputs.currentState = moduleState;
        inputs.modulePosition = getModulePosition();
    }

    /**
     * Set the module state.
     *
     * @param state The target state.
     */
    public SwerveModuleState setModuleState(SwerveModuleState state) {
        // Set the correct angle from the module's offset.
        SwerveModuleState correctedState = new SwerveModuleState(
            state.speedMetersPerSecond,
            state.angle.plus(moduleOffset)
        );

        correctedState.optimize(new Rotation2d(angleEncoder.getPosition())); // Optimize the path of travel.

        if (correctedState.speedMetersPerSecond != 0) {
            // Update the motor targets.
            driveController.setReference(
                correctedState.speedMetersPerSecond,
                ControlType.kVelocity
            );
            // driveMotor.set(
            //     state.speedMetersPerSecond / KinematicsConstants.MAX_MODULE_SPEED
            // );
        } else {
            driveMotor.stopMotor();
        }
        angleController.setReference(
            correctedState.angle.getRadians(),
            ControlType.kPosition
        );

        targetState = correctedState;
        return targetState;
    }

    /**
     * Get the module's current state.
     *
     * @return The SwerveModuleState.
     */
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
            driveEncoder.getVelocity(), // Get the drive motor velocity.
            Rotation2d.fromRadians(angleEncoder.getPosition()).minus(
                moduleOffset
            )
        ); // Get the corrected angle.
    }

    /**
     * Get the module's current position.
     *
     * @return The SwerveModulePosition.
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(), // Get the drive motor position.
            Rotation2d.fromRadians(angleEncoder.getPosition()).minus(
                moduleOffset
            )
        ); // Get the corrected angle.
    }

    /**
     * Update the simulation devices.
     */
    public void updateSimDevices() {
        // Throw an error if any of the sim devices haven't been initialized.
        if (
            driveSim == null ||
            angleSim == null ||
            drivePhysicsSim == null ||
            anglePhysicsSim == null
        ) {
            throw new IllegalStateException("Must be called in simulation.");
        }

        // Get the current voltage from simulated RoboRIO. :D
        double voltageIn = RoboRioSim.getVInVoltage();

        // Set the voltage input for both motors.
        drivePhysicsSim.setInput(driveSim.getAppliedOutput() * voltageIn);
        anglePhysicsSim.setInput(angleSim.getAppliedOutput() * voltageIn);

        // Update the motor physics sims.
        drivePhysicsSim.update(0.02);
        anglePhysicsSim.update(0.02);

        // Update the motor sims with the new physics sims.
        driveSim.iterate(
            Units.radiansPerSecondToRotationsPerMinute(
                drivePhysicsSim.getAngularVelocityRadPerSec()
            ),
            voltageIn,
            0.02
        );
        angleSim.iterate(
            Units.radiansPerSecondToRotationsPerMinute(
                anglePhysicsSim.getAngularVelocityRadPerSec()
            ),
            voltageIn,
            0.02
        );
    }
}
