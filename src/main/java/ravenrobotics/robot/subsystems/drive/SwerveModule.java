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

public class SwerveModule {
  private final SparkFlex driveMotor;
  private final SparkMax angleMotor;

  private SparkFlexSim driveSim;
  private SparkMaxSim angleSim;

  private FlywheelSim drivePhysicsSim;
  private FlywheelSim anglePhysicsSim;

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder angleEncoder;

  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController angleController;

  private final double moduleOffset;
  private SwerveModuleState targetState;

  public SwerveModule(int id) {
    switch (id) {
      case 0:
        driveMotor = new SparkFlex(SwerveModuleConstants.FL_DRIVE, MotorType.kBrushless);
        angleMotor = new SparkMax(SwerveModuleConstants.FL_ANGLE, MotorType.kBrushless);
        moduleOffset = SwerveModuleConstants.FL_OFFSET;
        break;
      case 1:
        driveMotor = new SparkFlex(SwerveModuleConstants.FR_DRIVE, MotorType.kBrushless);
        angleMotor = new SparkMax(SwerveModuleConstants.FR_ANGLE, MotorType.kBrushless);
        moduleOffset = SwerveModuleConstants.FR_OFFSET;
        break;
      case 2:
        driveMotor = new SparkFlex(SwerveModuleConstants.RL_DRIVE, MotorType.kBrushless);
        angleMotor = new SparkMax(SwerveModuleConstants.RL_ANGLE, MotorType.kBrushless);
        moduleOffset = SwerveModuleConstants.RL_OFFSET;
        break;
      case 3:
        driveMotor = new SparkFlex(SwerveModuleConstants.RR_DRIVE, MotorType.kBrushless);
        angleMotor = new SparkMax(SwerveModuleConstants.RR_ANGLE, MotorType.kBrushless);
        moduleOffset = SwerveModuleConstants.RR_OFFSET;
        break;
      default:
        throw new IllegalArgumentException("ID must be in range 0-3.");
    }

    if (Robot.isSimulation()) {
      driveSim = new SparkFlexSim(driveMotor, DCMotor.getNeoVortex(1));
      angleSim = new SparkMaxSim(angleMotor, DCMotor.getNeo550(1));

      drivePhysicsSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(1), 0.025,
          KinematicsConstants.DRIVE_CONVERSION_FACTOR), DCMotor.getNeoVortex(1));
      anglePhysicsSim = new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(1), 0.004, KinematicsConstants.ANGLE_CONVERSION_FACTOR),
          DCMotor.getNeo550(1));
    }

    driveEncoder = driveMotor.getEncoder();
    angleEncoder = angleMotor.getAbsoluteEncoder();

    driveController = driveMotor.getClosedLoopController();
    angleController = angleMotor.getClosedLoopController();

    driveMotor.configure(swerveDriveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    angleMotor.configure(swerveAngleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveEncoder.setPosition(0);

    targetState = new SwerveModuleState(0, new Rotation2d(angleEncoder.getPosition()));
  }

  public void updateInputs(SwerveModuleInputs inputs) {
    inputs.drivePositionMeters = driveEncoder.getPosition();
    inputs.driveVelocityMetersPerSecond = driveEncoder.getVelocity();
    inputs.driveVoltage = driveMotor.getBusVoltage();
    inputs.driveAmps = driveMotor.getOutputCurrent();

    var moduleState = getModuleState();

    inputs.anglePosition = moduleState.angle;
    inputs.angleVelocityMetersPerSecond = moduleState.speedMetersPerSecond;
    inputs.angleVoltage = angleMotor.getBusVoltage();
    inputs.angleAmps = angleMotor.getOutputCurrent();

    inputs.targetState = targetState;
    inputs.currentState = moduleState;
    inputs.modulePosition = getModulePosition();
  }

  public SwerveModuleState setModuleState(SwerveModuleState state) {
    SwerveModuleState correctedState = new SwerveModuleState(
        state.speedMetersPerSecond,
        state.angle.plus(Rotation2d.fromRadians(moduleOffset)));

    correctedState.optimize(new Rotation2d(angleEncoder.getPosition()));

    driveController.setReference(correctedState.speedMetersPerSecond, ControlType.kVelocity);
    angleController.setReference(correctedState.angle.getRadians(), ControlType.kPosition);

    targetState = state;
    return targetState;
  }

  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(
        driveEncoder.getVelocity(),
        Rotation2d.fromRadians(angleEncoder.getPosition() - moduleOffset));
  }

  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(
        driveEncoder.getPosition(),
        Rotation2d.fromRadians(angleEncoder.getPosition() - moduleOffset));
  }

  public double[] updateSimDevices() {
    if (driveSim == null || angleSim == null || drivePhysicsSim == null || anglePhysicsSim == null) {
      throw new IllegalStateException("Must be called in simulation.");
    }

    double voltageIn = RoboRioSim.getVInVoltage();

    drivePhysicsSim.setInput(driveSim.getAppliedOutput() * voltageIn);
    anglePhysicsSim.setInput(angleSim.getAppliedOutput() * voltageIn);

    drivePhysicsSim.update(0.02);
    anglePhysicsSim.update(0.02);

    driveSim.iterate(Units.radiansPerSecondToRotationsPerMinute(drivePhysicsSim.getAngularVelocityRadPerSec()),
        voltageIn, 0.02);
    angleSim.iterate(Units.radiansPerSecondToRotationsPerMinute(anglePhysicsSim.getAngularVelocityRadPerSec()),
        voltageIn, 0.02);

    return new double[] {
        drivePhysicsSim.getCurrentDrawAmps(),
        anglePhysicsSim.getCurrentDrawAmps()
    };
  }
}
