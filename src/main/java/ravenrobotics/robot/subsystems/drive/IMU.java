package ravenrobotics.robot.subsystems.drive;

import static ravenrobotics.robot.Configs.imuConfig;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearAcceleration;
import ravenrobotics.robot.Constants.IMUConstants;

/**
 * Class for getting information from a Pigeon 2.0 IMU.
 */
public class IMU {
  private final Pigeon2 imu; // The IMU.
  // TODO: Implement simulation for the IMU.
  private final Pigeon2SimState imuSim; // The simulated IMU.

  // The StatusSignals for the rotation axes.
  private final StatusSignal<Angle> imuRawRoll;
  private final StatusSignal<Angle> imuRawPitch;
  private final StatusSignal<Angle> imuRawYaw;

  // The StatusSignals for the axes accelerations.
  private final StatusSignal<LinearAcceleration> imuRawXAccel;
  private final StatusSignal<LinearAcceleration> imuRawYAccel;
  private final StatusSignal<LinearAcceleration> imuRawZAccel;

  /**
   * Class for getting information from a Pigeon 2.0 IMU.
   */
  public IMU() {
    imu = new Pigeon2(IMUConstants.IMU); // Initialize the IMU object.

    imu.getConfigurator().apply(imuConfig); // Apply the IMU config from Configs.

    imuSim = imu.getSimState(); // Get the SimState of the Pigeon2 for simulation.

    // Get the rotation axes StatusSignals.
    imuRawRoll = imu.getRoll();
    imuRawPitch = imu.getPitch();
    imuRawYaw = imu.getYaw();

    // Get the rotation axes acceleration StatusSignals.
    imuRawXAccel = imu.getAccelerationX();
    imuRawYAccel = imu.getAccelerationY();
    imuRawZAccel = imu.getAccelerationZ();

    // Set the update frequencies for the rotation axes.
    StatusSignal.setUpdateFrequencyForAll(100.0, imuRawRoll, imuRawPitch, imuRawYaw);
    // Set the update frequencies for the acceleration signals.
    StatusSignal.setUpdateFrequencyForAll(50.0, imuRawXAccel, imuRawYAccel, imuRawZAccel);

    imu.optimizeBusUtilization(); // Optimize bus utilization to reduce CPU load.
  }

  /**
   * Updates the given IMUInputs for processing.
   *
   * @param inputs The IMUInputs.
   */
  public void updateInputs(IMUInputs inputs) {
    // Refresh all of the signals for usage.
    StatusSignal.refreshAll(imuRawRoll, imuRawPitch, imuRawYaw, imuRawXAccel, imuRawYAccel, imuRawZAccel);

    inputs.imuHeading = imu.getRotation2d(); // Get the IMU's heading.
    inputs.imuOrientation = imu.getRotation3d(); // Get the IMU's orientation.

    inputs.imuRawRoll = imuRawRoll.getValue().in(Units.Degrees); // Get the roll in degrees.
    inputs.imuRawPitch = imuRawPitch.getValue().in(Units.Degrees); // Get the pitch in degrees.
    inputs.imuRawYaw = imuRawYaw.getValue().in(Units.Degrees); // Get the yaw in degrees.

    inputs.imuRawXAccel = imuRawXAccel.getValue().in(Units.Gs); // Get the x-axis acceleration in Gs.
    inputs.imuRawYAccel = imuRawYAccel.getValue().in(Units.Gs); // Get the y-axis acceleration in Gs.
    inputs.imuRawZAccel = imuRawZAccel.getValue().in(Units.Gs); // Get the z-axis acceleration in Gs.
  }

  /**
   * Set the heading of the IMU.
   *
   * @param heading The new heading.
   */
  public void setHeading(double heading) {
    imu.setYaw(heading);
  }

  /**
   * Reset the heading of the IMU to zero.
   */
  public void resetHeading() {
    imu.reset();
  }
}
