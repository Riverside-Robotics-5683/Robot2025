package ravenrobotics.robot.subsystems.drive;

import static ravenrobotics.robot.Configs.imuConfig;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearAcceleration;
import ravenrobotics.robot.Constants.IMUConstants;

public class IMU {
  private final Pigeon2 imu;
  private final Pigeon2SimState imuSim;

  private final StatusSignal<Angle> imuRawRoll;
  private final StatusSignal<Angle> imuRawPitch;
  private final StatusSignal<Angle> imuRawYaw;

  private final StatusSignal<LinearAcceleration> imuRawXAccel;
  private final StatusSignal<LinearAcceleration> imuRawYAccel;
  private final StatusSignal<LinearAcceleration> imuRawZAccel;

  public IMU() {
    imu = new Pigeon2(IMUConstants.IMU);

    imu.getConfigurator().apply(imuConfig);

    imuSim = imu.getSimState();

    imuRawRoll = imu.getRoll();
    imuRawPitch = imu.getPitch();
    imuRawYaw = imu.getYaw();

    imuRawXAccel = imu.getAccelerationX();
    imuRawYAccel = imu.getAccelerationY();
    imuRawZAccel = imu.getAccelerationZ();

    StatusSignal.setUpdateFrequencyForAll(100.0, imuRawRoll, imuRawPitch, imuRawYaw);
    StatusSignal.setUpdateFrequencyForAll(50.0, imuRawXAccel, imuRawYAccel, imuRawZAccel);

    imu.optimizeBusUtilization();
  }

  public void updateInputs(IMUInputs inputs) {
    StatusSignal.refreshAll(imuRawRoll, imuRawPitch, imuRawYaw, imuRawXAccel, imuRawYAccel, imuRawZAccel);

    inputs.imuHeading = imu.getRotation2d();
    inputs.imuOrientation = imu.getRotation3d();

    inputs.imuRawRoll = imuRawRoll.getValue().in(Units.Degrees);
    inputs.imuRawPitch = imuRawPitch.getValue().in(Units.Degrees);
    inputs.imuRawYaw = imuRawYaw.getValue().in(Units.Degrees);

    inputs.imuRawXAccel = imuRawXAccel.getValue().in(Units.Gs);
    inputs.imuRawYAccel = imuRawYAccel.getValue().in(Units.Gs);
    inputs.imuRawZAccel = imuRawZAccel.getValue().in(Units.Gs);
  }

  public void setHeading(double heading) {
    imu.setYaw(heading);
  }

  public void resetHeading() {
    imu.reset();
  }
}
