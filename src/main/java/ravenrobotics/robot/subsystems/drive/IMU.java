package ravenrobotics.robot.subsystems.drive;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

/**
 * Class for getting information from a Pigeon 2.0 IMU.
 */
public class IMU {

    private final AHRS imu = new AHRS(NavXComType.kMXP_SPI);

    /**
     * Class for getting information from a Pigeon 2.0 IMU.
     */
    public IMU() {
        imu.reset();
    }

    /**
     * Updates the given IMUInputs for processing.
     *
     * @param inputs The IMUInputs.
     */
    public void updateInputs(IMUInputs inputs) {
        inputs.imuHeading = imu.getRotation2d();
        inputs.imuOrientation = imu.getRotation3d();

        inputs.imuRawPitch = imu.getPitch();
        inputs.imuRawRoll = imu.getRoll();
        inputs.imuRawYaw = imu.getYaw();

        inputs.imuRawXAccel = imu.getRawAccelX();
        inputs.imuRawYAccel = imu.getRawAccelY();
        inputs.imuRawZAccel = imu.getRawAccelZ();
    }

    /**
     * Set the heading of the IMU.
     *
     * @param heading The new heading.
     */
    public void setHeading(double heading) {
        imu.setAngleAdjustment(heading);
    }

    /**
     * Reset the heading of the IMU to zero.
     */
    public void resetHeading() {
        imu.reset();
    }
}
