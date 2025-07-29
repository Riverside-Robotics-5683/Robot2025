package ravenrobotics.robot.util;

/**
 * Used to store deadband and rate limit values for various drivers/controllers.
 */
public class DriverProfile {
  /**
   * The name of the profile.
   */
  private final String driverName;

  /**
   * The deadband for the x-axis.
   */
  private final double xDeadband;
  /**
   * The deadband for the y-axis.
   */
  private final double yDeadband;
  /**
   * The deadband for the z-axis.
   */
  private final double zDeadband;

  /**
   * The rate limit for the x-axis.
   */
  private final double xRateLimit;
  /**
   * The rate limit for the y-axis.
   */
  private final double yRateLimit;
  /**
   * The rate limit for the z-axis.
   */
  private final double zRateLimit;

  /**
   * Creates a new DriverProfile for use.
   *
   * @param driverName The name of the profile/driver.
   * 
   * @param xDeadband The deadband for the x-axis.
   * @param yDeadband The deadband for the y-axis.
   * @param zDeadband The deadband for the z-axis.
   *
   * @param xRateLimit The rate limit for the x-axis.
   * @param yRateLimit The rate limit for the y-axis.
   * @param zRateLimit The rate limit for the z-axis.
   */
  public DriverProfile(
      String driverName,
      double xDeadband,
      double yDeadband,
      double zDeadband,
      double xRateLimit,
      double yRateLimit,
      double zRateLimit) {
    this.driverName = driverName;

    this.xDeadband = xDeadband;
    this.yDeadband = yDeadband;
    this.zDeadband = zDeadband;

    this.xRateLimit = xRateLimit;
    this.yRateLimit = yRateLimit;
    this.zRateLimit = zRateLimit;
  }

  /**
   * Returns all of the DriverProfile's deadbands.
   * 
   * @return The deadbands as a double array in the format [x, y, z].
   */
  public double[] getDeadbands() {
    return new double[] {
      getXDeadband(),
      getYDeadband(),
      getZDeadband()
    };
  }

  /**
   * Returns all of the DriverProfile's rate limits.
   *
   * @return The rate limits as a double array in the format [x, y ,z].
   */
  public double[] getRateLimits() {
    return new double[] {
      getXRateLimit(),
      getYRateLimit(),
      getZRateLimit()
    };
  }

  /**
   * Returns the driver name.
   *
   * @return The driver name as a String.
   */
  public String getDriverName() {
    return driverName;
  }

  /**
   * Returns the x-axis deadband.
   *
   * @return The deadband as a double.
   */
  public double getXDeadband() {
    return xDeadband;
  }

  /**
   * Returns the y-axis deadband.
   *
   * @return The deadband as a double.
   */
  public double getYDeadband() {
    return yDeadband;
  }

  /**
   * Returns the z-axis deadband.
   *
   * @return The deadband as a double.
   */
  public double getZDeadband() {
    return zDeadband;
  }

  /**
   * Returns the x-axis rate limit.
   *
   * @return The rate limit as a double.
   */
  public double getXRateLimit() {
    return xRateLimit;
  }

  /**
   * Returns the y-axis rate limit.
   *
   * @return The rate limit as a double.
   */ 
  public double getYRateLimit() {
    return yRateLimit;
  }

  /**
   * Returns the z-axis rate limit.
   *
   * @return The rate limit as a double.
   */
  public double getZRateLimit() {
    return zRateLimit;
  }
}
