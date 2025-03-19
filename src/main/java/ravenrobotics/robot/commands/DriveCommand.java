package ravenrobotics.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import ravenrobotics.robot.Constants.KinematicsConstants;
import ravenrobotics.robot.subsystems.drive.DriveSubsystem;
import ravenrobotics.robot.subsystems.elevator.ElevatorSubsystem;
import ravenrobotics.robot.util.DriverProfile;

/**
 * Command for driving the robot using a controller.
 */
public class DriveCommand extends Command {

    private final DriveSubsystem driveSubsystem; // Subsystem for sending SwerveModuleStates.
    private DriverProfile profile; // The driver profile that contains deadbands and rate limits.

    private final DoubleSupplier xSupplier; // Supplier for getting the x-axis value.
    private final DoubleSupplier ySupplier; // Supplier for getting the y-axis value.
    private final DoubleSupplier zSupplier; // Supplier for getting the z-axis value.

    private SlewRateLimiter xLimiter; // SlewRateLimiter for limiting x-axis acceleration.
    private SlewRateLimiter yLimiter; // SlewRateLimiter for limiting y-axis acceleration.
    private SlewRateLimiter zLimiter; // SlewRateLimiter for limiting z-axis acceleration.

    private double[] deadbands; // Deadbands to give the SlewRateLimiters.

    /**
     * Command for driving the robot using a controller.
     *
     * @param initialProfile The initial DriverProfile to use for deadbands and rate
     *                       limits.
     * @param xAxis          The DoubleSupplier for the x-axis.
     * @param yAxis          The DoubleSupplier for the y-axis.
     * @param zAxis          The DoubleSupplier for the z-axis.
     */
    public DriveCommand(
        DriverProfile initialProfile,
        DoubleSupplier xAxis,
        DoubleSupplier yAxis,
        DoubleSupplier zAxis
    ) {
        this.profile = initialProfile;

        this.xSupplier = xAxis;
        this.ySupplier = yAxis;
        this.zSupplier = zAxis;

        double[] rateLimits = profile.getRateLimits(); // Get the rate limits from the driver profile.
        deadbands = profile.getDeadbands(); // Get the deadbands from the driver profile.

        // Create the SlewRateLimiters for each axis using the deadbands.
        this.xLimiter = new SlewRateLimiter(rateLimits[0], -rateLimits[0], 0.0);
        this.yLimiter = new SlewRateLimiter(rateLimits[1], -rateLimits[1], 0.0);
        this.zLimiter = new SlewRateLimiter(rateLimits[2], -rateLimits[2], 0.0);

        this.driveSubsystem = DriveSubsystem.getInstance(); // Get the active DriveSubsystem instance.
        addRequirements(this.driveSubsystem); // Add the DriveSubsystem as a requirement, so that no other commands access
        // the DriveSubsystem while this command is running.
    }

    @Override
    public void execute() {
        double xSpeed, ySpeed, zSpeed; // Initialize the temporary variables for the current speed.

        double currentMaxSpeed;

        if (ElevatorSubsystem.getInstance().isElevatorPastLimit()) {
            currentMaxSpeed = KinematicsConstants.ELEVATOR_MODULE_SPEED;
        } else {
            currentMaxSpeed = KinematicsConstants.MAX_MODULE_SPEED;
        }

        xSpeed = xLimiter.calculate(
            // Apply the deadband for the x-axis, then rate limit the value. Finally,
            // multiply by the max module speed to convert to m/s.
            MathUtil.applyDeadband(xSupplier.getAsDouble(), deadbands[0]) *
            currentMaxSpeed
        );
        ySpeed = yLimiter.calculate(
            // Apply the deadband for the y-axis, then rate limit the value. Finally,
            // multiply by the max module speed to convert to m/s.
            MathUtil.applyDeadband(ySupplier.getAsDouble(), deadbands[1]) *
            currentMaxSpeed
        );
        zSpeed = zLimiter.calculate(
            // Apply the deadband for the z-axis, then rate limit the value. Finally,
            // multiply by the max module speed to convert to m/s.
            MathUtil.applyDeadband(zSupplier.getAsDouble(), deadbands[2]) *
            currentMaxSpeed
        );

        if (xSpeed == 0) {
            xLimiter.reset(0);
        }

        if (ySpeed == 0) {
            yLimiter.reset(0);
        }

        if (zSpeed == 0) {
            zLimiter.reset(0);
        }

        ChassisSpeeds driveSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed,
            ySpeed,
            zSpeed,
            driveSubsystem.getFusedHeading()
        ); // Convert the speeds from the joysticks to a field-relative ChassisSpeeds,
        // using the current heading.

        driveSubsystem.run(driveSpeeds); // Send the speeds to the DriveSubsystem to be sent to the swerve modules.
    }

    /**
     * Update the active DriverProfile with a new one.
     *
     * @param newProfile The new profile to use.
     */
    public void updateDriverProfile(DriverProfile newProfile) {
        profile = newProfile;

        double[] rateLimits = profile.getRateLimits(); // Get the rate limits from the new profile.

        // Re-initialize all of the SlewRateLimiters with the new rate limits.
        xLimiter = new SlewRateLimiter(rateLimits[0]);
        yLimiter = new SlewRateLimiter(rateLimits[1]);
        zLimiter = new SlewRateLimiter(rateLimits[2]);

        deadbands = profile.getDeadbands(); // Set the new deadbands from the new profile.
    }
}
