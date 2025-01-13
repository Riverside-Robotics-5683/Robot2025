package ravenrobotics.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import ravenrobotics.robot.Constants.KinematicsConstants;
import ravenrobotics.robot.subsystems.drive.DriveSubsystem;
import ravenrobotics.robot.util.DriverProfile;

public class DriveCommand extends Command {
  private final DriveSubsystem driveSubsystem;
  private DriverProfile profile;

  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier zSupplier;

  private SlewRateLimiter xLimiter;
  private SlewRateLimiter yLimiter;
  private SlewRateLimiter zLimiter;

  private double[] deadbands;

  public DriveCommand(DriverProfile initialProfile, DoubleSupplier xAxis, DoubleSupplier yAxis, DoubleSupplier zAxis) {
    this.profile = initialProfile;

    this.xSupplier = xAxis;
    this.ySupplier = yAxis;
    this.zSupplier = zAxis;

    double[] rateLimits = profile.getRateLimits();
    deadbands = profile.getDeadbands();

    this.xLimiter = new SlewRateLimiter(rateLimits[0]);
    this.yLimiter = new SlewRateLimiter(rateLimits[1]);
    this.zLimiter = new SlewRateLimiter(rateLimits[2]);

    this.driveSubsystem = DriveSubsystem.getInstance();
    addRequirements(this.driveSubsystem);
  }

  @Override
  public void execute() {
    double xSpeed, ySpeed, zSpeed;

    xSpeed = xLimiter.calculate(
        MathUtil.applyDeadband(xSupplier.getAsDouble(), deadbands[0]) * KinematicsConstants.MAX_MODULE_SPEED);
    ySpeed = yLimiter.calculate(
        MathUtil.applyDeadband(ySupplier.getAsDouble(), deadbands[1]) * KinematicsConstants.MAX_MODULE_SPEED);
    zSpeed = zLimiter.calculate(
        MathUtil.applyDeadband(zSupplier.getAsDouble(), deadbands[2]) * KinematicsConstants.MAX_MODULE_SPEED);

    ChassisSpeeds driveSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(xSpeed, ySpeed, zSpeed),
        driveSubsystem.getHeading());

    driveSubsystem.run(driveSpeeds);
  }

  public void updateDriverProfile(DriverProfile newProfile) {
    profile = newProfile;

    double[] rateLimits = profile.getRateLimits();

    xLimiter = new SlewRateLimiter(rateLimits[0]);
    yLimiter = new SlewRateLimiter(rateLimits[1]);
    zLimiter = new SlewRateLimiter(rateLimits[2]);

    deadbands = profile.getDeadbands();
  }
}
