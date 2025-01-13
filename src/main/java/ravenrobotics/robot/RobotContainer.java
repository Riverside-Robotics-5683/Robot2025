package ravenrobotics.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import ravenrobotics.robot.Constants.DSConstants;
import ravenrobotics.robot.commands.DriveCommand;
import ravenrobotics.robot.subsystems.drive.DriveSubsystem;

public class RobotContainer {
  private final CommandXboxController driverController = new CommandXboxController(DSConstants.DRIVE_CONTROLLER);

  private final DriveCommand driveCommand = new DriveCommand(
      DSConstants.DEFAULT_PROFILE,
      () -> -driverController.getLeftY(),
      () -> -driverController.getLeftX(),
      () -> -driverController.getRightX());

  public RobotContainer() {
    DriveSubsystem.getInstance().setDefaultCommand(driveCommand);
  }
}
