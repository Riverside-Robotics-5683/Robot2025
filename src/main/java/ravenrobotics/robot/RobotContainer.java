package ravenrobotics.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import ravenrobotics.robot.Constants.DSConstants;
import ravenrobotics.robot.commands.DriveCommand;
import ravenrobotics.robot.subsystems.drive.DriveSubsystem;

public class RobotContainer {
  // Driver controller.
  private final CommandXboxController driverController = new CommandXboxController(DSConstants.DRIVE_CONTROLLER);

  // Drive command.
  private final DriveCommand driveCommand = new DriveCommand(
      DSConstants.DEFAULT_PROFILE,
      () -> -driverController.getLeftY(), // Forward/backward.
      () -> -driverController.getLeftX(), // Left/right.
      () -> -driverController.getRightX()); // Rotation.

  public RobotContainer() {
    DriveSubsystem.getInstance().setDefaultCommand(driveCommand); // Set the default subsystem command for the DriveSubsystem.
  }
}
