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
      () -> driverController.getLeftY(), // Forward/backward.
      () -> driverController.getLeftX(), // Left/right.
      () -> driverController.getRightX()); // Rotation.

  public RobotContainer() {
    // Set the default command for the DriveSubsystem to the drive command.
    DriveSubsystem.getInstance().setDefaultCommand(driveCommand);

    // Configure controller bindings.
    configBindings();
  }

  private void configBindings() {
    driverController.back().onTrue(DriveSubsystem.getInstance().resetHeading());
  }
}
