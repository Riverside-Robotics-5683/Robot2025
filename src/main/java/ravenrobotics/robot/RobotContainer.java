package ravenrobotics.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import ravenrobotics.robot.Constants.DSConstants;
import ravenrobotics.robot.commands.DriveCommand;
import ravenrobotics.robot.subsystems.climber.ClimberSubsystem;
import ravenrobotics.robot.subsystems.drive.DriveSubsystem;
import ravenrobotics.robot.subsystems.elevator.ElevatorSubsystem;
import ravenrobotics.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import ravenrobotics.robot.subsystems.intake.IntakeSubsystem;
import ravenrobotics.robot.subsystems.intake.IntakeSubsystem.IntakeAngle;
import ravenrobotics.robot.subsystems.scheduler.SchedulerSubsystem;

public class RobotContainer {

    // Driver controller.
    private final CommandXboxController driverController =
        new CommandXboxController(DSConstants.DRIVE_CONTROLLER);

    // Systems controller for manual driving.
    private final CommandXboxController systemsController =
        new CommandXboxController(DSConstants.SYSTEMS_CONTROLLER);

    // Drive command.
    private final DriveCommand driveCommand = new DriveCommand(
        DSConstants.DEFAULT_PROFILE,
        () -> -driverController.getLeftY(), // Forward/backward.
        () -> -driverController.getLeftX(), // Left/right.
        () -> -driverController.getRightX()
    ); // Rotation.

    private SendableChooser<Integer> modeChooser = new SendableChooser<
        Integer
    >();

    public RobotContainer() {
        // Set the default command for the DriveSubsystem to the drive command.
        DriveSubsystem.getInstance().setDefaultCommand(driveCommand);
        ElevatorSubsystem.getInstance();
        IntakeSubsystem.getInstance();

        modeChooser.setDefaultOption("Manual", 0);
        modeChooser.addOption("Semi-Auto", 1);

        SmartDashboard.putData("ModeChooser", modeChooser);
    }

    public void teleopSetup() {
        int mode = modeChooser.getSelected();

        switch (mode) {
            case 0:
                configManualBindings();
                break;
            case 1:
                configBindings();
                break;
            default:
                throw new IllegalStateException("How did we get here?");
        }
    }

    private void configBindings() {
        driverController
            .back()
            .onTrue(DriveSubsystem.getInstance().resetHeading());

        driverController
            .b()
            .onTrue(SchedulerSubsystem.getInstance().cancelRunningCommmand());
    }

    private void configManualBindings() {
        // Driver IMU reset.
        driverController
            .back()
            .onTrue(DriveSubsystem.getInstance().resetHeading());

        // Driver climber up.
        driverController
            .leftTrigger(0.5)
            .onTrue(ClimberSubsystem.getInstance().setPower(0.25));
        // Driver climber down.
        driverController
            .rightTrigger(0.5)
            .onTrue(ClimberSubsystem.getInstance().setPower(-0.25));

        // Driver set climber hold.
        driverController.b().onTrue(ClimberSubsystem.getInstance().setHold());

        // Driver cancel hold.
        driverController
            .x()
            .onTrue(ClimberSubsystem.getInstance().cancelHold());

        // Systems set elevator to L4.
        systemsController
            .y()
            .onTrue(
                ElevatorSubsystem.getInstance()
                    .setElevatorPosition(ElevatorPosition.L4)
            );
        // Systems set elevator to L3.
        systemsController
            .x()
            .onTrue(
                ElevatorSubsystem.getInstance()
                    .setElevatorPosition(ElevatorPosition.L3)
            );
        // Systems set elevator to L2.
        systemsController
            .b()
            .onTrue(
                ElevatorSubsystem.getInstance()
                    .setElevatorPosition(ElevatorPosition.L2)
            );
        // Systems set elevator to L1.
        systemsController
            .a()
            .onTrue(
                ElevatorSubsystem.getInstance()
                    .setElevatorPosition(ElevatorPosition.L1)
            );
        systemsController
            .rightBumper()
            .onTrue(
                ElevatorSubsystem.getInstance()
                    .setElevatorPosition(ElevatorPosition.CLOSED)
            );
        systemsController
            .leftBumper()
            .onTrue(
                ElevatorSubsystem.getInstance()
                    .setElevatorPosition(ElevatorPosition.L1)
            );

        systemsController
            .povUp()
            .onTrue(
                IntakeSubsystem.getInstance().setIntakeAngle(IntakeAngle.L4)
            );
        systemsController
            .povLeft()
            .onTrue(
                IntakeSubsystem.getInstance().setIntakeAngle(IntakeAngle.L3)
            );
        systemsController
            .povRight()
            .onTrue(
                IntakeSubsystem.getInstance().setIntakeAngle(IntakeAngle.L2)
            );
        systemsController
            .povDown()
            .onTrue(
                IntakeSubsystem.getInstance().setIntakeAngle(IntakeAngle.L1)
            );
        systemsController
            .back()
            .onTrue(
                IntakeSubsystem.getInstance()
                    .setIntakeAngle(IntakeAngle.DEFAULT)
            );

        // systemsController
        //     .leftTrigger(0.5)
        //     .onTrue(IntakeSubsystem.getInstance().setRollers(0.05));
        // systemsController
        //     .rightTrigger(0.5)
        //     .onTrue(IntakeSubsystem.getInstance().setRollers(-0.05));
        systemsController
            .leftTrigger(0.5)
            .onTrue(IntakeSubsystem.getInstance().setIntake(1));
        systemsController
            .rightTrigger(0.5)
            .onTrue(IntakeSubsystem.getInstance().setIntake(-1));
    }
}
