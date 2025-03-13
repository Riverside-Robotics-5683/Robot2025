package ravenrobotics.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import ravenrobotics.robot.Constants.DSConstants;
import ravenrobotics.robot.commands.AlignToReefCommand;
import ravenrobotics.robot.commands.DriveCommand;
import ravenrobotics.robot.commands.MoveAuto;
import ravenrobotics.robot.subsystems.climber.ClimberSubsystem;
import ravenrobotics.robot.subsystems.drive.DriveSubsystem;
import ravenrobotics.robot.subsystems.elevator.ElevatorSubsystem;
import ravenrobotics.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import ravenrobotics.robot.subsystems.intake.IntakeSubsystem;
import ravenrobotics.robot.subsystems.vision.VisionSubsystem;

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

    private final Command elevatorTestCommand = ElevatorSubsystem.getInstance()
        .rawElevatorPower(() -> -systemsController.getLeftY());

    private final Command station2L1 = new ParallelCommandGroup(
        new MoveAuto(1.5, 1.5),
        ElevatorSubsystem.getInstance().setElevatorPosition(ElevatorPosition.L1)
    ).andThen(IntakeSubsystem.getInstance().outtakeCoralL1());

    private SendableChooser<Integer> modeChooser = new SendableChooser<
        Integer
    >();

    private SendableChooser<Command> autoChooser;

    public RobotContainer() {
        VisionSubsystem.getInstance();

        // Set the default command for the DriveSubsystem to the drive command.
        DriveSubsystem.getInstance().setDefaultCommand(driveCommand);
        ElevatorSubsystem.getInstance()
            .setDefaultCommand(
                ElevatorSubsystem.getInstance()
                    .rawElevatorPower(() ->
                        MathUtil.applyDeadband(
                            -systemsController.getLeftY(),
                            0.05
                        )
                    )
            );
        IntakeSubsystem.getInstance();
        //ClimberSubsystem.getInstance();

        DriveSubsystem.getInstance().resetHeading().schedule();

        modeChooser.setDefaultOption("Manual", 0);
        modeChooser.addOption("Semi-Auto", 1);

        SmartDashboard.putData("ModeChooser", modeChooser);

        while (!AutoBuilder.isConfigured()) {
            continue;
        }

        addNamedCommands();

        autoChooser = AutoBuilder.buildAutoChooser();

        autoChooser.setDefaultOption(
            "Move Forward Basic",
            new MoveAuto(Units.feetToMeters(4), 2.5)
        );

        autoChooser.addOption("Station 2 L1 (no PathPlanner)", station2L1);

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getTestCommand() {
        systemsController.x().onTrue(new AlignToReefCommand());

        systemsController
            .povUp()
            .onTrue(
                ElevatorSubsystem.getInstance()
                    .sysIdQuasistatic(Direction.kForward)
            );
        systemsController
            .povDown()
            .onTrue(
                ElevatorSubsystem.getInstance()
                    .sysIdQuasistatic(Direction.kReverse)
            );

        systemsController
            .povLeft()
            .onTrue(
                ElevatorSubsystem.getInstance().sysIdDynamic(Direction.kForward)
            );
        systemsController
            .povRight()
            .onTrue(
                ElevatorSubsystem.getInstance().sysIdDynamic(Direction.kReverse)
            );

        return elevatorTestCommand;
    }

    public Command getAutoCommand() {
        return autoChooser.getSelected();
    }

    private void addNamedCommands() {
        NamedCommands.registerCommand(
            "elevatorL1",
            ElevatorSubsystem.getInstance()
                .setElevatorPosition(ElevatorPosition.L1)
        );
        NamedCommands.registerCommand(
            "elevatorL2",
            ElevatorSubsystem.getInstance()
                .setElevatorPosition(ElevatorPosition.L2)
        );
        NamedCommands.registerCommand(
            "elevatorL3",
            ElevatorSubsystem.getInstance()
                .setElevatorPosition(ElevatorPosition.L3)
        );
        NamedCommands.registerCommand(
            "elevatorL4",
            ElevatorSubsystem.getInstance()
                .setElevatorPosition(ElevatorPosition.L4)
        );
        NamedCommands.registerCommand(
            "elevatorFeed",
            ElevatorSubsystem.getInstance()
                .setElevatorPosition(ElevatorPosition.FEED)
        );
        NamedCommands.registerCommand(
            "elevatorClose",
            ElevatorSubsystem.getInstance()
                .setElevatorPosition(ElevatorPosition.CLOSED)
        );

        NamedCommands.registerCommand(
            "intakeCoral",
            IntakeSubsystem.getInstance().intakeCoral()
        );
        NamedCommands.registerCommand(
            "scoreCoral",
            IntakeSubsystem.getInstance().outtakeCoral()
        );
        NamedCommands.registerCommand(
            "scoreCoralL1",
            IntakeSubsystem.getInstance().outtakeCoralL1()
        );
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
    }

    private void configManualBindings() {
        // Driver IMU reset.
        driverController
            .back()
            .onTrue(DriveSubsystem.getInstance().resetHeading());

        // Driver climber up.
        driverController
            .rightTrigger(0.5)
            .whileTrue(ClimberSubsystem.getInstance().setPower(1));
        // Driver climber down.
        driverController
            .leftTrigger(0.5)
            .whileTrue(ClimberSubsystem.getInstance().setPower(-1));

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
                    .setElevatorPosition(ElevatorPosition.FEED)
            );

        systemsController
            .leftTrigger(0.5)
            .onTrue(IntakeSubsystem.getInstance().intakeCoral());
        systemsController
            .rightTrigger(0.5)
            .onTrue(IntakeSubsystem.getInstance().outtakeCoral());
        // systemsController
        //     .leftTrigger(0.5)
        //     .onTrue(IntakeSubsystem.getInstance().setIntake(1));
        // systemsController
        //     .rightTrigger(0.5)
        //     .onTrue(IntakeSubsystem.getInstance().setIntake(-1));
    }
}
