package ravenrobotics.robot.subsystems.scheduler;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import ravenrobotics.robot.subsystems.elevator.ElevatorSubsystem;
import ravenrobotics.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import ravenrobotics.robot.subsystems.vision.VisionSubsystem;

public class SchedulerSubsystem extends SubsystemBase {

    private Command runningCommand;

    private boolean isRunning = false;

    private static SchedulerSubsystem instance;

    public static SchedulerSubsystem getInstance() {
        if (instance == null) {
            instance = new SchedulerSubsystem();
        }

        return instance;
    }

    @Override
    public void periodic() {
        if (!isRunning) {
            return;
        }

        PhotonPipelineResult visibleTag = VisionSubsystem.getInstance()
            .getIntakeAprilTags();

        int tagId;

        if (visibleTag.getBestTarget() != null) {
            tagId = visibleTag.getBestTarget().fiducialId;
        } else {
            tagId = -1;
        }

        switch (tagId) {
            case 1, 2, 12, 13:
                changeRunningCommand(
                    ElevatorSubsystem.getInstance()
                        .setElevatorPosition(ElevatorPosition.FEED)
                );
                break;
            case 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22:
                setElevatorForScoring();
                break;
            default:
                changeRunningCommand(
                    ElevatorSubsystem.getInstance()
                        .setElevatorPosition(ElevatorPosition.CLOSED)
                );
                break;
        }
    }

    private void setElevatorForScoring() {
        var coral = VisionSubsystem.getInstance().getDetectedCoral();

        if (coral == null) {
            changeRunningCommand(
                ElevatorSubsystem.getInstance()
                    .setElevatorPosition(ElevatorPosition.L1)
            );
            return;
        }

        var coralTargets = coral.getTargets();

        int[] filledLevels = new int[3];

        for (PhotonTrackedTarget current : coralTargets) {
            double height = current
                .getBestCameraToTarget()
                .getMeasureZ()
                .in(Units.Meter);

            if (height >= 0.81 && height <= 1.21) {
                filledLevels[0] = 1;
            } else {
                filledLevels[0] = 0;
            }

            if (height >= 1.21 && height <= 1.83) {
                filledLevels[1] = 1;
            } else {
                filledLevels[1] = 0;
            }

            if (height >= 1.8) {
                filledLevels[2] = 1;
            } else {
                filledLevels[2] = 0;
            }
        }

        if (filledLevels[2] == 0) {
            changeRunningCommand(
                ElevatorSubsystem.getInstance()
                    .setElevatorPosition(ElevatorPosition.L4)
            );
        } else if (filledLevels[1] == 0) {
            changeRunningCommand(
                ElevatorSubsystem.getInstance()
                    .setElevatorPosition(ElevatorPosition.L3)
            );
        } else if (filledLevels[0] == 0) {
            changeRunningCommand(
                ElevatorSubsystem.getInstance()
                    .setElevatorPosition(ElevatorPosition.L2)
            );
        } else {
            changeRunningCommand(
                ElevatorSubsystem.getInstance()
                    .setElevatorPosition(ElevatorPosition.L1)
            );
        }
    }

    private void changeRunningCommand(Command newCommand) {
        runningCommand.cancel();

        runningCommand = newCommand;

        runningCommand.schedule();
    }

    public Command cancelRunningCommmand() {
        return this.runOnce(() -> runningCommand.cancel());
    }

    public void disableScheduling() {
        isRunning = false;
    }

    public void enableScheduling() {
        isRunning = true;
    }
}
