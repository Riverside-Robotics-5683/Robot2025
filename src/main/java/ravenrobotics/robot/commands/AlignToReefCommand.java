// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ravenrobotics.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;
import ravenrobotics.robot.subsystems.drive.DriveSubsystem;
import ravenrobotics.robot.subsystems.vision.VisionSubsystem;

/**
 * Command for aligning the robot to a reef (AprilTag) using vision-based positioning.
 * Uses PID controllers to control the robot's position and orientation relative to the target.
 */
public class AlignToReefCommand extends Command {

    /** Subsystem for vision processing and AprilTag detection */
    private VisionSubsystem visionSubsystem;

    /** Subsystem for controlling robot movement */
    private DriveSubsystem driveSubsystem;

    /** X component of the rotated velocity vector */
    private double rotatedVelocityX;

    /** Y component of the rotated velocity vector */
    private double rotatedVelocityY;

    /** Limits acceleration in the forward direction to prevent jerky movement */
    private SlewRateLimiter forwardsAccelerationLimit = new SlewRateLimiter(
        0.75
    );

    /** Limits acceleration in the left direction to prevent jerky movement */
    private SlewRateLimiter leftAccelerationLimit = new SlewRateLimiter(0.75);

    /** Limits rotational acceleration to prevent jerky rotation */
    private SlewRateLimiter thetaAceelerationLimiter = new SlewRateLimiter(
        0.75
    );

    /** ID of the AprilTag that the robot should align to. -1 indicates not yet set. */
    private double priorityTagId = -1;

    /** PID controller for X-axis positioning (forward/backward) */
    private PIDController xController = new PIDController(10.0, 0.0, 0.0);

    /** PID controller for Y-axis positioning (left/right) */
    private PIDController yController = new PIDController(10.0, 0.0, 0.0);

    /** PID controller for Z-axis rotation (heading) */
    private PIDController zController = new PIDController(1.5, 0.0, 0.0);

    /**
     * Creates a new AlignToReefCommand for aligning to an april tag on the reef.
     */
    public AlignToReefCommand() {
        this.visionSubsystem = VisionSubsystem.getInstance();
        this.driveSubsystem = DriveSubsystem.getInstance();
        // Enable continuous input for heading control to handle the -180 to 180 degree wrap-around
        zController.enableContinuousInput(-180, 180);

        // Register the drivetrain as a requirement for this command
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        // Reset acceleration limiters to start from zero
        leftAccelerationLimit.reset(0);
        forwardsAccelerationLimit.reset(0);

        // Set tolerances for each PID controller to determine when target is reached
        xController.setTolerance(0.02);
        yController.setTolerance(0.02);
        zController.setTolerance(0.5);

        // Reset accumulated errors in PID controllers
        xController.reset();
        yController.reset();
        zController.reset();
    }

    /**
     * Rotates a translation vector by a specified rotation
     *
     * @param translationToRotate The translation vector to rotate
     * @param rotation The rotation to apply
     * @return The rotated translation vector
     */
    public static Translation2d rotateTranslation(
        Translation2d translationToRotate,
        Rotation2d rotation
    ) {
        return translationToRotate.rotateBy(rotation);
    }

    @Override
    public void execute() {
        // Get AprilTag detection results from the left camera
        var resultsOptional = visionSubsystem.getLeftAprilTags();

        // Exit if no vision results are available
        if (resultsOptional.isEmpty()) return;

        var targetList = resultsOptional.get();

        // Exit if no targets were detected
        if (targetList.isEmpty()) return;

        // Use the largest target.
        PhotonTrackedTarget biggestTarget = targetList.get(0);

        // Set the priority tag ID on first detection if not already set
        if (priorityTagId == -1) {
            priorityTagId = biggestTarget.getFiducialId();
        }

        // Exit if this isn't the tag we're focusing on
        if (biggestTarget.getFiducialId() != priorityTagId) return;

        // Get the pose of the detected AprilTag from the field layout
        var tagPoseOptional = visionSubsystem
            .getFieldLayout()
            .getTagPose(biggestTarget.getFiducialId());

        // Exit if tag pose information is unavailable
        if (tagPoseOptional.isEmpty()) return;

        // Calculate the angle of the tag, rotated 90 degrees (π/2 radians)
        Rotation2d tagAngle = tagPoseOptional
            .get()
            .getRotation()
            .toRotation2d()
            .rotateBy(new Rotation2d(Math.PI / 2.0));

        // Calculate X velocity using PID controller (negative for correct direction)
        double velX = -xController.calculate(
            biggestTarget.getBestCameraToTarget().getX(),
            0.35 // Target distance in X direction
        );

        // Calculate Y velocity using PID controller (negative for correct direction)
        double velY = -yController.calculate(
            biggestTarget.getBestCameraToTarget().getY(),
            0.05 // Target offset in Y direction
        );

        // Log current and target values for debugging
        Logger.recordOutput(
            "AlignmentCommand/currentTheta",
            driveSubsystem.getRawHeading().getDegrees()
        );
        Logger.recordOutput(
            "AlignmentCommand/targetTheta",
            tagAngle.getDegrees()
        );
        Logger.recordOutput(
            "AlignmentCommand/currentX",
            biggestTarget.getBestCameraToTarget().getX()
        );
        Logger.recordOutput(
            "AlignmentCommand/currentY",
            biggestTarget.getBestCameraToTarget().getY()
        );

        // Calculate rotation velocity using PID controller
        double velTheta = -zController.calculate(
            driveSubsystem.getRawHeading().getDegrees(),
            tagAngle.getDegrees()
        );

        // If already close to the target angle, stop rotating
        if (
            driveSubsystem.getRawHeading().getDegrees() <
                tagAngle.getDegrees() + 0.5 &&
            driveSubsystem.getRawHeading().getDegrees() >
            tagAngle.getDegrees() - 0.5
        ) {
            velTheta = 0;
        }

        // Apply acceleration limits to the raw PID outputs
        Translation2d PIDSpeed = new Translation2d(
            forwardsAccelerationLimit.calculate(velX),
            leftAccelerationLimit.calculate(velY)
        );

        // Rotate the speed vector to align with the tag's orientation
        Translation2d rotatedPIDSpeeds = rotateTranslation(PIDSpeed, tagAngle);

        // Store the rotated velocity components for later use
        rotatedVelocityX = rotatedPIDSpeeds.getX();
        rotatedVelocityY = rotatedPIDSpeeds.getY();

        // Command the drivetrain to move at the calculated speeds
        driveSubsystem.run(
            new ChassisSpeeds(
                rotatedVelocityX,
                rotatedVelocityY,
                thetaAceelerationLimiter.calculate(velTheta)
            )
        );
    }

    @Override
    public void end(boolean interrupted) {
        // Stop all robot movement
        driveSubsystem.run(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        // Log whether each axis has reached its target for debugging
        Logger.recordOutput("AlignmentCommand/atX", xController.atSetpoint());
        Logger.recordOutput("AlignmentCommand/atY", yController.atSetpoint());
        Logger.recordOutput("AlignmentCommand/atZ", zController.atSetpoint());

        // Command is finished when all three axes are at their setpoints
        return (
            (xController.atSetpoint() &&
                yController.atSetpoint() &&
                zController.atSetpoint())
        );
    }
}
