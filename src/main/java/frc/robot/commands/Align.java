// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.state.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.Enums.OperationMode;
import frc.robot.util.TargetSelector;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Align extends Command {

  private static final double ANGLE_KP = 4;
  private static final double ANGLE_KD = 0.2;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double XY_MAX_VELOCITY = 3;
  private static final double XY_MAX_ACCELERATION = 4;

  private Drive drive;
  private Pose2d targetPose = AllianceFlipUtil.apply(Constants.FieldConstants.sidePoses[0]);
  private Pose2d currPose = new Pose2d();
  private Pose2d delta = new Pose2d();
  private TargetSelector targetSelector;

  ProfiledPIDController angleController =
      new ProfiledPIDController(
          ANGLE_KP,
          0.0,
          ANGLE_KD,
          new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));

  ProfiledPIDController xController =
      new ProfiledPIDController(
          0.325, 0.0, 0, new TrapezoidProfile.Constraints(XY_MAX_VELOCITY, XY_MAX_ACCELERATION));

  ProfiledPIDController yController =
      new ProfiledPIDController(
          0.325, 0.0, 0, new TrapezoidProfile.Constraints(XY_MAX_VELOCITY, XY_MAX_ACCELERATION));

  public Align(Drive drive, TargetSelector targetSelector) {
    this.drive = drive;
    this.targetSelector = targetSelector;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    currPose = drive.getPose();
    targetPose = targetSelector.getClosestSidePose(RobotState.getSelectedPosition());
    delta =
        new Pose2d(
            targetPose.getX() - currPose.getX(),
            targetPose.getY() - currPose.getY(),
            targetPose.getRotation().minus(currPose.getRotation()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currPose = drive.getPose();
    targetPose = targetSelector.getClosestSidePose(RobotState.getSelectedPosition());
    Logger.recordOutput("TargetingSystem/Going to", targetPose);
    // Get linear velocity
    Translation2d linearVelocity =
        new Translation2d(
            xController.calculate(currPose.getX(), targetPose.getX()),
            yController.calculate(currPose.getY(), targetPose.getY()));
    Logger.recordOutput("TargetingSystem/Auto Align Calculated Velocities", linearVelocity);
    // Calculate angular speed
    double omega =
        angleController.calculate(
            drive.getRotation().getRadians(), targetPose.getRotation().getRadians());

    // Convert to field relative speeds & send command
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            omega);
    Logger.recordOutput("TargetingSystem/Auto Align Chassis Speeds", speeds);

    delta =
        new Pose2d(
            targetPose.getX() - currPose.getX(),
            targetPose.getY() - currPose.getY(),
            targetPose.getRotation().minus(currPose.getRotation()));

    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(delta.getX()) < 0.1
            && Math.abs(delta.getY()) < 0.1
            && Math.abs(delta.getRotation().getDegrees()) < 5)
        || RobotState.getMode() == OperationMode.HUMAN;
  }
}
