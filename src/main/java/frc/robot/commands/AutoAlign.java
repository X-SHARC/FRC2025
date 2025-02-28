// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.state.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.TargetSelector;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {

  private Drive drive;
  private Pose2d currentPose = new Pose2d();
  private Pose2d targetPose = new Pose2d();

  // Constants
  private static final double ANGLE_KP = 4;
  private static final double ANGLE_KD = 0.2;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double XY_KP = 0.1; // 0.325
  private static final double XY_KD = 0.1;
  private static final double XY_MAX_VELOCITY = 4;
  private static final double XY_MAX_ACCELERATION = 4;
  private static final double ANGLE_TOLERANCE = Math.toRadians(5); // rad
  private static final double XY_TOLERANCE = 0.05; // m

  // Controllers
  ProfiledPIDController angleController = new ProfiledPIDController(
      ANGLE_KP,
      0.0,
      ANGLE_KD,
      new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));

  PIDController xController = new PIDController(
      XY_KP,
      0.0,
      XY_KD);

  PIDController yController = new PIDController(
      XY_KP,
      0.0,
      XY_KD);

  /** Creates a new AutoAlign. */
  public AutoAlign(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    currentPose = drive.getPose();
    targetPose = TargetSelector.getNearestBranch(RobotState.getSelectedPosition());

    angleController.reset(currentPose.getRotation().getRadians());

    xController.setTolerance(XY_TOLERANCE);
    yController.setTolerance(XY_TOLERANCE);
    angleController.setTolerance(ANGLE_TOLERANCE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPose = drive.getPose();
    targetPose = AllianceFlipUtil.apply(TargetSelector.getNearestBranch(RobotState.getSelectedPosition()));

    Logger.recordOutput("AutoAlign/TargetPose", targetPose);

    Translation2d linearVelocity = new Translation2d(
        xController.calculate(currentPose.getX(), targetPose.getX()),
        yController.calculate(currentPose.getY(), targetPose.getY()));

    double omega = angleController.calculate(
        drive.getRotation().getRadians(), targetPose.getRotation().getRadians());

    ChassisSpeeds speeds = new ChassisSpeeds(
        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
        omega * drive.getMaxAngularSpeedRadPerSec());

    drive.runVelocity(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (xController.atSetpoint() && yController.atSetpoint() && angleController.atGoal())
        || !RobotState.isAuto();
  }
}
