// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.HeightSequence;
import frc.robot.commands.OutCoral;
import frc.robot.commands.TakeAlgea;
import frc.robot.commands.TakeCoral;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtake.OuttakeIO;
import frc.robot.subsystems.outtake.OuttakeIOTalonFX;
import frc.robot.util.Enums.Height;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Drive drive;

  // @SuppressWarnings("unused")
  // private final Vision vision;

  private final Elevator elevator;

  private final Outtake outtake;

  // Controller
  private final CommandXboxController m_driver = new CommandXboxController(0);
  private final CommandPS4Controller m_operator = new CommandPS4Controller(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        // vision = new Vision(
        // drive::addVisionMeasurement,
        // new VisionIOLimelight("limelight-sharc", drive::getRotation));

        elevator = new Elevator(new ElevatorIOTalonFX());

        outtake = new Outtake(new OuttakeIOTalonFX());

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        // vision = new Vision(
        // drive::addVisionMeasurement,
        // new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
        // new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));

        elevator = new Elevator(new ElevatorIOSim());

        // isn't simulated
        outtake = new Outtake(new OuttakeIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        // vision = new Vision(drive::addVisionMeasurement, new VisionIO() {
        // }, new VisionIO() {
        // });

        elevator = new Elevator(new ElevatorIO() {});

        outtake = new Outtake(new OuttakeIO() {});

        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -m_driver.getLeftY(),
            () -> -m_driver.getLeftX(),
            () -> -m_driver.getRightX()));

    // Lock to 0° when A button is held
    // controller
    // .a()
    // .whileTrue(
    // DriveCommands.joystickDriveAtAngle(
    // drive,
    // () -> -controller.getLeftY(),
    // () -> -controller.getLeftX(),
    // () -> new Rotation2d()));

    // // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // // Reset gyro to 0° when B button is pressed
    // controller
    // .b()
    // .onTrue(
    // Commands.runOnce(
    // () ->
    // drive.setPose(
    // new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    // drive)
    // .ignoringDisable(true));
    // m_operator.L1().onTrue(new ElevatorSet(elevator, 90)).onFalse(new
    // ElevatorSet(elevator, 0));
    // m_operator
    // .circle()
    // .onTrue(new InstantCommand(() -> RobotState.setMode(OperationMode.AUTO)))
    // .onTrue(new GenerateAuto(elevator));

    // m_operator.square().onTrue(new InstantCommand(() ->
    // RobotState.setMode(OperationMode.HUMAN)));

    m_operator
        .cross()
        .whileTrue(new HeightSequence(outtake, elevator, Height.L1))
        .onFalse(new InstantCommand(() -> elevator.setVoltage(0)));

    m_operator.square().whileTrue(new HeightSequence(outtake, elevator, Height.L2));

    m_operator.circle().whileTrue(new HeightSequence(outtake, elevator, Height.L3));

    m_operator.triangle().whileTrue(new HeightSequence(outtake, elevator, Height.L4));

    m_operator.L1().whileTrue(new TakeCoral(outtake));

    m_operator.R1().whileTrue(new OutCoral(outtake));

    m_driver.rightBumper().whileTrue(new TakeAlgea(outtake));

    m_operator
        .R2()
        .whileTrue(new RunCommand(() -> outtake.setOuttakeVoltage(-5), outtake))
        .onFalse(new InstantCommand(() -> outtake.stop(), outtake));

    m_driver
        .leftTrigger()
        .whileTrue(new RunCommand(() -> outtake.setPivotAngle(0), outtake))
        .onFalse(new InstantCommand(() -> outtake.setPivotVoltage(0), outtake));

    m_driver
        .rightTrigger()
        .whileTrue(new TakeCoral(outtake))
        .onFalse(new InstantCommand(() -> outtake.setPivotVoltage(0), outtake));
    ;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
