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
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutonomousCommands;
import frc.robot.commands.BaseCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPose;
import frc.robot.generated.TunerConstants;
import frc.robot.state.RobotState;
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
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtake.OuttakeIO;
import frc.robot.subsystems.outtake.OuttakeIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.Enums.Height;
import frc.robot.util.Enums.OperationMode;
import frc.robot.util.Enums.Position;
import frc.robot.util.TargetSelector;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // Subsystems
    private final Drive drive;

    @SuppressWarnings("unused")
    private final Vision vision;

    private final Elevator elevator;

    private final Outtake outtake;

    @SuppressWarnings("unused")
    private final Leds leds = Leds.getInstance();

    // Controller
    private final CommandXboxController m_driver = new CommandXboxController(0);

    private final CommandPS4Controller m_operator = new CommandPS4Controller(1);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));

                vision = new Vision(
                        drive::addVisionMeasurement,
                        new VisionIOLimelight("limelight-sharc", drive::getRotation));

                elevator = new Elevator(new ElevatorIOTalonFX());

                outtake = new Outtake(new OuttakeIOTalonFX());

                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight));

                vision = new Vision(drive::addVisionMeasurement);

                elevator = new Elevator(new ElevatorIOSim());

                // isn't simulated
                outtake = new Outtake(new OuttakeIO() {
                });
                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        });

                vision = new Vision(drive::addVisionMeasurement, new VisionIO() {
                }, new VisionIO() {
                });

                elevator = new Elevator(new ElevatorIO() {
                });

                outtake = new Outtake(new OuttakeIO() {
                });

                break;
        }

        NamedCommands.registerCommand("CoralIn", BaseCommands.intakeCoral(outtake));
        NamedCommands.registerCommand(
                "CoralOut",
                BaseCommands.manipulateObject(outtake, 10, 0, () -> !outtake.isBeamBreakTriggered()));
        NamedCommands.registerCommand("ElevatorHome", BaseCommands.homeElevator(elevator, outtake));
        NamedCommands.registerCommand(
                "ElevatorL2", AutonomousCommands.autonomousHeight(elevator, outtake, Height.L2));
        NamedCommands.registerCommand(
                "ElevatorL3", AutonomousCommands.autonomousHeight(elevator, outtake, Height.L3));
        NamedCommands.registerCommand(
                "ElevatorL4", AutonomousCommands.autonomousHeight(elevator, outtake, Height.L4));

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
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        doubleDriverConfig();
        // singleDriverConfig();
    }

    @SuppressWarnings("unused")
    private void doubleDriverConfig() {

        // Default command, normal field-relative drive
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> -m_driver.getLeftY(),
                        () -> -m_driver.getLeftX(),
                        () -> -m_driver.getRightX()));

        m_driver
                .leftBumper()
                .whileTrue(
                        Commands.sequence(
                                new InstantCommand(() -> RobotState.setMode(OperationMode.AUTO)),
                                new DriveToPose(
                                        drive,
                                        () -> TargetSelector.getNearestBranch(RobotState.getSelectedPosition()))))
                .onFalse(new InstantCommand(() -> RobotState.setMode(OperationMode.HUMAN)));

        m_driver.rightBumper().whileTrue(BaseCommands.outCoral(outtake));

        m_driver.rightTrigger().whileTrue(BaseCommands.autoElevator(elevator));

        m_driver.start().onTrue(new InstantCommand(() -> drive.resetYaw(), drive));

        // Set Selected Auto Align Position
        m_operator
                .povUp()
                .onTrue(
                        new InstantCommand(
                                () -> {
                                    RobotState.setSelectedPosition(Position.MIDDLE);
                                    RobotState.setSelectedElevatorHeight(Height.ALGEA_HIGH);
                                }));

        m_operator
                .povDown()
                .onTrue(
                        new InstantCommand(
                                () -> {
                                    RobotState.setSelectedPosition(Position.MIDDLE);
                                    RobotState.setSelectedElevatorHeight(Height.ALGEA_LOW);
                                }));

        m_operator
                .povLeft()
                .onTrue(new InstantCommand(() -> RobotState.setSelectedPosition(Position.LEFT)));

        m_operator
                .povRight()
                .onTrue(new InstantCommand(() -> RobotState.setSelectedPosition(Position.RIGHT)));

        // Set Selected Elevator Height
        m_operator.cross().whileTrue(BaseCommands.homeElevator(elevator, outtake));

        m_operator
                .square()
                .onTrue(new InstantCommand(() -> RobotState.setSelectedElevatorHeight(Height.L2)));

        m_operator
                .circle()
                .onTrue(new InstantCommand(() -> RobotState.setSelectedElevatorHeight(Height.L3)));

        m_operator
                .triangle()
                .onTrue(new InstantCommand(() -> RobotState.setSelectedElevatorHeight(Height.L4)));

        // Intake/Outtake Object
        m_operator.L1().whileTrue(BaseCommands.intakeCoral(outtake));

        m_operator.R1().whileTrue(BaseCommands.outAlgea(outtake));

        m_operator
                .R2()
                .whileTrue(new RunCommand(() -> outtake.setOuttakeVoltage(3), outtake))
                .onFalse(new InstantCommand(() -> outtake.stop(), outtake));

        m_operator.touchpad().whileTrue(BaseCommands.intakeAlgea(outtake));

        // Reset Elevator
        m_operator.options().onTrue(new InstantCommand(() -> elevator.resetEncoders(), elevator));
    }

    @SuppressWarnings("unused")
    private void singleDriverConfig() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> -m_operator.getLeftY(),
                        () -> -m_operator.getLeftX(),
                        () -> -m_operator.getRightX()));

        // Reset the gyro and encoders

        m_operator.L3().onTrue(new InstantCommand(() -> elevator.resetEncoders(), elevator));

        m_operator.options().onTrue(new InstantCommand(() -> drive.resetYaw(), drive));

        // Intake/ outtake control

        m_operator.L1().whileTrue(BaseCommands.intakeCoral(outtake));

        m_operator.R1().whileTrue(BaseCommands.outCoral(outtake));

        m_operator.touchpad().whileTrue(BaseCommands.intakeAlgea(outtake));

        m_operator.R2().whileTrue(BaseCommands.outAlgea(outtake));

        // Set the elevator height based on the cross, circle, and triangle buttons
        m_operator
                .cross()
                .whileTrue(BaseCommands.homeElevator(elevator, outtake))
                .onFalse(new InstantCommand(() -> elevator.stop(), elevator));

        m_operator
                .square()
                .whileTrue(BaseCommands.setHeight(elevator, outtake, Height.L2))
                .onFalse(BaseCommands.setPivot(outtake, 0));

        m_operator
                .circle()
                .whileTrue(BaseCommands.setHeight(elevator, outtake, Height.L3))
                .onFalse(BaseCommands.setPivot(outtake, 0));

        m_operator
                .triangle()
                .whileTrue(BaseCommands.setHeight(elevator, outtake, Height.L4))
                .onFalse(BaseCommands.setPivot(outtake, 0));

        // Set the selected position based on the D-Pad
        m_operator
                .povUp()
                .onTrue(new InstantCommand(() -> RobotState.setSelectedPosition(Position.MIDDLE)));
        m_operator.L3().onTrue(new InstantCommand(() -> RobotState.setSelectedPosition(Position.LEFT)));
        m_operator
                .R3()
                .onTrue(new InstantCommand(() -> RobotState.setSelectedPosition(Position.RIGHT)));

        // Auto-align
        m_operator
                .L2()
                .whileTrue(
                        Commands.sequence(
                                new InstantCommand(() -> RobotState.setMode(OperationMode.AUTO)),
                                new DriveToPose(
                                        drive,
                                        () -> TargetSelector.getNearestBranch(RobotState.getSelectedPosition()))))
                .onFalse(new InstantCommand(() -> RobotState.setMode(OperationMode.HUMAN)));
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
