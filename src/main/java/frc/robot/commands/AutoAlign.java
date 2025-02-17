package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SharcGen;
import frc.robot.state.RobotState;
import java.util.function.Supplier;

public class AutoAlign extends Command {
  private final Supplier<Command> commandSupplier;
  private Command currentCommand;

  public AutoAlign(SharcGen generator) {
    this.commandSupplier = () -> generator.generateSidePath();
  }

  @Override
  public void initialize() {
    currentCommand = commandSupplier.get();
    currentCommand.schedule();
  }

  @Override
  public void execute() {
    if (!RobotState.isAuto()) {
      cancelCommand();
      end(true);
    }
  }

  @Override
  public boolean isFinished() {
    return currentCommand != null && currentCommand.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    cancelCommand();
  }

  private void cancelCommand() {
    if (currentCommand != null && currentCommand.isScheduled()) {
      currentCommand.cancel();
    }
  }
}
