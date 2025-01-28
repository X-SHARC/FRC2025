package frc.robot.subsystems.elevator;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {

  private final ElevatorSim elevatorSim;
  public double masterMotorAppliedVolts = 0.0;
  public double slaveMotorAppliedVolts = 0.0;

  private PIDConstants pidConstants = ElevatorConstants.kSimPIDConstants;
  private PIDController controller = new PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD);

  public ElevatorIOSim() {
    elevatorSim = new ElevatorSim(
        LinearSystemId.createElevatorSystem(
            DCMotor.getKrakenX60(2),
            ElevatorConstants.weight,
            ElevatorConstants.gearRadius,
            ElevatorConstants.kGearRatio),
        DCMotor.getKrakenX60(2),
        0,
        0.90,
        true,
        0);

    this.controller.setTolerance(ElevatorConstants.kTolerance);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    elevatorSim.setInputVoltage(masterMotorAppliedVolts);
    elevatorSim.update(0.02);

    double currentDrawAmps = Math.abs(elevatorSim.getCurrentDrawAmps());
    double positionMeters = getPosition();
    double velocityMetersPerSecond = getVelocity();

    inputs.masterMotorConnected = true;
    inputs.masterMotorCurrentAmps = currentDrawAmps / 2;
    inputs.masterMotorAppliedVolts = masterMotorAppliedVolts;
    inputs.masterMotorPositionRad = positionMeters * ElevatorConstants.kGearRatio * 2 * Math.PI;
    inputs.masterMotorVelocityRadPerSec = velocityMetersPerSecond * ElevatorConstants.kGearRatio * 2 * Math.PI;

    inputs.slaveMotorConnected = true;
    inputs.slaveMotorCurrentAmps = currentDrawAmps / 2;
    inputs.slaveMotorAppliedVolts = slaveMotorAppliedVolts;
    inputs.slaveMotorPositionRad = positionMeters * ElevatorConstants.kGearRatio * 2 * Math.PI;
    inputs.slaveMotorVelocityRadPerSec = velocityMetersPerSecond * ElevatorConstants.kGearRatio * 2 * Math.PI;

    inputs.elevatorPositionMeters = positionMeters;
    inputs.elevatorVelocityMetersPerSec = velocityMetersPerSecond;
    inputs.elevatorCurrentAmps = currentDrawAmps;
  }

  @Override
  public void setVoltage(double voltage) {
    masterMotorAppliedVolts = voltage;
    slaveMotorAppliedVolts = voltage;
  }

  // return centimeters
  @Override
  public double getPosition() {
    return elevatorSim.getPositionMeters();
  }

  @Override
  public double getCurrent() {
    return Math.abs(elevatorSim.getCurrentDrawAmps());
  }

  @Override
  public double getVelocity() {
    return elevatorSim.getVelocityMetersPerSecond();
  }

  @Override
  public boolean isAtSetpoint() {
    return controller.atSetpoint();
  }

  @Override
  public void setHeight(double height) {
    double out = controller.calculate(getPosition() * 100, height);
    setVoltage(MathUtil.clamp(out, -0.9, .9) * 12);
  }
}
