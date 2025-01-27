package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {

  private final ElevatorSim elevatorSim;

  public double kP = ElevatorConstants.kP; // Override with specific values
  public double kI = ElevatorConstants.kI; // Override with specific values
  public double kD = ElevatorConstants.kD; // Override with specific values

  public double masterMotorAppliedVolts = 0.0;
  public double slaveMotorAppliedVolts = 0.0;

  public ElevatorIOSim() {
    elevatorSim =
        new ElevatorSim(
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
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    elevatorSim.setInputVoltage(masterMotorAppliedVolts);
    elevatorSim.update(0.02);

    double currentDrawAmps = elevatorSim.getCurrentDrawAmps();
    double positionMeters = getPosition();
    double velocityMetersPerSecond = getVelocity();

    inputs.masterMotorConnected = true;
    inputs.masterMotorCurrentAmps = currentDrawAmps / 2;
    inputs.masterMotorAppliedVolts = masterMotorAppliedVolts;
    inputs.masterMotorPositionRad = positionMeters * ElevatorConstants.kGearRatio * 2 * Math.PI;
    inputs.masterMotorVelocityRadPerSec =
        velocityMetersPerSecond * ElevatorConstants.kGearRatio * 2 * Math.PI;

    inputs.slaveMotorConnected = true;
    inputs.slaveMotorCurrentAmps = currentDrawAmps / 2;
    inputs.slaveMotorAppliedVolts = slaveMotorAppliedVolts;
    inputs.slaveMotorPositionRad = positionMeters * ElevatorConstants.kGearRatio * 2 * Math.PI;
    inputs.slaveMotorVelocityRadPerSec =
        velocityMetersPerSecond * ElevatorConstants.kGearRatio * 2 * Math.PI;

    inputs.elevatorPositionMeters = positionMeters;
    inputs.elevatorVelocityMetersPerSec = velocityMetersPerSecond;
    inputs.elevatorCurrentAmps = currentDrawAmps;
  }

  @Override
  public void setVoltage(double voltage) {
    masterMotorAppliedVolts = voltage;
    slaveMotorAppliedVolts = voltage;
  }

  @Override
  public double getPosition() {
    return elevatorSim.getPositionMeters();
  }

  @Override
  public double getVelocity() {
    return elevatorSim.getVelocityMetersPerSecond();
  }
}
