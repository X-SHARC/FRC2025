package frc.robot.subsystems.elevator;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ElevatorIOTalonFX implements ElevatorIO {

  private final TalonFX masterMotor;
  private final TalonFX slaveMotor;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

  private final StatusSignal<Angle> masterPosition;
  private final StatusSignal<AngularVelocity> masterVelocity;
  private final StatusSignal<Voltage> masterAppliedVolts;
  private final StatusSignal<Current> masterCurrent;

  private final StatusSignal<Angle> slavePosition;
  private final StatusSignal<AngularVelocity> slaveVelocity;
  private final StatusSignal<Voltage> slaveAppliedVolts;
  private final StatusSignal<Current> slaveCurrent;

  private final Debouncer masterConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer slaveConnectedDebouncer = new Debouncer(0.5);

  public ElevatorIOTalonFX() {
    this.masterMotor = new TalonFX(ElevatorConstants.masterMotorPort, Constants.canivoreCANBus);
    this.slaveMotor = new TalonFX(ElevatorConstants.slaveMotorPort, Constants.canivoreCANBus);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // TODO: Check inverted values
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.MaxCurrent;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    Slot0Configs slot0 = config.Slot0;
    slot0.kP = ElevatorConstants.kP;
    slot0.kI = ElevatorConstants.kI;
    slot0.kD = ElevatorConstants.kD;

    slot0.kS = ElevatorConstants.kS;
    slot0.kV = ElevatorConstants.kV;
    slot0.kA = ElevatorConstants.kA;
    slot0.kG = ElevatorConstants.kG;

    MotionMagicConfigs motionMagic = config.MotionMagic;

    motionMagic.MotionMagicCruiseVelocity = ElevatorConstants.kMMCruiseVelocity;
    motionMagic.MotionMagicAcceleration = ElevatorConstants.kMMAcceleration;
    motionMagic.MotionMagicJerk = ElevatorConstants.kMMJerk;

    tryUntilOk(5, () -> masterMotor.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> masterMotor.setPosition(0.0, 0.25));
    tryUntilOk(5, () -> slaveMotor.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> slaveMotor.setPosition(0.0, 0.25));

    slaveMotor.setControl(new Follower(masterMotor.getDeviceID(), true));

    masterPosition = masterMotor.getPosition();
    masterVelocity = masterMotor.getVelocity();
    masterAppliedVolts = masterMotor.getMotorVoltage();
    masterCurrent = masterMotor.getStatorCurrent();

    slavePosition = slaveMotor.getPosition();
    slaveVelocity = slaveMotor.getVelocity();
    slaveAppliedVolts = slaveMotor.getMotorVoltage();
    slaveCurrent = slaveMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        masterPosition,
        masterVelocity,
        masterAppliedVolts,
        masterCurrent,
        slavePosition,
        slaveVelocity,
        slaveAppliedVolts,
        slaveCurrent);

    ParentDevice.optimizeBusUtilizationForAll(masterMotor, slaveMotor);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    StatusCode masterStatus =
        BaseStatusSignal.refreshAll(
            masterPosition, masterVelocity, masterAppliedVolts, masterCurrent);

    StatusCode slaveStatus =
        BaseStatusSignal.refreshAll(slavePosition, slaveVelocity, slaveAppliedVolts, slaveCurrent);

    inputs.masterMotorConnected = masterConnectedDebouncer.calculate(masterStatus.isOK());
    inputs.masterMotorPositionRad = Units.rotationsToRadians(masterPosition.getValueAsDouble());
    inputs.masterMotorVelocityRadPerSec =
        Units.rotationsToRadians(masterVelocity.getValueAsDouble());
    inputs.masterMotorAppliedVolts = masterAppliedVolts.getValueAsDouble();
    inputs.masterMotorCurrentAmps = masterCurrent.getValueAsDouble();

    inputs.slaveMotorConnected = slaveConnectedDebouncer.calculate(slaveStatus.isOK());
    inputs.slaveMotorPositionRad = Units.rotationsToRadians(slavePosition.getValueAsDouble());
    inputs.slaveMotorVelocityRadPerSec = Units.rotationsToRadians(slaveVelocity.getValueAsDouble());
    inputs.slaveMotorAppliedVolts = slaveAppliedVolts.getValueAsDouble();
    inputs.slaveMotorCurrentAmps = slaveCurrent.getValueAsDouble();

    inputs.elevatorPositionMeters =
        inputs.masterMotorPositionRad * ElevatorConstants.gearRadius / ElevatorConstants.kGearRatio;
    inputs.elevatorVelocityMetersPerSec =
        inputs.masterMotorVelocityRadPerSec
            * ElevatorConstants.gearRadius
            / ElevatorConstants.kGearRatio;
    inputs.elevatorCurrentAmps =
        Math.abs(inputs.masterMotorCurrentAmps) + Math.abs(inputs.slaveMotorCurrentAmps);
  }

  @Override
  public void setVoltage(double voltage) {
    masterMotor.setControl(voltageRequest.withOutput(voltage));
  }

  // return meters
  @Override
  public double getPosition() {
    return Units.rotationsToRadians(masterPosition.getValueAsDouble())
        * ElevatorConstants.gearRadius
        / ElevatorConstants.kGearRatio;
  }

  // meters per second
  @Override
  public double getVelocity() {
    return Units.rotationsToRadians(masterVelocity.getValueAsDouble())
        * ElevatorConstants.gearRadius
        / ElevatorConstants.kGearRatio;
  }

  @Override
  public double getCurrent() {
    return Math.abs(masterCurrent.getValueAsDouble()) + Math.abs(slaveCurrent.getValueAsDouble());
  }

  @Override
  public void resetEncoders() {
    tryUntilOk(5, () -> masterMotor.setPosition(0.0, 0.25));
    tryUntilOk(5, () -> slaveMotor.setPosition(0.0, 0.25));
  }

  @Override
  public boolean isAtSetpoint() {
    return false;
  }

  // set height in meters
  @Override
  public void setHeight(double meters) {
    double rotations =
        meters * ElevatorConstants.kGearRatio / ElevatorConstants.gearRadius * 2 * Math.PI;
    masterMotor.setControl(motionMagicRequest.withPosition(rotations));
  }
}
