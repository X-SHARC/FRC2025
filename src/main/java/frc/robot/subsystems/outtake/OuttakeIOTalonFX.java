package frc.robot.subsystems.outtake;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

/**
 * OuttakeIOTalonFX is a subsystem that controls the outtake mechanism using TalonFX motor
 * controllers. It manages the pivot and outtake motors, beam break sensor, and provides methods to
 * control and monitor the system.
 */
public class OuttakeIOTalonFX implements OuttakeIO {

  // TalonFX motor controllers for pivot and outtake mechanisms
  private final TalonFX pivotMotor;
  private final TalonFX outtakeMotor;

  // Digital input for beam break sensor
  private final DigitalInput beamBreak;
  private double setpoint = 0;

  // Voltage control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

  // Status signals for pivot motor
  private final StatusSignal<Angle> pivotPosition;
  private final StatusSignal<AngularVelocity> pivotVelocity;
  private final StatusSignal<Voltage> pivotAppliedVolts;
  private final StatusSignal<Current> pivotCurrent;

  // Status signals for outtake motor
  private final StatusSignal<Voltage> outtakeVoltage;
  private final StatusSignal<Current> outtakeCurrent;

  // Debouncers for connection status
  private final Debouncer pivotConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer outtakeConnectedDebouncer = new Debouncer(0.5);
  // private final Debouncer outtakeCurrentDebouncer = new Debouncer(0.1);

  public OuttakeIOTalonFX() {
    // Initialize motors with CAN bus
    this.pivotMotor = new TalonFX(OuttakeConstants.pivotMotor, Constants.canivoreCANBus);
    this.outtakeMotor = new TalonFX(OuttakeConstants.outtakeMotor, Constants.canivoreCANBus);

    // Initialize beam break sensor
    this.beamBreak = new DigitalInput(OuttakeConstants.beamBreakPort);

    // Configure pivot motor
    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    pivotConfig.CurrentLimits.StatorCurrentLimit = OuttakeConstants.pivotMaxCurrent;
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // PID slot configuration for pivot motor
    Slot0Configs pivotSlot0 = pivotConfig.Slot0;
    pivotSlot0.kP = OuttakeConstants.kP;
    pivotSlot0.kI = OuttakeConstants.kI;
    pivotSlot0.kD = OuttakeConstants.kD;
    pivotSlot0.kS = OuttakeConstants.kS;
    pivotSlot0.kV = OuttakeConstants.kV;
    pivotSlot0.kA = OuttakeConstants.kA;

    // Motion magic configuration for pivot motor
    MotionMagicConfigs pivotMotionMagic = pivotConfig.MotionMagic;
    pivotMotionMagic.MotionMagicCruiseVelocity = OuttakeConstants.kMMCruiseVelocity;
    pivotMotionMagic.MotionMagicAcceleration = OuttakeConstants.kMMAcceleration;
    pivotMotionMagic.MotionMagicJerk = OuttakeConstants.kMMJerk;

    // Configure outtake motor
    TalonFXConfiguration outtakeConfig = new TalonFXConfiguration();
    outtakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    outtakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    outtakeConfig.CurrentLimits.StatorCurrentLimit = OuttakeConstants.outtakeMaxCurrent;
    outtakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Apply configurations and set initial positions
    tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(pivotConfig, 0.25));
    tryUntilOk(5, () -> pivotMotor.setPosition(0.0, 0.25));
    tryUntilOk(5, () -> outtakeMotor.getConfigurator().apply(outtakeConfig, 0.25));
    tryUntilOk(5, () -> outtakeMotor.setPosition(0.0, 0.25));

    // Initialize status signals
    pivotPosition = pivotMotor.getPosition();
    pivotVelocity = pivotMotor.getVelocity();
    pivotAppliedVolts = pivotMotor.getMotorVoltage();
    pivotCurrent = pivotMotor.getStatorCurrent();
    outtakeVoltage = outtakeMotor.getMotorVoltage();
    outtakeCurrent = outtakeMotor.getStatorCurrent();

    // Set update frequency for status signals
    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        pivotPosition,
        pivotVelocity,
        pivotAppliedVolts,
        pivotCurrent,
        outtakeVoltage,
        outtakeCurrent);

    // Optimize bus utilization
    ParentDevice.optimizeBusUtilizationForAll(pivotMotor, outtakeMotor);
  }

  /**
   * Updates the input values for the outtake system.
   *
   * @param inputs The input values to be updated.
   */
  @Override
  public void updateInputs(OuttakeIOInputs inputs) {
    // Refresh status signals and update inputs
    StatusCode pivotStatus =
        BaseStatusSignal.refreshAll(pivotPosition, pivotVelocity, pivotAppliedVolts, pivotCurrent);
    StatusCode outtakeStatus = BaseStatusSignal.refreshAll(outtakeVoltage, outtakeCurrent);

    inputs.pivotMotorConnected = pivotConnectedDebouncer.calculate(pivotStatus.isOK());
    inputs.pivotMotorPositionRad = Units.rotationsToRadians(pivotPosition.getValueAsDouble());
    inputs.pivotMotorVelocityRadPerSec = Units.rotationsToRadians(pivotVelocity.getValueAsDouble());
    inputs.pivotMotorAppliedVolts = pivotAppliedVolts.getValueAsDouble();
    inputs.pivotMotorCurrentAmps = pivotCurrent.getValueAsDouble();

    inputs.outtakeMotorConnected = outtakeConnectedDebouncer.calculate(outtakeStatus.isOK());
    inputs.outtakeMotorAppliedVolts = outtakeVoltage.getValueAsDouble();
    inputs.outtakeMotorCurrentAmps = outtakeCurrent.getValueAsDouble();

    inputs.pivotAngleDegrees = getPivotPosition();
  }

  /**
   * Sets the voltage for the pivot motor.
   *
   * @param voltage The voltage to be set.
   */
  @Override
  public void setPivotVoltage(double voltage) {
    pivotMotor.setControl(voltageRequest.withOutput(voltage));
  }

  /**
   * Sets the voltage for the outtake motor.
   *
   * @param voltage The voltage to be set.
   */
  @Override
  public void setOuttakeVoltage(double voltage) {
    // Set outtake motor voltage
    outtakeMotor.setControl(voltageRequest.withOutput(voltage));
  }

  /**
   * Gets the current position of the pivot motor in degrees.
   *
   * @return The pivot motor position in degrees.
   */
  @Override
  public double getPivotPosition() {
    return Units.radiansToDegrees(pivotPosition.getValueAsDouble() / (OuttakeConstants.kGearRatio));
  }

  /**
   * Gets the current drawn by the pivot motor.
   *
   * @return The pivot motor current in amps.
   */
  @Override
  public double getPivotCurrent() {
    return pivotCurrent.getValueAsDouble();
  }

  /**
   * Gets the velocity of the pivot motor.
   *
   * @return The pivot motor velocity in radians per second.
   */
  @Override
  public double getPivotVelocity() {
    return Units.rotationsToRadians(pivotVelocity.getValueAsDouble());
  }

  /**
   * Gets the current drawn by the outtake motor.
   *
   * @return The outtake motor current in amps.
   */
  @Override
  public double getOuttakeCurrent() {
    return outtakeCurrent.getValueAsDouble();
  }

  /**
   * Checks if the beam break sensor is triggered.
   *
   * @return True if the beam break sensor is triggered, false otherwise.
   */
  @Override
  public boolean isBeamBreakTriggered() {
    return beamBreak.get();
    // double current = getOuttakeCurrent();
    // return outtakeCurrentDebouncer.calculate(current < 60 && current > 25);
  }

  /**
   * Sets the angle for the pivot motor using motion magic.
   *
   * @param angle The angle to be set in degrees.
   */
  @Override
  public void setPivotAngle(double angle) {
    this.setpoint = angle;
    double rotations = Units.degreesToRotations(angle) * OuttakeConstants.kGearRatio * 2 * Math.PI;
    pivotMotor.setControl(motionMagicRequest.withPosition(rotations));
  }

  /**
   * Checks if the pivot motor is at the setpoint.
   *
   * @return True if the pivot motor is at the setpoint, false otherwise.
   */
  @Override
  public boolean isAtSetpoint() {
    return (Math.abs(setpoint - this.getPivotPosition()) <= OuttakeConstants.kTolerance);
  }

  /** Resets the encoder position of the pivot motor. */
  @Override
  public void resetPivotEncoder() {
    tryUntilOk(5, () -> pivotMotor.setPosition(0.0, 0.25));
  }
}
