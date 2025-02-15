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

public class OuttakeIOTalonFX implements OuttakeIO {

  private final TalonFX pivotMotor;
  private final TalonFX outtakeMotor;

  private final DigitalInput beamBreak;
  private double setpoint = 0;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

  private final StatusSignal<Angle> pivotPosition;
  private final StatusSignal<AngularVelocity> pivotVelocity;
  private final StatusSignal<Voltage> pivotAppliedVolts;
  private final StatusSignal<Current> pivotCurrent;

  private final StatusSignal<Voltage> outtakeVoltage;
  private final StatusSignal<Current> outtakeCurrent;

  private final Debouncer pivotConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer outtakeConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer outtakeCurrentDebouncer = new Debouncer(0.1);

  public OuttakeIOTalonFX() {
    this.pivotMotor = new TalonFX(OuttakeConstants.pivotMotor, Constants.canivoreCANBus);
    this.outtakeMotor = new TalonFX(OuttakeConstants.outtakeMotor, Constants.canivoreCANBus);

    this.beamBreak = new DigitalInput(OuttakeConstants.beamBreakPort);

    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    pivotConfig.CurrentLimits.StatorCurrentLimit = OuttakeConstants.pivotMaxCurrent;
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    Slot0Configs pivotSlot0 = pivotConfig.Slot0;

    pivotSlot0.kP = OuttakeConstants.kP;
    pivotSlot0.kI = OuttakeConstants.kI;
    pivotSlot0.kD = OuttakeConstants.kD;

    pivotSlot0.kS = OuttakeConstants.kS;
    pivotSlot0.kV = OuttakeConstants.kV;
    pivotSlot0.kA = OuttakeConstants.kA;

    MotionMagicConfigs pivotMotionMagic = pivotConfig.MotionMagic;

    pivotMotionMagic.MotionMagicCruiseVelocity = OuttakeConstants.kMMCruiseVelocity;
    pivotMotionMagic.MotionMagicAcceleration = OuttakeConstants.kMMAcceleration;
    pivotMotionMagic.MotionMagicJerk = OuttakeConstants.kMMJerk;

    TalonFXConfiguration outtakeConfig = new TalonFXConfiguration();

    outtakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    outtakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    outtakeConfig.CurrentLimits.StatorCurrentLimit = OuttakeConstants.outtakeMaxCurrent;
    outtakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(pivotConfig, 0.25));
    tryUntilOk(5, () -> pivotMotor.setPosition(0.0, 0.25));
    tryUntilOk(5, () -> outtakeMotor.getConfigurator().apply(outtakeConfig, 0.25));
    tryUntilOk(5, () -> outtakeMotor.setPosition(0.0, 0.25));

    pivotPosition = pivotMotor.getPosition();
    pivotVelocity = pivotMotor.getVelocity();
    pivotAppliedVolts = pivotMotor.getMotorVoltage();
    pivotCurrent = pivotMotor.getStatorCurrent();

    outtakeVoltage = outtakeMotor.getMotorVoltage();
    outtakeCurrent = outtakeMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        pivotPosition,
        pivotVelocity,
        pivotAppliedVolts,
        pivotCurrent,
        outtakeVoltage,
        outtakeCurrent);

    ParentDevice.optimizeBusUtilizationForAll(pivotMotor, outtakeMotor);
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {
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

  @Override
  public void setPivotVoltage(double voltage) {
    pivotMotor.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public void setOuttakeVoltage(double voltage) {
    outtakeMotor.setControl(voltageRequest.withOutput(voltage));
  }

  // Degrees
  @Override
  public double getPivotPosition() {
    return Units.radiansToDegrees(pivotPosition.getValueAsDouble() / (OuttakeConstants.kGearRatio));
  }

  @Override
  public double getPivotCurrent() {
    return pivotCurrent.getValueAsDouble();
  }

  @Override
  public double getPivotVelocity() {
    return Units.rotationsToRadians(pivotVelocity.getValueAsDouble());
  }

  @Override
  public double getOuttakeCurrent() {
    return outtakeCurrent.getValueAsDouble();
  }

  @Override
  public boolean isBeamBreakTriggered() {

    double current = getOuttakeCurrent();
    return outtakeCurrentDebouncer.calculate(current < 60 && current > 25);
    // return beamBreak.get();
  }

  @Override
  public void setPivotAngle(double angle) {
    this.setpoint = angle;
    double rotations = Units.degreesToRotations(angle) * OuttakeConstants.kGearRatio * 2 * Math.PI;
    pivotMotor.setControl(motionMagicRequest.withPosition(rotations));
  }

  @Override
  public boolean isAtSetpoint() {
    return (Math.abs(setpoint - this.getPivotPosition()) <= OuttakeConstants.kTolerance);
  }

  @Override
  public void resetPivotEncoder() {
    tryUntilOk(5, () -> pivotMotor.setPosition(0.0, 0.25));
  }
}
