package frc.robot.subsystems;

import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.AlgaePivotConstants;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

public class AlgaePivot extends SubsystemBase {
  private TalonFX algaePivot;
  private MotionMagicVoltage motionControl;
  private SysIdRoutine sysIdRoutine;
  private VoltageOut voltageOut;
  private DigitalInput limitSwitch;
  private double setpoint;

  public AlgaePivot() {
    algaePivot = new TalonFX(AlgaePivotConstants.kAlgaePivotID);
    limitSwitch = new DigitalInput(AlgaePivotConstants.kLimitSwitchID);

    algaePivot.getConfigurator().apply(AlgaePivotConstants.AlgaePivotConfiguration);
    algaePivot.getConfigurator().apply(AlgaePivotConstants.AlgaePivotMMConfiguration);

    motionControl = new MotionMagicVoltage(0.0);
    voltageOut = new VoltageOut(0.0);

    sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Velocity.ofBaseUnits(0.2, VelocityUnit.combine(Volts, Seconds)), Volts.of(1), null,
            (state) -> SignalLogger.writeString("state", state.toString())),
        new SysIdRoutine.Mechanism((volts) -> algaePivot.setControl(voltageOut.withOutput(volts.in(Volts))), null,
            this));

    setpoint = 0;
  }
  // I Love MEN
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);

  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  // returns if the limit switch activates or not (true or false)
  public boolean getLimitSwitchValue() {
    return limitSwitch.get();
  }

  // returns the algae pivot position (rotations)
  public double getPositionValue() {
    return algaePivot.getPosition().getValueAsDouble();
  }

  // resets the position of the algae pivot encoder to 0
  public void resetPosition() {
    algaePivot.setPosition(0);
  }

  // runs the algae pivot motor to a position setpoint
  public void toSetpoint(double newSetpoint) {
    setpoint = newSetpoint;
    algaePivot.setControl(motionControl.withPosition(setpoint));
  }

  // returns if the motion magic on the algae pivot motor is at the setpoint
  public boolean isAtSetpoint() {
    return Math.abs(setpoint - getPositionValue()) <= AlgaePivotConstants.kTolerance;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("[A] Pivot Position", algaePivot.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("[A] Pivot Current", algaePivot.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putBoolean("[A] Limit Switch", getLimitSwitchValue());

    if (getLimitSwitchValue()) {
      resetPosition();
    }
  }
}
