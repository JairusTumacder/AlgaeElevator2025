// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// neo 550 for algea intake
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
 
public class AlgeaShooter extends SubsystemBase {
  private final TalonFX algeaMotor = new TalonFX(1); 
  private boolean ballHeld = false;

  

  public void algeaIntake() {
    ballHeld = false;
      if (!ballHeld) {
          algeaMotor.set(-1); // Full speed intake
      }
  }

  public void checkMotorStall() {
      if (algeaMotor.getStatorCurrent().getValueAsDouble() > 20 && algeaMotor.get() < -0.08) {  //check the direction )    // replace with .getOutPutCurrent for sparks
          algeaMotor.set(0); // Stop the motor
          algeaMotor.setNeutralMode(NeutralModeValue.Brake);
          ballHeld = true;
      }
  }
  public void stopIntake() {
      algeaMotor.set(0); // Stop the motor
      ballHeld = false;
  }
  public void algeaOutake() {
        ballHeld = false;
        algeaMotor.set(.9); // Reverse to shoot the algea
      

  }

  @Override
  public void periodic() {
      checkMotorStall(); // Check if the motor is stalled
      if (!ballHeld) {
        algeaMotor.setNeutralMode(NeutralModeValue.Coast);
      }
      SmartDashboard.putBoolean("ballHeld", ballHeld); // Display if the ball is held
      SmartDashboard.putNumber("current", algeaMotor.getStatorCurrent().getValueAsDouble()); // replace with .getOutPutCurrent for sparks
  }
}