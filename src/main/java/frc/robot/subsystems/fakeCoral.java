// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class fakeCoral extends SubsystemBase {
  /** Creates a new fakeCoral. */
  private SparkMax can;
  public fakeCoral() {
    can = new SparkMax(7, MotorType.kBrushless);
  }

  public SparkLimitSwitch getLimitSwitch(){
    return can.getForwardLimitSwitch();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
