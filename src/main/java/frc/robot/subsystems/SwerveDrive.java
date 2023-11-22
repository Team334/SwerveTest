// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
  private final TalonFX _randomTalon;

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    _randomTalon = new TalonFX(7);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
