// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.TestModule;
import frc.robot.subsystems.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveDrive _swerveDrive = new SwerveDrive();
  private final RobotCtrl _robotCtrl = new RobotCtrl();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    // _swerveDrive.setDefaultCommand(new TestModule(_swerveDrive, _robotCtrl :: driveLeftY, _robotCtrl :: driveRightX));
    _swerveDrive.setDefaultCommand(new TeleopDrive(_swerveDrive, () -> -_robotCtrl.driveLeftY(), () -> -_robotCtrl.driveLeftX(), () -> -_robotCtrl.driveRightX()));

    configureBindings();
  }

  private void configureBindings() {

  }
}
