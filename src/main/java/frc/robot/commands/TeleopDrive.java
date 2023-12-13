// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotCtrl;
import frc.robot.subsystems.SwerveDrive;

public class TeleopDrive extends CommandBase {
  private final SwerveDrive _swerveDrive;

  private final DoubleSupplier _xSpeed;
  private final DoubleSupplier _ySpeed;

  private final DoubleSupplier _rotationSpeed;

  /** Creates a new TeleopDrive. */
  public TeleopDrive(SwerveDrive swerveDrive, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotationSpeed) {
    _swerveDrive = swerveDrive;

    _xSpeed = xSpeed;
    _ySpeed = ySpeed;

    _rotationSpeed = rotationSpeed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // _swerveDrive.stateTest(new SwerveModuleState(0, new Rotation2d(Math.toRadians(0))));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // apply controller deadband
    double xSpeed = RobotCtrl.ApplyDeadband(_xSpeed.getAsDouble(), 0.07);
    double ySpeed = RobotCtrl.ApplyDeadband(_ySpeed.getAsDouble(), 0.07);
    double rotationSpeed = RobotCtrl.ApplyDeadband(_rotationSpeed.getAsDouble(), 0.07);

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
      xSpeed * Constants.Speeds.SWERVE_DRIVE_MAX_SPEED,
      ySpeed * Constants.Speeds.SWERVE_DRIVE_MAX_SPEED,
      rotationSpeed * Constants.Speeds.SWERVE_DRIVE_MAX_ANGULAR_SPEED
    );

    SwerveModuleState[] moduleStates = Constants.Physical.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

    _swerveDrive.setStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}