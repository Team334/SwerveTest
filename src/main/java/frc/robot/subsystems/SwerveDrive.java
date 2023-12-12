// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SwerveModule;

public class SwerveDrive extends SubsystemBase {
  // TODO: Get angle offset for each module (zero each one)
  private final SwerveModule _frontLeft = new SwerveModule(Constants.CAN.DRIVE_FRONT_LEFT, Constants.CAN.ROT_FRONT_LEFT, Constants.CAN.ENC_FRONT_LEFT, Constants.Offsets.ENCODER_FRONT_LEFT);
  private final SwerveModule _frontRight = new SwerveModule(Constants.CAN.DRIVE_FRONT_RIGHT, Constants.CAN.ROT_FRONT_RIGHT, Constants.CAN.ENC_FRONT_RIGHT, Constants.Offsets.ENCODER_FRONT_RIGHT);
  private final SwerveModule _backRight = new SwerveModule(Constants.CAN.DRIVE_BACK_RIGHT, Constants.CAN.ROT_BACK_RIGHT, Constants.CAN.ENC_BACK_RIGHT, Constants.Offsets.ENCODER_BACK_RIGHT);
  private final SwerveModule _backLeft = new SwerveModule(Constants.CAN.DRIVE_BACK_LEFT, Constants.CAN.ROT_BACK_LEFT, Constants.CAN.ENC_BACK_LEFT, Constants.Offsets.ENCODER_BACK_LEFT);

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Front Left", _frontLeft.getAngle());
    SmartDashboard.putNumber("Front Right", _frontRight.getAngle());
    SmartDashboard.putNumber("Back Right", _backRight.getAngle());
    SmartDashboard.putNumber("Back Left", _backLeft.getAngle());

    // SmartDashboard.putNumber("Front Left Speed", _frontLeft.getDriveVelocity());
    // SmartDashboard.putNumber("Front Left Angle", _frontLeft.getAngle());
    // SmartDashboard.putNumber("Front Right Speed", _frontRight.getDriveVelocity());
    // SmartDashboard.putNumber("Front Right Angle", _frontRight.getAngle());
  }

  public void setStates(SwerveModuleState[] states) {
    _frontLeft.setState(states[0]);
    _frontRight.setState(states[1]);
    _backRight.setState(states[2]);
    _backLeft.setState(states[3]);
  }

  /**
   * Calls drive method of each SwerveModule.
   */
  public void driveTest(double speed) {
    // _frontLeft.drive(speed);
    _frontRight.drive(speed);
    // _backRight.drive(speed);
    // _backLeft.drive(speed);
  }

  /**
   * Calls rotate method of each SwerveModule.
   */
  public void rotateTest(double speed) {
    // _frontLeft.rotate(speed);
    _frontRight.rotate(speed);
    // _backRight.rotate(speed);
    // _backLeft.rotate(speed);
  }

  public void stateTest(SwerveModuleState state) {
    _frontLeft.setState(state);
    _frontRight.setState(state);
    _backRight.setState(state);
    _backLeft.setState(state);
  }
}
