package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class SwerveModule {
    private final TalonFX _driveMotor;
    private final TalonFX _rotationMotor;

    public SwerveModule(int driveMotorId, int rotationMotorId) {
        _driveMotor = new TalonFX(driveMotorId);
        _rotationMotor = new TalonFX(rotationMotorId);
    }

    public void rotate(int speed) {
        _rotationMotor.set(TalonFXControlMode.PercentOutput, speed);
    }

    public void drive(int speed) {
        _driveMotor.set(TalonFXControlMode.PercentOutput, speed);
    }

}
