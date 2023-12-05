package frc.robot.utils;


import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule {
    private final TalonFX _driveMotor;
    private final TalonFX _rotationMotor;

    private final PIDController _rotationController;

    private final CANCoder _encoder;

    public SwerveModule(int driveMotorId, int rotationMotorId, int encoderId, double angleOffset) {
        _driveMotor = new TalonFX(driveMotorId);
        _rotationMotor = new TalonFX(rotationMotorId);

        _encoder = new CANCoder(encoderId);

        _encoder.configMagnetOffset(angleOffset, Constants.CAN.CAN_TIMEOUT);
        _encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180, Constants.CAN.CAN_TIMEOUT);

        _rotationController = new PIDController(0, 0, 0);

        TalonFXConfig.configureFalcon(_driveMotor);
        TalonFXConfig.configureFalcon(_rotationMotor);
    }

    public void drive(double speed) {
        _driveMotor.set(TalonFXControlMode.PercentOutput, speed);
        // _driveMotor.set(TalonFXControlMode.Velocity, speed);
    }

    public void rotate(double speed) {
        _rotationMotor.set(TalonFXControlMode.PercentOutput, speed);
    }

    public double getDriveVelocity() {
        return _driveMotor.getSelectedSensorVelocity();
    }

    public double getAngle() {
        return _encoder.getAbsolutePosition();
    }

    public void setState(SwerveModuleState state) {
        // double state.speedMetersPerSecond
    }

    public SwerveModuleState getState() {
        // TODO: Make this return a value and use radians?
        return new SwerveModuleState();
    }
}
