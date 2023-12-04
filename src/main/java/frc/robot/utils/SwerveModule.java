package frc.robot.utils;


import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class SwerveModule {
    private final TalonFX _driveMotor;
    private final TalonFX _rotationMotor;

    private final CANCoder _encoder;

    public SwerveModule(int driveMotorId, int rotationMotorId, int encoderId) {
        _driveMotor = new TalonFX(driveMotorId);
        _rotationMotor = new TalonFX(rotationMotorId);

        _encoder = new CANCoder(encoderId);

        _encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360, Constants.CAN.CAN_TIMEOUT);
        TalonFXConfig.configureFalcon(_driveMotor);
        TalonFXConfig.configureFalcon(_rotationMotor);
    }

    public void drive(double speed) {
        _driveMotor.set(TalonFXControlMode.PercentOutput, speed);
    }

    public void rotate(double speed) {
        _rotationMotor.set(TalonFXControlMode.PercentOutput, speed);
    }

    public double getEnc() {
        return _encoder.getAbsolutePosition();
    }
}
