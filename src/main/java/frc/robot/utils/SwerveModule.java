package frc.robot.utils;


import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

        _rotationController = new PIDController(0.07, 0, 0);
        _rotationController.enableContinuousInput(-180, 180);
        _rotationController.setTolerance(0.5);

        // SmartDashboard.putData("PID CONTROLLER", _rotationController);

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
        double talon_rps = (_driveMotor.getSelectedSensorVelocity() / 2048) * 10;
        double wheel_circumference = 2 * Math.PI * Constants.Physical.SWERVE_DRIVE_WHEEL_RADIUS;
        
        // WHEEL ROTATIONS PER SECOND
        // TODO: CONFIRM THE CONSTANT VALUES
        // return the speed of the swerve wheel itself (talon rps times gear ratio time wheel size)
        return (talon_rps / Constants.Physical.SWERVE_DRIVE_GEAR_RATIO) * wheel_circumference;
    }

    public double getAngle() {
        return _encoder.getAbsolutePosition();
    }

    public void setState(SwerveModuleState state) {
        // TODO: TEST THAT THIS WORKS

        state = SwerveModuleState.optimize(state, new Rotation2d(Math.toRadians(getAngle())));

        double rotation_volts = -MathUtil.clamp(_rotationController.calculate(getAngle(), state.angle.getDegrees()), -0.7, 0.7);
        double speed = MathUtil.clamp(state.speedMetersPerSecond, -Constants.Speeds.SWERVE_DRIVE_MAX_SPEED, Constants.Speeds.SWERVE_DRIVE_MAX_SPEED);

        rotate(
            rotation_volts / RobotController.getBatteryVoltage()
        );

        _driveMotor.set(TalonFXControlMode.PercentOutput, (speed / Constants.Speeds.SWERVE_DRIVE_MAX_SPEED) * Constants.Speeds.SWERVE_DRIVE_SPEED);
    }
}
