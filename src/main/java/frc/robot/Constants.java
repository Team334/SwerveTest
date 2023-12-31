// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class CAN {
    public static final int DRIVE_FRONT_LEFT = 1;
    public static final int ROT_FRONT_LEFT = 2;
    
    public static final int DRIVE_FRONT_RIGHT = 3;
    public static final int ROT_FRONT_RIGHT = 4;

    public static final int DRIVE_BACK_RIGHT = 5;
    public static final int ROT_BACK_RIGHT = 6;

    public static final int DRIVE_BACK_LEFT = 7;
    public static final int ROT_BACK_LEFT = 8;

    public static final int ENC_FRONT_LEFT = 9;
    public static final int ENC_FRONT_RIGHT = 10;
    public static final int ENC_BACK_LEFT = 12;
    public static final int ENC_BACK_RIGHT = 11;

    public static final int CAN_TIMEOUT = 10;
  }

  public static class Speeds {
    public static final double SWERVE_DRIVE_COEFF = 0.3;

    public static final double SWERVE_DRIVE_MAX_SPEED = 2.88; // TODO: Get this value
    public static final double SWERVE_DRIVE_MAX_ANGULAR_SPEED = Math.PI; // Todo: Get this value
  }

  public static class Physical {
    // GEAR RATIOS ARE: DRIVEN GEAR TEETH / DRIVING GEAR TEETH

    public static final double SWERVE_DRIVE_GEAR_RATIO = 6.75; // TODO: Get this value
    public static final double SWERVE_DRIVE_WHEEL_RADIUS = 0.1; // TODO: Get this value

    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(0.292, 0.292),
      new Translation2d(0.292, -0.292),
      new Translation2d(-0.292, -0.292),
      new Translation2d(-0.292, 0.292)
    );
  }

  public static class Offsets {
    public static final double ENCODER_FRONT_LEFT = -93;
    public static final double ENCODER_FRONT_RIGHT = -58;
    public static final double ENCODER_BACK_RIGHT = 10;
    public static final double ENCODER_BACK_LEFT = 43;
   
  }

  public static class Ports {
    public static final int DRIVER_CONTROLLER = 0;
  }
}
