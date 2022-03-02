// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DRIVE_TRAIN{
        public static final int LEFT_L_MOTOR_ID = 40;
        public static final int LEFT_F_MOTOR_ID = 41;
        public static final int RIGHT_L_MOTOR_ID = 42;
        public static final int RIGHT_F_MOTOR_ID = 43;

        public static final double ENCODER_DISTANCE_CONVERSION_FACTOR = 0.044705;

        public static final double FRAME_WIDTH = 0.6223; //meters

        public static final double Z_AXIS_TELEOP_ADJUSTMENT = 2;
    }

    public static final class FLAPPER{
        public static final double SPEED = 0.2;
    }

    public static final class INTAKE{
        public static final double SPEED = 0.6;
        public static final int MOTOR_ID = 21;
    }

    public static final class HANGER{
        public static final int INIT_BUTTON_1 = 1;
        public static final int INIT_BUTTON_2 = 1;

        public static final int EXTEND_BUTTON = 1;
        public static final int WINCH_BUTTON = 1;

        public static final double EXTENDER_MAX_POSITION = 1;
        public static final double EXTENDER_MIN_POSITION = 1;
        public static final double EXTENDER_HOOK_POSITION = 1;

        public static final double WINCH_MAX_POSITION = 1;
        public static final double WINCH_MIN_POSITION = 1;
    }
}
