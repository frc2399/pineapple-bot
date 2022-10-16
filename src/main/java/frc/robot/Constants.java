package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {
    public static final class DriveConstants {
        // motor ids
        public static final int RIGHT_FRONT_MOTOR_ID = 1;
        public static final int RIGHT_BACK_MOTOR_ID = 2;
        public static final int LEFT_FRONT_MOTOR_ID = 4;
        public static final int LEFT_BACK_MOTOR_ID = 3;

        // No turning sensitivity
        public static final double MAX_TURN_SPEED = 1;

        public static final double TURN_SENSITIVITY = 1;

    }

    public static final class JoystickConstants {
        // joystick ports
        public static final int JOYSTICK_PORT = 1;

        public static final int FORWARD_JOYSTICK_INVERT = 1;    
        public static final int TURN_JOYSTICK_INVERT = 1;

    } 

    public static final class XboxMappingToJoystick{
        public static final int LEFT_STICK_X = 0;
        public static final int LEFT_STICK_Y = 1;
        public static final int RIGHT_STICK_X = 4;
        public static final int RIGHT_STICK_Y = 5;

        public static final int LEFT_TRIGGER = 2;
        public static final int RIGHT_TRIGGER = 3;

        public static final int LEFT_BUMPER = 5;
        public static final int RIGHT_BUMPER = 6;
        public static final int A_BUTTON = 1;
        public static final int B_BUTTON = 2;
        public static final int X_BUTTON = 3;
        public static final int Y_BUTTON = 4;
        public static final int START_BUTTON = 8;
        public static final int BACK_BUTTON = 7;

        public static final int LEFT_STICK_PUSH = 9;
        public static final int RIGHT_STICK_PUSH = 10;
    }

    public static final class XboxConstants {
        public static final int XBOX_PORT = 0;

        public static final int ARCADE_DRIVE_SPEED_AXIS = XboxMappingToJoystick.LEFT_STICK_Y;
        public static final int ARCADE_DRIVE_TURN_AXIS = XboxMappingToJoystick.RIGHT_STICK_X; 


        public static final int FORWARD_JOYSTICK_INVERT = 1;
        public static final int TURN_JOYSTICK_INVERT = 1;

        public static final double FORWARD_DEADBAND = 0.05;
        public static final double TURN_DEADBAND = 0.1;

        public static final double DRIVE_SLEW_RATE = 5.0;
        public static final double TURN_SLEW_RATE = 5.0;

        public static final double JOYSTICK_SENSITIVITY = 0.5;
    }
  
}




