// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int CONTROLLER_MAIN_LABEL = 0;
    public static final int MOTOR_LEFT_0_ID = 6;
    public static final int MOTOR_LEFT_1_ID = 7;
    public static final int MOTOR_RIGHT_0_ID = 8;
    public static final int MOTOR_RIGHT_1_ID = 9;

    public static final int LEFT_STICK_Y = 1; //the axis of the left stick
    public static final int RIGHT_STICK_Y = 5; //the axis of the right stick
    public static double SENSITIVITY = .5; //Sensitivity of the joysticks
}
