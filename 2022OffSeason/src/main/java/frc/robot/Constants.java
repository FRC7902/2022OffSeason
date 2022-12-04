// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final static class DriveConstants {
        //CAN
        // PWM ports for drive motors
        public static final int kLeftLeader = 1;
        public static final int kLeftFollower = 2;
        public static final int kRightLeader = 11;
        public static final int kRightFollower = 12;


        public static final int kLeftEncoderA = 0;
        public static final int kLeftEncoderB = 1; 
        public static final int kLeftEncoderIndex = 2;

        public static final int kRightEncoderA = 3; 
        public static final int kRightEncoderB = 4; 
        public static final int kRightEncoderIndex = 5;

        public static final int gyroChannel = 1;
    }
    public final static class ShooterConstants{
        

    }
    public final static class TransferConstants{
        public static final int kTransferCAN = 5;
        public static final double kRampTime = 0;
    }

}
