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

    public static final class DriveConstants{

        public final static int DrivetrainRightLeaderCAN = 4;
        public final static int DrivetrainRightFollowerCAN = 3;

        public final static int DrivetrainLeftLeaderCAN = 11;
        public final static int DrivetrainLeftFollowerCAN = 12;
    }

    public final static class ShooterConstants{
        public final static int ShooterLeaderCAN = 8;
        public final static int ShooterFollowerCAN = 9;

        //for every 3.75 rotations of input, output rotates once
        public final static double ShooterOutputGearRatio = 3.75;
    }

    public final static class TransferConstants{
        public final static int TransferVerticalCAN = 5;
        public final static double VerticalTransferRampTimeInSeconds = 1.5;
        public final static double VerticalTransferUpPower = 0.35;
        public final static double TransferVerticalGearRatio = 7;
    }

    public final static class ClimberRopeConstants{
        public final static int ClimberRopeAdjustmentCAN = 2;
        public final static int ClimberRopeMainCAN = 14;

        public final static double ClimberRopeAdjustmentGearRatio = 28;
        public final static double ClimberRopeMainGearRatio = 48;
    }

    public final static class ElevatorConstants {

        public final static int ElevatorCAN = 1;

        public final static int ElevatorCurrentLimit = 25;
    }

    public final static class IntakeConstants {

        public final static int IntakeDeploymentCAN = 10;
        public final static int IntakePowerCAN = 13;

        public final static int IntakeDeploymentForwardPCMChannel = 3;
        public final static int IntakeDeploymentReversePCMChannel = 4;

        public final static double IntakeDeploymentOutputGearRatio = 130.67;
        public final static double IntakePowerOutputGearRatio = 9.0;


        public final static double IntakeDeploymentPercentOut = 0.75;

    }

}
