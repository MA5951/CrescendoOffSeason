// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public class PortMap {

    public static class CanBus {
        public static final String CANivoreBus = "Swerve";
        public static final String RioBus = "rio";
    }
    
    public static class Controllers {
        public static final int driveID = 0;
        public static final int operatorID = 1;    
    }
    

    public class Swerve {
        public static final int leftFrontAbsoluteEncoder = 22;
        public static final int leftFrontDriveID = 8;
        public static final int leftFrontTurningID = 5;

        public static final int leftBackAbsoluteEncoder = 21;
        public static final int leftBackDriveID = 4;
        public static final int leftBackTurningID = 9;

        public static final int rightFrontAbsoluteEncoder = 23;
        public static final int rightFrontDriveID = 7;
        public static final int rightFrontTurningID = 6;

        public static final int rightBackAbsoluteEncoder = 24;
        public static final int rightBackDriveID = 2;
        public static final int rightBackTurningID = 3;

        public static final int Pigeon2ID = 12;
    }

    public class Intake {
        public static final int KrakenIntakeMotor  = 14;
    }

    public class Feeder {
        public static final int KrakenFeederMotor = 18;
        public static final int DIO_FeederSensor = 0;
    }

    public class Shooter {
        public static final int FalconLeftMotor = 0;
        public static final int FalconRightMotor = 0;
        public static final int DIO_ShooterSensor = 1;
    }

    public class Arm {
        public static final int KrakenArmMotor  = 15;
    }

    public class Vision {
        public final static String CAMERA_NAME = "CAMERA";
    }

    public class Leds {
        public static final int ledPort = 9; 
    }

}
