// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotControl;

import com.ma5951.utils.StateControl.StatesTypes.State;

import frc.robot.RobotConstants;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Arm.ArmConstants;
import frc.robot.Subsystem.Feeder.Feeder;
import frc.robot.Subsystem.Feeder.FeederConstants;
import frc.robot.Subsystem.Intake.Intake;
import frc.robot.Subsystem.Intake.IntakeConstants;
import frc.robot.Subsystem.Shooter.Shooter;
import frc.robot.Subsystem.Shooter.ShooterConstants;

/** Add your docs here. */
public class RobotState {
    public static  RobotState robotStates;
    private static State currentRobotState = RobotConstants.IDLE;
    private static State lastRobotState = RobotConstants.IDLE;

    public void setRobotState(State state) {
        lastRobotState = currentRobotState;
        currentRobotState = state;

        switch (state.getName()) {
            case "IDLE":
                setIDLE();
                break;
        
            case "INTAKE":
                setINTAKE();
                break;

            case "EJECT":
                setEJECT();
                break;

            case "WARMING":
                setWARMING();
                break;

            case "AMP":
                setAMP();
                break;

            case "FEEDING":
                setFEEDING();
                break;

            case "SOURCE_INTAKE":
                setSOURCE_INTAKE();
                break;

            case "STATIONARY_SHOOTING":
                setSTATIONARY_SHOOTING();
                break;

            case "PODIUM_SHOOTING":
                setPODIUM_SHOOTING();
                break;
            
            case "SUBWOOPER_SHOOTING":
                setSUBWOOPER_SHOOTING();
                break;

            case "HOME":
                setHOME();
                break;

            default:
                break;
        }
    }

    public void setIDLE() {
        Arm.getInstance().setTargetState(ArmConstants.IDLE);
        Feeder.getInstance().setTargetState(FeederConstants.IDLE);
        Intake.getInstance().setTargetState(IntakeConstants.IDLE);
        Shooter.getInstance().setTargetState(ShooterConstants.IDLE);
    }

    public void setINTAKE() {
        Arm.getInstance().setTargetState(ArmConstants.INTAKE);
        Intake.getInstance().setTargetState(IntakeConstants.INTAKING);
        Feeder.getInstance().setTargetState(FeederConstants.FEEDING);
        Shooter.getInstance().setTargetState(ShooterConstants.IDLE);
    }

    public void setEJECT() {
        Arm.getInstance().setTargetState(ArmConstants.IDLE);
        Intake.getInstance().setTargetState(IntakeConstants.EJECTING);
        Feeder.getInstance().setTargetState(FeederConstants.EJECTING);
        Shooter.getInstance().setTargetState(ShooterConstants.EJECTING);
    }

    public void setWARMING() {
        Arm.getInstance().setTargetState(ArmConstants.FOLLOW_SPEAKER);
        Intake.getInstance().setTargetState(IntakeConstants.IDLE);
        Feeder.getInstance().setTargetState(FeederConstants.IDLE);
        Shooter.getInstance().setTargetState(ShooterConstants.WARM);
    }

    public void setAMP() {
        Arm.getInstance().setTargetState(ArmConstants.AMP);
        Intake.getInstance().setTargetState(IntakeConstants.IDLE);
        Feeder.getInstance().setTargetState(FeederConstants.FEEDING);
        Shooter.getInstance().setTargetState(ShooterConstants.IDLE);
    }

    public void setFEEDING() {
        Arm.getInstance().setTargetState(ArmConstants.FEEDING);
        Intake.getInstance().setTargetState(IntakeConstants.IDLE);
        Feeder.getInstance().setTargetState(FeederConstants.FEEDING);
        Shooter.getInstance().setTargetState(ShooterConstants.FEEDING);
    }

    public void setSOURCE_INTAKE() {
        Arm.getInstance().setTargetState(ArmConstants.SOURCE_INTAKE);
        Intake.getInstance().setTargetState(IntakeConstants.IDLE);
        Feeder.getInstance().setTargetState(FeederConstants.EJECTING);
        Shooter.getInstance().setTargetState(ShooterConstants.SOURCE_INTAKE);
    }

    public void setSTATIONARY_SHOOTING() {
        Arm.getInstance().setTargetState(ArmConstants.FOLLOW_SPEAKER);
        Intake.getInstance().setTargetState(IntakeConstants.IDLE);
        Feeder.getInstance().setTargetState(FeederConstants.FEEDING);
        Shooter.getInstance().setTargetState(ShooterConstants.SHOOTING);
    }

    public void setPODIUM_SHOOTING() {
        Arm.getInstance().setTargetState(ArmConstants.FEEDING);
        Intake.getInstance().setTargetState(IntakeConstants.IDLE);
        Feeder.getInstance().setTargetState(FeederConstants.FEEDING);
        Shooter.getInstance().setTargetState(ShooterConstants.FEEDING);
    }

    public void setSUBWOOPER_SHOOTING() {
        Arm.getInstance().setTargetState(ArmConstants.FOLLOW_SPEAKER);
        Intake.getInstance().setTargetState(IntakeConstants.IDLE);
        Feeder.getInstance().setTargetState(FeederConstants.FEEDING);
        Shooter.getInstance().setTargetState(ShooterConstants.FEEDING);
    }

    public void setHOME() {
        Arm.getInstance().setTargetState(ArmConstants.HOME);
        Intake.getInstance().setTargetState(IntakeConstants.IDLE);
        Feeder.getInstance().setTargetState(FeederConstants.IDLE);
        Shooter.getInstance().setTargetState(ShooterConstants.IDLE);
    }

    public State getRobotState() {
        return currentRobotState;
    }

    public State getLastRobotState() {
        return lastRobotState;
    }

    public static RobotState getInstance() {
        if (robotStates == null) {
            robotStates = new RobotState();
        }
        return robotStates;
    }
}
