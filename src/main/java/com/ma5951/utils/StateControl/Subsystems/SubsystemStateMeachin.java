// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.StateControl.Subsystems;

import com.ma5951.utils.StateControl.RobotState.RobotStates;

public abstract class SubsystemStateMeachin {

    public StateControlledSubsystem subsystem;
    public void setSubsystem(StateControlledSubsystem system) {
        subsystem = system;
    }

    public void SystemLoop() {
        switch (RobotStates.getRobotEnableState().getName()) {
            case "ROBOT_ENABLE":
                switch (subsystem.getSystemEnablState().getName()) {
                    case "SYSTEM_ENABLE":
                        switch (RobotStates.getRobotFunctionState().getName()) {
                            case "TELEOP":
                                TeleopLoop();
                                break;
                            case "AUTO":
                                AutoLoop();
                                break;
                            case "TEST":
                                TestLoop();
                                break;
                        }
                        break;
                
                    default:
                        //System Disable
                        break;
                }
                break;
            default:
                //Robot Disable
                break;
        }
    }

    public void TeleopLoop() {
        switch (subsystem.getSystemFunctionState().getName()) {
            case "AUTOMATIC":
                AutomaticLoop();
                break;
        
            case "MANUEL":
                ManuelLoop();
                break;
        }
    }

    public abstract void AutoLoop();

    public abstract void TestLoop() ;

    public abstract void AutomaticLoop(); 

    public abstract void ManuelLoop() ;

}
