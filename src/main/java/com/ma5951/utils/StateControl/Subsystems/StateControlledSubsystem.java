// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.StateControl.Subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.ma5951.utils.StateControl.StatesTypes.State;
import com.ma5951.utils.StateControl.StatesTypes.StatesConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public abstract class StateControlledSubsystem extends SubsystemBase {

    private List systemStatesList;
    private int systemCanMove = 1;
    private State systemFunctionState = StatesConstants.AUTOMATIC;
    private State targetState;
    private State lastState;
    private SubsystemStateMeachin subsystemStateMeachin;

    public StateControlledSubsystem(State[] states) {
        systemStatesList = Arrays.asList(states);

    }

    public void setStateMeachin(SubsystemStateMeachin meachin) {
        subsystemStateMeachin = meachin;
        subsystemStateMeachin.setSubsystem(this);
    }

    public SubsystemStateMeachin getStateMeachin() {
        return subsystemStateMeachin;
    }

    public void setSystemFunctionState(State FunctioState) {
        systemFunctionState = FunctioState;
    }

    public State getSystemFunctionState() {
        return systemFunctionState;
    }

    //add get posiible states as arry

    public State getTargetState() {
        return targetState;
    }

    public State getLastState() {
        return lastState;
    }

    public void setTargetState(State state) {
        if (systemStatesList.contains(state)) {
            lastState = targetState;
            targetState = state;
        } else {
            System.err.println("Can set target state for " + getSubsystem());
        }
    }

    public int canMove() {
        return systemCanMove;
    }

    public State getCurrenState() {
        return targetState;
    }

   @Override
   public void periodic() {
       //Print States 
       //Dashboard cahnge to manuel / automatic
   }

}
