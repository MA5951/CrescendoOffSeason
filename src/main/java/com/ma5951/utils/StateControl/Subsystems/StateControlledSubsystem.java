// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.StateControl.Subsystems;

import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import com.ma5951.utils.StateControl.StatesTypes.State;
import com.ma5951.utils.StateControl.StatesTypes.StatesConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public abstract class StateControlledSubsystem extends SubsystemBase {

    private State[] systemStates;
    private List<State> systemStatesList;
    private State systemEnableState = StatesConstants.CANT_MOVE;
    private State systemFunctionState = StatesConstants.AUTOMATIC;
    private State targetState;
    private State travelState;
    private Supplier<Boolean> measurmentSupplier;
    private SubsystemStateMeachin subsystemStateMeachin;

    public void setSystemStates(State[] states ,State travelState ,Supplier<Boolean> supplier) {
        systemStates = states;
        systemStatesList = Arrays.asList(systemStates);
        this.travelState = travelState;
        measurmentSupplier = supplier;
    }

    public void setStateMeachin(SubsystemStateMeachin meachin) {
        subsystemStateMeachin = meachin;
        subsystemStateMeachin.setSubsystem(this);
    }

    public SubsystemStateMeachin getStateMeachin() {
        return subsystemStateMeachin;
    }

    public void disableSystem() {
        systemEnableState = StatesConstants.SYSTEM_DISABLED;
    }

    public void enableSystem() {
        systemEnableState = StatesConstants.SYSTEM_ENABLE;
    }

    public State getSystemEnablState() {
        return systemEnableState;
    }

    public void setSystemFunctionState(State FunctioState) {
        systemFunctionState = FunctioState;
    }

    public State getSystemFunctionState() {
        return systemFunctionState;
    }

    public State[] getSystemStates() {
        return systemStates;
    }

    public State getTargetState() {
        return targetState;
    }

    public void setTargetState(State state) {
        if (systemStatesList.contains(state)) {
            targetState = state;
        } else {
            System.err.println("Can set target state for " + getSubsystem());
        }
    }

    public boolean canMove() {
        return false;
    }

    public State getCurrenState() {
        if (measurmentSupplier.get()) {
            return targetState;
        } else {
            return travelState;
        }
    }

}
