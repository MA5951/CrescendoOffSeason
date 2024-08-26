// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Feeder;

import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedString;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystem.Feeder.IOs.FeederIO;

public class Feeder extends SubsystemBase {
  private static Feeder feeder;

  private FeederIO feederIO =  FeederConstants.getFeederIO();

  private LoggedBool beamBrakerLog;

  public Feeder() {
    feederIO.setNutralMode(true);
    beamBrakerLog = new LoggedBool("/Subsystems/Feeder/Is Note");
  }

  public void isNoteInFeeder() {
    feederIO.getBeamBraker();
  }

  public void turnOnFeeder() {
    feederIO.setVoltage(FeederConstants.FEEDER_POWER);
  }

  public void turnOnEjectFeeder() {
    feederIO.setVoltage(FeederConstants.EJECT_POWER);
  }

  public void turnOffFeeder() {
    feederIO.setVoltage(0);
  }

  public static Feeder getInstance() {
    if (feeder == null) {
        feeder = new Feeder();  
    }
    return feeder;
  }

  @Override
  public void periodic() {
    beamBrakerLog.update(feederIO.getBeamBraker());
  }
}
