// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Feeder;

import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedString;
import com.ma5951.utils.StateControl.Subsystems.StateControlledSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.RobotControl.RobotState;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Feeder.IOs.FeederIO;
import frc.robot.Subsystem.Intake.Intake;
import frc.robot.Subsystem.Intake.IntakeConstants;
import frc.robot.Subsystem.Shooter.Shooter;

public class Feeder extends StateControlledSubsystem {
  private static Feeder feeder;

  private FeederIO feederIO =  FeederConstants.getFeederIO();

  private LoggedBool beamBrakerLog;

  public Feeder() {
    super(FeederConstants.SYSTEM_STATES);
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

  @Override
  public int canMove() {
      if ((RobotState.getInstance().getRobotState() == RobotConstants.INTAKE && Arm.getInstance().atPoint() )||
          (RobotState.getInstance().getRobotState() == RobotConstants.EJECT )||
          (RobotState.getInstance().getRobotState() == RobotConstants.FEEDING && Shooter.getInstance().atPoint() && Arm.getInstance().atPoint()
           && SuperStructure.getInstance().isHeadingForFeeding() )|| 
          (RobotState.getInstance().getRobotState() == RobotConstants.STATIONARY_SHOOTING && Shooter.getInstance().atPoint() && Arm.getInstance().atPoint() 
          && SuperStructure.getInstance().isHeadingForShooting() && !SuperStructure.getInstance().isRobotMoving() ) ||
          (RobotState.getInstance().getRobotState() == RobotConstants.AMP && Arm.getInstance().atPoint()) ||
          (RobotState.getInstance().getRobotState() == RobotConstants.SUBWOOPER_SHOOTING && Shooter.getInstance().atPoint() && Arm.getInstance().atPoint() )||
          (RobotState.getInstance().getRobotState() == RobotConstants.PODIUM_SHOOTING && Shooter.getInstance().atPoint() && Arm.getInstance().atPoint())){
          return 1;
      } else {
          return 0;
      }
  }

  public static Feeder getInstance() {
    if (feeder == null) {
        feeder = new Feeder();  
    }
    return feeder;
  }

  @Override
  public void periodic() {
    super.periodic();
    beamBrakerLog.update(feederIO.getBeamBraker());
  }
}
