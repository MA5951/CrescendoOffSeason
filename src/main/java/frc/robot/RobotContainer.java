// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ma5951.utils.StateControl.StatesTypes.State;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Arm.ArmConstants;
import frc.robot.Subsystem.Feeder.Feeder;
import frc.robot.Subsystem.Feeder.FeederConstants;
import frc.robot.Subsystem.Intake.Intake;
import frc.robot.Subsystem.Intake.IntakeConstants;
import frc.robot.Subsystem.PoseEstimation.SwervePoseCalculator;
import frc.robot.Subsystem.Shooter.Shooter;
import frc.robot.Subsystem.Shooter.ShooterConstants;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;
import frc.robot.commands.DeafultCommands.ArmDeafultCommand;
import frc.robot.commands.DeafultCommands.FeederDeafultCommand;
import frc.robot.commands.DeafultCommands.IntakeDeafultCommand;
import frc.robot.commands.DeafultCommands.ShooterDeafultCommand;
import frc.robot.commands.Swerve.TeleopSwerveController;

public class RobotContainer {
  public static State currentRobotState = RobotConstants.IDLE;
  public static State lastRobotState = currentRobotState;

  public static CommandPS5Controller driverController = new CommandPS5Controller(PortMap.Controllers.driveID);
  public static CommandPS5Controller oporatorController = new CommandPS5Controller(PortMap.Controllers.operatorID);

  public RobotContainer() {
    Intake.getInstance();
    Arm.getInstance();
    Feeder.getInstance();
    Shooter.getInstance();
    SwerveSubsystem.getInstance();
    SwervePoseCalculator.getInstance();
    SuperStructure.setupInterpolation();
    setDeafultCommands();
    configureBindings();
    
  }

  public void setIDLE() {
      lastRobotState = currentRobotState;
      currentRobotState = RobotConstants.IDLE;
      Arm.getInstance().setTargetState(ArmConstants.HOME);
      Feeder.getInstance().setTargetState(FeederConstants.IDLE);
      Intake.getInstance().setTargetState(IntakeConstants.IDLE);
      Shooter.getInstance().setTargetState(ShooterConstants.IDLE);
  }

  public void setINTAKE() {
      currentRobotState = RobotConstants.INTAKE;
      //TODO add last state
      Arm.getInstance().setTargetState(ArmConstants.INTAKE);
      Intake.getInstance().setTargetState(IntakeConstants.INTAKING);
      Feeder.getInstance().setTargetState(FeederConstants.FORWARD);
      Shooter.getInstance().setTargetState(ShooterConstants.IDLE);
  }

  public void setEJECT() {
    lastRobotState = currentRobotState;
    currentRobotState = RobotConstants.EJECT;
    Arm.getInstance().setTargetState(ArmConstants.IDLE);
    Intake.getInstance().setTargetState(IntakeConstants.EJECTING); //idel 
    Feeder.getInstance().setTargetState(FeederConstants.REVERSE); //TODO change to fowerd
    Shooter.getInstance().setTargetState(ShooterConstants.EJECTING);
  }

  public void setWARMING() {
    lastRobotState = currentRobotState;
    currentRobotState = RobotConstants.WARMING;
    Arm.getInstance().setTargetState(ArmConstants.FOLLOW_SPEAKER);
    Intake.getInstance().setTargetState(IntakeConstants.IDLE);
    Feeder.getInstance().setTargetState(FeederConstants.IDLE);
    Shooter.getInstance().setTargetState(ShooterConstants.WARM);
  }

  public void setAMP() {
    lastRobotState = currentRobotState;
    currentRobotState = RobotConstants.AMP;
    Arm.getInstance().setTargetState(ArmConstants.AMP);
    Intake.getInstance().setTargetState(IntakeConstants.IDLE);
    Feeder.getInstance().setTargetState(FeederConstants.FORWARD); //TODO Cahge to idel
    Shooter.getInstance().setTargetState(ShooterConstants.IDLE);
  }

  public void setFEEDING() {
    lastRobotState = currentRobotState;
    currentRobotState = RobotConstants.FEEDING;
    Arm.getInstance().setTargetState(ArmConstants.FEEDING);
    Intake.getInstance().setTargetState(IntakeConstants.IDLE);
    Feeder.getInstance().setTargetState(FeederConstants.FORWARD);
    Shooter.getInstance().setTargetState(ShooterConstants.FEEDING);
  }

  public void setSOURCE_INTAKE() {
    lastRobotState = currentRobotState;
    currentRobotState = RobotConstants.SOURCE_INTAKE;
    Arm.getInstance().setTargetState(ArmConstants.SOURCE_INTAKE);
    Intake.getInstance().setTargetState(IntakeConstants.IDLE);
    Feeder.getInstance().setTargetState(FeederConstants.REVERSE);
    Shooter.getInstance().setTargetState(ShooterConstants.SOURCE_INTAKE);
  }

  public void setSTATIONARY_SHOOTING() {
    lastRobotState = currentRobotState;
    currentRobotState = RobotConstants.STATIONARY_SHOOTING;
    Arm.getInstance().setTargetState(ArmConstants.FOLLOW_SPEAKER);
    Intake.getInstance().setTargetState(IntakeConstants.IDLE);
    Feeder.getInstance().setTargetState(FeederConstants.FORWARD);
    Shooter.getInstance().setTargetState(ShooterConstants.SHOOTING);
  }

  public void setPRESET_SHOOTING() {
    lastRobotState = currentRobotState;
    currentRobotState = RobotConstants.PRESET_SHOOTING;
    Arm.getInstance().setTargetState(ArmConstants.FEEDING);
    Intake.getInstance().setTargetState(IntakeConstants.IDLE);
    Feeder.getInstance().setTargetState(FeederConstants.FORWARD);
    Shooter.getInstance().setTargetState(ShooterConstants.FEEDING); //TODO change to the right state
  }
  
  private void setDeafultCommands() {
    //TODO while you set the defult command
    CommandScheduler.getInstance().setDefaultCommand(Arm.getInstance(), new ArmDeafultCommand());
    CommandScheduler.getInstance().setDefaultCommand(Feeder.getInstance(), new FeederDeafultCommand());
    CommandScheduler.getInstance().setDefaultCommand(Intake.getInstance(), new IntakeDeafultCommand());
    CommandScheduler.getInstance().setDefaultCommand(Shooter.getInstance(), new ShooterDeafultCommand());
    CommandScheduler.getInstance().setDefaultCommand(SwerveSubsystem.getInstance(), new TeleopSwerveController(RobotContainer.driverController));
  }

  private void configureBindings() {
    //Start, stop  and inturupt intake
    new Trigger(() -> currentRobotState == RobotConstants.INTAKE && driverController.getHID().getL1Button()).onTrue(new InstantCommand(() -> setIDLE()));
    new Trigger(() -> driverController.getHID().getL1Button() && !SuperStructure.isNote()).onTrue(new InstantCommand(() -> setINTAKE()));
    new Trigger(() -> currentRobotState == RobotConstants.INTAKE && SuperStructure.isNoteInShooter()).onTrue(new InstantCommand(() -> setIDLE()));


    //Start, stop and inturupt amp
    new Trigger(() -> currentRobotState == RobotConstants.AMP && driverController.getHID().getL1Button()).onTrue(new InstantCommand(() -> setIDLE()));
    new Trigger(() -> driverController.getHID().getCircleButton() && currentRobotState != RobotConstants.AMP && SuperStructure.isNote()).onTrue(new InstantCommand(() -> setAMP()));
    new Trigger(() -> currentRobotState == RobotConstants.AMP && driverController.getHID().getCircleButton() && !SuperStructure.isNote()).onTrue(new InstantCommand(() -> setIDLE()));

    //Start and stop warm up //add not amp
    new Trigger(() -> SuperStructure.isInWarmUpZone() && SuperStructure.isNote() && currentRobotState != RobotConstants.SOURCE_INTAKE).onTrue(new InstantCommand(() -> setWARMING()));
    new Trigger(() -> !SuperStructure.isInWarmUpZone() && currentRobotState == RobotConstants.WARMING).onTrue(new InstantCommand(() -> setIDLE()));

    //Starts Shooting 
    new Trigger(() -> driverController.getHID().getR1Button() && currentRobotState != RobotConstants.SOURCE_INTAKE).onTrue(new InstantCommand(() -> setSTATIONARY_SHOOTING()));
    //Shooting turn off in feeder

    //Preset Shooting
    new Trigger(() -> driverController.getHID().getPOV() == -90 && currentRobotState != RobotConstants.SOURCE_INTAKE)
    .onTrue(new InstantCommand(() -> SuperStructure.setPRESETParameters(RobotConstants.PODIUM_SHOOTING_PARAMETERS))
    .andThen(new InstantCommand(() -> setPRESET_SHOOTING())));
    new Trigger(() -> driverController.getHID().getPOV() == 180 && currentRobotState != RobotConstants.SOURCE_INTAKE)
    .onTrue(new InstantCommand(() -> SuperStructure.setPRESETParameters(RobotConstants.SUBWOOF_SHOOTING_PARAMETERS))
    .andThen(new InstantCommand(() -> setPRESET_SHOOTING())));
    //Shooting turn off in feeder

    //Ejecting while held
    new Trigger(() -> driverController.getHID().getCrossButton() && currentRobotState != RobotConstants.SOURCE_INTAKE && SuperStructure.isNote())
    .onTrue(new InstantCommand(() -> setEJECT())).onFalse(new InstantCommand(() -> setIDLE()));


  }

  public Command getAutonomousCommand() {
    return null; //TODO add auto commands 
  }
}
