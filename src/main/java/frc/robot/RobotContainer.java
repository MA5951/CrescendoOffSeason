// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ma5951.utils.DashBoard.AutoOption;
import com.ma5951.utils.DashBoard.AutoSelector;
import com.ma5951.utils.StateControl.StatesTypes.State;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Arm.ArmConstants;
import frc.robot.Subsystem.Feeder.Feeder;
import frc.robot.Subsystem.Feeder.FeederConstants;
import frc.robot.Subsystem.Intake.Intake;
import frc.robot.Subsystem.Intake.IntakeConstants;
import frc.robot.Subsystem.PoseEstimation.Vision;
import frc.robot.Subsystem.Shooter.Shooter;
import frc.robot.Subsystem.Shooter.ShooterConstants;
import frc.robot.Subsystem.Swerve.SwerveAutoFollower;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;
import frc.robot.commands.Auto.SystemCommands.Eject;
import frc.robot.commands.Auto.SystemCommands.IntakeCommand;
import frc.robot.commands.Auto.SystemCommands.MircozAutomation;
import frc.robot.commands.Auto.SystemCommands.SetArmAngle;
import frc.robot.commands.Auto.SystemCommands.ShootCommand;
import frc.robot.commands.Auto.SystemCommands.ShootCommand2;
import frc.robot.commands.Controllers.IntakeRumble;
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
  public static CommandXboxController driverControllerRumble = new CommandXboxController(2);

  private static AutoSelector autoSelector;

  public RobotContainer() {
    Intake.getInstance();
    Arm.getInstance();
    Feeder.getInstance();
    Shooter.getInstance();
    SwerveSubsystem.getInstance();
    Vision.getInstance();
    configureBindings();
    new SwerveAutoFollower();
    autoSelector = new AutoSelector();
    setUpAutoCommands();
    //new Trigger(() -> oporatorController.getHID().getTriangleButton()).onTrue(new InstantCommand(() -> Arm.getInstance().setTargetState(ArmConstants.AMP)));
    new Trigger(() -> oporatorController.getHID().getCircleButton()).onTrue(new InstantCommand(() -> Arm.getInstance().setTargetState(ArmConstants.HOME)));
    //new Trigger(() -> oporatorController.getHID().getCrossButton()).onTrue(new InstantCommand(() -> Arm.getInstance().setTargetState(ArmConstants.INTAKE)));
  }

  public void setUpAutoCommands() {
    
    NamedCommands.registerCommand("Intake Command", new IntakeCommand());
    NamedCommands.registerCommand("Shoot Command", new ShootCommand2());
    NamedCommands.registerCommand("Eject Command", new Eject());
    NamedCommands.registerCommand("Mircoz Command", new MircozAutomation());
    NamedCommands.registerCommand("Arm Command", new SetArmAngle(() -> RobotConstants.SUPER_STRUCTURE.getShootingPrameters().getArmAngle()));
  
    autoSelector.setAutoOptions(
    new AutoOption[] {
      new AutoOption(new InstantCommand(), "None"),
      new AutoOption("Close 3 Middle", "Middle3"),
      new AutoOption("Midline 2 Amp", "Midline 2")
    }  
    , true);
  }

  public static void update() {
    autoSelector.updateViz();
  }

  public static void setIDLE() {
      lastRobotState = currentRobotState;
      currentRobotState = RobotConstants.IDLE;
      RobotConstants.SUPER_STRUCTURE.isOdometry = false;
      Arm.getInstance().setTargetState(ArmConstants.INTAKE);
      Feeder.getInstance().setTargetState(FeederConstants.IDLE);
      Intake.getInstance().setTargetState(IntakeConstants.IDLE);
      Shooter.getInstance().setTargetState(ShooterConstants.IDLE);
      Vision.getInstance().resetFilter();
  }

  public void setIDLE_AFTER_INTAKE() {
      lastRobotState = currentRobotState;
      currentRobotState = RobotConstants.IDLE;
      RobotConstants.SUPER_STRUCTURE.isOdometry = false;
      Arm.getInstance().setTargetState(ArmConstants.INTAKE);
      Intake.getInstance().setTargetState(IntakeConstants.IDLE);
      Shooter.getInstance().setTargetState(ShooterConstants.IDLE);
      Vision.getInstance().resetFilter();
  }

  public void setINTAKE() {
      lastRobotState = currentRobotState;  
      currentRobotState = RobotConstants.INTAKE;
      Arm.getInstance().setTargetState(ArmConstants.INTAKE);
      Intake.getInstance().setTargetState(IntakeConstants.INTAKING);
      Feeder.getInstance().setTargetState(FeederConstants.FORWARD);
      Shooter.getInstance().setTargetState(ShooterConstants.IDLE);
  }

  public void setEJECT() {
    lastRobotState = currentRobotState;
    currentRobotState = RobotConstants.EJECT;
    Arm.getInstance().setTargetState(ArmConstants.INTAKE);
    Intake.getInstance().setTargetState(IntakeConstants.INTAKING);
    Feeder.getInstance().setTargetState(FeederConstants.FORWARD);
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
    Feeder.getInstance().setTargetState(FeederConstants.REVERSE);
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
    Shooter.getInstance().setTargetState(ShooterConstants.SHOOTING);
    Arm.getInstance().setTargetState(ArmConstants.FOLLOW_SPEAKER);
    Intake.getInstance().setTargetState(IntakeConstants.IDLE);
    Feeder.getInstance().setTargetState(FeederConstants.FORWARD);
    Vision.getInstance().filterSpeaker();
    
  }

  public void setPRESET_SHOOTING() {
    lastRobotState = currentRobotState;
    currentRobotState = RobotConstants.PRESET_SHOOTING;
    Shooter.getInstance().setTargetState(ShooterConstants.PRESET_SHOOTING);
    Arm.getInstance().setTargetState(ArmConstants.PRESET_SHOOTING);
    Intake.getInstance().setTargetState(IntakeConstants.IDLE);
    Feeder.getInstance().setTargetState(FeederConstants.FORWARD);
  }
  
  public static void setDeafultCommands() {
    CommandScheduler.getInstance().setDefaultCommand(Arm.getInstance(), new ArmDeafultCommand());
    CommandScheduler.getInstance().setDefaultCommand(Feeder.getInstance(), new FeederDeafultCommand());
    CommandScheduler.getInstance().setDefaultCommand(Intake.getInstance(), new IntakeDeafultCommand());
    CommandScheduler.getInstance().setDefaultCommand(Shooter.getInstance(), new ShooterDeafultCommand());
    CommandScheduler.getInstance().setDefaultCommand(SwerveSubsystem.getInstance(), new TeleopSwerveController(RobotContainer.driverController));
  }

  public static void disableDeafultCommands() {
    CommandScheduler.getInstance().removeDefaultCommand(Arm.getInstance());
    CommandScheduler.getInstance().removeDefaultCommand(Feeder.getInstance());
    CommandScheduler.getInstance().removeDefaultCommand(Intake.getInstance());
    CommandScheduler.getInstance().removeDefaultCommand(Shooter.getInstance());
    CommandScheduler.getInstance().removeDefaultCommand(SwerveSubsystem.getInstance());
  }

  private void configureBindings() {
    //Start, stop  and inturupt intake
    new Trigger(() -> driverController.getHID().getR1Button() && !RobotConstants.SUPER_STRUCTURE.isNote()).onTrue(new InstantCommand(() -> setINTAKE()));
    
    new Trigger(() -> currentRobotState == RobotConstants.INTAKE && RobotConstants.SUPER_STRUCTURE.isNoteInShooter()).onTrue(new InstantCommand(() -> Feeder.getInstance().setTargetState(FeederConstants.NOTE_ADJUSTING))
    .andThen(new InstantCommand(() -> setIDLE_AFTER_INTAKE())));

    new Trigger(() -> RobotConstants.SUPER_STRUCTURE.isNoteInFeeder() && currentRobotState == RobotConstants.INTAKE).onTrue(
      new IntakeRumble()
    );

    //Start, stop and inturupt amp
    new Trigger(() -> driverController.getHID().getL2Button() && currentRobotState != RobotConstants.AMP && currentRobotState != RobotConstants.INTAKE && RobotConstants.SUPER_STRUCTURE.isNote()
    && Feeder.getInstance().getTargetState() !=  FeederConstants.NOTE_ADJUSTING).onTrue(new InstantCommand(() -> setAMP()));
    
    new Trigger(() -> currentRobotState == RobotConstants.AMP && !RobotConstants.SUPER_STRUCTURE.isNote()
    && RobotConstants.SUPER_STRUCTURE.shouldCloseArmAfterAmp() && !driverController.getHID().getL2Button()).onTrue(new InstantCommand(() -> setIDLE()));

    //Start and stop warm up //add not amp
    new Trigger(() -> RobotConstants.SUPER_STRUCTURE.isInWarmUpZone() && RobotConstants.SUPER_STRUCTURE.isNoteInShooter() && currentRobotState != RobotConstants.SOURCE_INTAKE
    && currentRobotState != RobotConstants.INTAKE && Feeder.getInstance().getTargetState() ==  FeederConstants.IDLE).onTrue(new InstantCommand(() -> setWARMING()));
    new Trigger(() -> (!RobotConstants.SUPER_STRUCTURE.isInWarmUpZone() || !RobotConstants.SUPER_STRUCTURE.isNoteInShooter()) && currentRobotState == RobotConstants.WARMING ).onTrue(new InstantCommand(() -> setIDLE()));

    //Starts and stops Shooting 
    new Trigger(() -> driverController.getHID().getL1Button() && currentRobotState != RobotConstants.SOURCE_INTAKE && Feeder.getInstance().getTargetState() !=  FeederConstants.NOTE_ADJUSTING
    && RobotConstants.SUPER_STRUCTURE.getDistanceToTag() <= RobotConstants.DISTANCE_TO_SHOOT).onTrue(new InstantCommand(() -> setSTATIONARY_SHOOTING()));
    
    new Trigger(() -> currentRobotState == RobotConstants.STATIONARY_SHOOTING && !RobotConstants.SUPER_STRUCTURE.isNoteInShooter()).onTrue(new InstantCommand(() -> setIDLE()));

    //Preset Shooting
    new Trigger(() -> driverController.getHID().getPOV() == 90 && RobotConstants.SUPER_STRUCTURE.isNoteInShooter() && Feeder.getInstance().getTargetState() !=  FeederConstants.NOTE_ADJUSTING)
    .onTrue(new InstantCommand(() -> RobotConstants.SUPER_STRUCTURE.setPRESETParameters(RobotConstants.PODIUM_SHOOTING_PARAMETERS))
    .andThen(new InstantCommand(() -> setPRESET_SHOOTING())));
    
    new Trigger(() -> driverController.getHID().getPOV() == 180 && RobotConstants.SUPER_STRUCTURE.isNoteInShooter() && Feeder.getInstance().getTargetState() !=  FeederConstants.NOTE_ADJUSTING)
    .onTrue(new InstantCommand(() -> RobotConstants.SUPER_STRUCTURE.setPRESETParameters(RobotConstants.SUBWOOF_SHOOTING_PARAMETERS))
    .andThen(new InstantCommand(() -> setPRESET_SHOOTING())));
    
    new Trigger(() -> currentRobotState == RobotConstants.PRESET_SHOOTING && !RobotConstants.SUPER_STRUCTURE.isNoteInShooter()).onTrue(new InstantCommand(() -> setIDLE()));

    //Ejecting while held
    new Trigger(() -> driverController.getHID().getCrossButton() )
    .onTrue(new InstantCommand(() -> setEJECT())).onFalse(new InstantCommand(() -> setIDLE()));

    // //IDLE
    new Trigger(() -> driverController.getHID().getTouchpad()).onTrue(new InstantCommand(() -> setIDLE()));

    //Feeding?
    new Trigger(() -> driverController.getHID().getPOV() == 270 && currentRobotState != RobotConstants.SOURCE_INTAKE)
    .onTrue(new InstantCommand(() -> RobotConstants.SUPER_STRUCTURE.setPRESETParameters(RobotConstants.FEEDING_SHOOTING_PARAMETERS.get()))
    .andThen(new InstantCommand(() -> setPRESET_SHOOTING())));

    // //Low Feed

    //Update Offset
    new Trigger(() -> driverController.getHID().getTriangleButton()).onTrue(new InstantCommand(() -> SwerveSubsystem.getInstance().updateOffset()));

    new Trigger(() -> driverController.getHID().getOptionsButton() && Shooter.getInstance().getLeftSpeed() < ShooterConstants.SOURCE_INTAKE_SPEED_LIMIT &&
    Shooter.getInstance().getRightSpeed() < ShooterConstants.SOURCE_INTAKE_SPEED_LIMIT && Feeder.getInstance().getTargetState() !=  FeederConstants.NOTE_ADJUSTING).onTrue(new InstantCommand(() -> setSOURCE_INTAKE()));
    
    new Trigger(() -> currentRobotState == RobotConstants.SOURCE_INTAKE && RobotConstants.SUPER_STRUCTURE.isNoteInFeeder() 
    && !RobotConstants.SUPER_STRUCTURE.isNoteInShooter()).onTrue(new InstantCommand(() -> setIDLE())
     .andThen(new InstantCommand(() -> Feeder.getInstance().setTargetState(FeederConstants.NOTE_ADJUSTING))));

    // new Trigger(() -> driverController.getHID().getR2Button()).whileTrue(
    //   new InstantCommand(() -> SwerveSubsystem.getInstance().setCurrentLimits(SwerveConstants.Slow40Precent))
    // ).whileFalse(new InstantCommand(() -> SwerveSubsystem.getInstance().setCurrentLimits(SwerveConstants.DEFUALT)));

    //Open Arm
    new Trigger(() -> oporatorController.getHID().getR1Button()).onTrue(new InstantCommand(() -> Arm.getInstance().setTargetState(ArmConstants.AMP)));
    new Trigger(() -> oporatorController.getHID().getTouchpad()).onTrue(new InstantCommand(() -> setIDLE()));
  
  }

  public Command getAutonomousCommand() {
    return autoSelector.getSelectedAutoCommand();
  }
}
