// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SubsystemsDeafultCommands;

import com.ma5951.utils.StateControl.Commands.CanMoveCommand;
import com.ma5951.utils.StateControl.RobotState.RobotStates;
import com.ma5951.utils.StateControl.Subsystems.StateControlledSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class FeederDeafultCommand extends CanMoveCommand {
  /** Creates a new RobotFunctionStatesCommand. */
  private StateControlledSubsystem subsystem;
  
  public FeederDeafultCommand(StateControlledSubsystem subsystem) {
    super(subsystem);
    this.subsystem = super.subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void CAN_MOVE() {
    super.CAN_MOVE();
    switch (RobotStates.getRobotFunctionState().getName()) {
                            case "TELEOP":
                              switch (subsystem.getSystemFunctionState().getName()) {
                                case "AUTOMATIC":
                                    AutomaticLoop();
                                    break;
                            
                                case "MANUEL":
                                    ManuelLoop();
                                    break;
                            }
                                break;
                            case "AUTO":
                                AutoLoop();
                                break;
                            case "TEST":
                                TestLoop();
                                break;
    }
  }

  @Override
  public void CANT_MOVE() {
      super.CANT_MOVE();
  }

  public void AutomaticLoop() {
    switch (subsystem.getCurrenState().getName()) {
      case "IDLE":
        
        break;
      case "FEEDING":
        
        break;
      case "EJECTING":
          
        break;
      default:
        break;
    }
  }

  public void ManuelLoop() {

  }

  public void AutoLoop() {
    
  }

  public void TestLoop() {
    
  }


  
}
