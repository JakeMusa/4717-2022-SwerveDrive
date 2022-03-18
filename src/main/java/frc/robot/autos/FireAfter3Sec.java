// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Index;

public class FireAfter3Sec extends CommandBase {
  private final Index index; 
  private boolean shot;
  private Timer shottimer;

  /** Creates a new FireAfter3Sec. */
  public FireAfter3Sec(Index i) {
    index = i;
    addRequirements(index);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      shottimer = new Timer();
      shottimer.reset();
      shot = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shottimer.start();
    if(shottimer.get() > 4.2){
      shot = true;
    }
    else if(shottimer.get()>3){
      index.fire(0.6);
      index.indexBall(0.3);
    }
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    index.stopIndex();

  }

  // Returns true when the command   should end.
  @Override
  public boolean isFinished() {
    return shot;
  }
}
