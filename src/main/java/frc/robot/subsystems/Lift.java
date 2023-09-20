// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lift extends SubsystemBase {
  private VictorSP liftMotor; 
  /** Creates a new Lift. */
  public Lift() {
    liftMotor = new VictorSP(3); 
  }

  public void lift(){
    liftMotor.set(Constants.liftSpeed);
  }
  public void stopLift(){
    liftMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
