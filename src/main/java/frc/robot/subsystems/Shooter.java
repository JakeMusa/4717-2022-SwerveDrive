// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private TalonFX leftShoot, rightShoot; 
  public static boolean pootornoot = false;
  /** Creates a new Shooter. */
  public Shooter() {
    
    leftShoot = new TalonFX(Constants.LEFT_SHOOT);
    leftShoot.setNeutralMode(NeutralMode.Coast);
    leftShoot.configClosedloopRamp(Constants.shootRamp);
    leftShoot.configOpenloopRamp(Constants.shootRamp);
      
    leftShoot.setInverted(false);

    rightShoot = new TalonFX(Constants.RIGHT_SHOOT);
    rightShoot.setNeutralMode(NeutralMode.Coast);
    rightShoot.configClosedloopRamp(Constants.shootRamp);
    rightShoot.configOpenloopRamp(Constants.shootRamp);
    rightShoot.setInverted(true);
  }

  public void shootBall(double speed){
    leftShoot.set(ControlMode.PercentOutput, speed);
    rightShoot.set(ControlMode.PercentOutput, speed);
  }
  
  public void stopShoot(){
    leftShoot.set(ControlMode.PercentOutput, 0);
    rightShoot.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("left shooter speed", leftShoot.getSelectedSensorVelocity()*600/2048);
    SmartDashboard.putNumber("shooter surface speed", leftShoot.getSelectedSensorVelocity()*60/2048 * Math.PI); 
    SmartDashboard.putBoolean("pooting?", pootornoot); 
    // This method will be called once per scheduler run
  }
}
