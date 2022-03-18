// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubSystem;

public class SwerveJoystickCMD extends CommandBase {

    private final SwerveSubSystem swerveSubSystem; 

    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction; 

   // private final Supplier<Boolean> fieldOrientedFunction;
    

  public SwerveJoystickCMD(SwerveSubSystem swerveSubsystem, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, 
    Supplier<Double> turningSpdFunction) {
      this.swerveSubSystem = swerveSubsystem; 
      this.xSpdFunction = xSpdFunction; 
      this.ySpdFunction = ySpdFunction; 
      this.turningSpdFunction = turningSpdFunction; 
     // this.fieldOrientedFunction = fieldOrientedFunction;
      addRequirements(swerveSubsystem);
  }

  
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get(); 

    xSpeed = Math.abs(xSpeed) > Constants.Deadband ? xSpeed : 0; 
    ySpeed = Math.abs(ySpeed) > Constants.Deadband ? ySpeed : 0; 
    turningSpeed = Math.abs(turningSpeed) > Constants.Deadband ? turningSpeed : 0; 
    
    ChassisSpeeds chassisSpeeds;
    // if (fieldOrientedFunction.get()) {
    //   // Relative to field
    //   chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
    //           xSpeed, ySpeed, turningSpeed);
    // } else {
    //   // Relative to robot
    //   chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    // }
    chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);

    SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds); 

    swerveSubSystem.setModuleState(moduleStates);
    
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubSystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
