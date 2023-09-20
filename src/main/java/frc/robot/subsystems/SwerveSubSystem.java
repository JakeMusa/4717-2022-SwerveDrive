// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveSubSystem extends SubsystemBase {

    private final SwerveModule frontLeft = new SwerveModule(
        Constants.frontLeft, 
        Constants.a_frontLeft, 
        Constants.driveMotorReverse,    
        Constants.angleMotorFoward); 

    private final SwerveModule frontRight = new SwerveModule(
        Constants.frontRight, 
        Constants.a_frontRight, 
        Constants.driveMotorReverse, 
        Constants.angleMotorFoward); 

    private final SwerveModule backLeft = new SwerveModule(
        Constants.backLeft, 
        Constants.a_backLeft, 
        Constants.driveMotorForward, 
        Constants.angleMotorFoward); 
        
    private final SwerveModule backRight = new SwerveModule(
        Constants.backRight, 
        Constants.a_backRight, 
        Constants.driveMotorForward, 
        Constants.angleMotorFoward); 

        private final Gyro gyro = new AnalogGyro(0); 

     private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.kDriveKinematics, new Rotation2d(0)); 

    public SwerveSubSystem(){
        new Thread(() -> {
            try{
                Thread.sleep(1000);
                zeroHeading();
            }catch(Exception e) {
            }
        }).start();
    }

    public void zeroHeading(){
        gyro.reset();
    }
    public double getHeading(){
        return Math.IEEEremainder(gyro.getAngle(), 360); 
    }
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }
    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(pose, getRotation2d());
    }
    
    @Override
    public void periodic() {
        odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(),
                backRight.getState());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }
    
    public void driveOffLine(){
        frontLeft.driveOffLine1(Constants.autoDriveSpeed);
        frontRight.driveOffLine1(Constants.autoDriveSpeed);
        backLeft.driveOffLine1(Constants.autoDriveSpeed);
        backRight.driveOffLine1(Constants.autoDriveSpeed);
    }

    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleState(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MAXDriveSpeed);
        frontLeft.setDesiredState(desiredStates[0]); 
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

  
}
