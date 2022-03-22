// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.DriveOffLine;
import frc.robot.autos.FireAfter3Sec;
import frc.robot.autos.RevShooter;
import frc.robot.commands.FastIndex;
import frc.robot.commands.Fire;
import frc.robot.commands.IndexBall;
import frc.robot.commands.ReverseIndex;
import frc.robot.commands.ShootBall;
import frc.robot.commands.SwerveJoystickCMD;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubSystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubSystem swerveSubSystem = new SwerveSubSystem(); 
  private final Shooter shooter; 
  private final ShootBall shootBall; 
  private final Index index; 
  private final IndexBall indexBall; 
  private final Fire fire; 
  private final FastIndex fastIndex; 
  private final ReverseIndex reverseIndex; 
  private final DriveOffLine driveOffLine; 
 
  private SendableChooser<Command> autonTaskChooser;

  private XboxController driver = new XboxController(0);

  
 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveSubSystem.setDefaultCommand(new SwerveJoystickCMD(swerveSubSystem, 
    () -> -driver.getRawAxis(1), 
    () -> driver.getRawAxis(0),
    () -> driver.getRawAxis(4),
    () -> !driver.getRightBumper())); 

    shooter = new Shooter(); 
    shootBall = new ShootBall(shooter);
    shootBall.addRequirements(shooter);

    index = new Index();
    indexBall = new IndexBall(index); 
    indexBall.addRequirements(index);
    fire = new Fire(index);
    fire.addRequirements(index);
    fastIndex = new FastIndex(index); 
    fastIndex.addRequirements(index);
    reverseIndex = new ReverseIndex(index);
    reverseIndex.addRequirements(index);

    //uwu i like making things that are unnecessary just for derek 
    driveOffLine = new DriveOffLine(swerveSubSystem);
    driveOffLine.addRequirements(swerveSubSystem);

   
    

    // Configure the button bindings
    configureButtonBindings();
    configureAuton();
  }

  private void configureAuton() {
    autonTaskChooser = new SendableChooser<>();
    autonTaskChooser.addOption("poop ass", new ParallelDeadlineGroup(new FireAfter3Sec(index), new RevShooter(shooter)));
    autonTaskChooser.addOption("drive off line", new DriveOffLine(swerveSubSystem));
    SmartDashboard.putData("auton", autonTaskChooser);
}


  /**x`
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    new JoystickButton(driver, XboxController.Button.kStart.value).whenPressed(() -> swerveSubSystem.zeroHeading()); 

    JoystickButton shootButton = new JoystickButton(driver, XboxController.Button.kA.value);  
    shootButton.toggleWhenPressed(new ShootBall(shooter));  
    
    JoystickButton indexButton = new JoystickButton(driver, XboxController.Button.kX.value);
    indexButton.toggleWhenPressed(new IndexBall(index)); 

    JoystickButton fireButton = new JoystickButton(driver, XboxController.Button.kY.value); 
    fireButton.whileHeld(new Fire(index)); 
    
    JoystickButton fastIndexButton = new JoystickButton(driver, XboxController.Button.kB.value); 
    fastIndexButton.toggleWhenPressed(new FastIndex(index)); 

    JoystickButton reverseIndexButton = new JoystickButton(driver, XboxController.Button.kLeftBumper.value); 
    reverseIndexButton.whileHeld(new ReverseIndex(index)); 

  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous


    return autonTaskChooser.getSelected();
  }
}
