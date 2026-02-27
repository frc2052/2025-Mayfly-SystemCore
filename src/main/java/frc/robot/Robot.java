// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autos.AutoChooser;
import frc.robot.util.FieldConstants;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final AutoChooser autoChooser;

  public Robot() {
    Logger.recordMetadata("ProjectName", "Mayfly-Systemcore"); // Set a metadata value

    if (isReal()) {
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else {
        setUseTiming(false); // Run as fast as possible
    }
    
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start(); 
    m_robotContainer = new RobotContainer();
    Pose2d loadPose = FieldConstants.blueLeftBranches.get(0);
    if (loadPose != null) {
        System.out.println("Loaded Field Constants");
    }

    autoChooser = AutoChooser.create(m_robotContainer);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    RobotState.getInstance().output();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    autoChooser.update();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    Command auto = autoChooser.getAuto();
    if(auto != null){
      CommandScheduler.getInstance().schedule(auto);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
