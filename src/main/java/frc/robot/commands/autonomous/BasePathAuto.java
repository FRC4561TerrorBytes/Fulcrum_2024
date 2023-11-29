// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Base of a path planner auto with only driving
 */
public class BasePathAuto {

  DriveSubsystem m_driveSubsystem;
  PathPlannerPath m_pathPlannerPath;
  Map<String, Command> m_eventMap = new HashMap<>();
  String autoPathName = "";

  /**
   * Creates a new PathPlanner trajectory for swerve modules to follow in
   * autonomous
   * 
   * @param driveSubsystem
   * @param autoPathName
   * @param maxSpeedMetersPerSec
   * @param maxAccelerationMetersPerSecSquared
   */
  public BasePathAuto(DriveSubsystem driveSubsystem, String autoPathName, double maxSpeedMetersPerSec,
      double maxAccelerationMetersPerSecSquared) {
    this.m_driveSubsystem = driveSubsystem;
    this.autoPathName = autoPathName;
  }

  protected void setEventMap(Map<String, Command> eventMap){
    m_eventMap = eventMap;
  }

  /**
   * 
   * @return returns a path with events for the robot to follow
   */
  public Command getCommandAndStop() {
    return m_driveSubsystem.followTrajectoryCommand(autoPathName);
  }
}