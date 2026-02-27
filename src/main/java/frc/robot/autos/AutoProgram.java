// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Function;

// AutoProgram connects the Auto enum to the AutoFactory actions
// AutoProgram also allows access to auto details like the start pose & commands
// so we can generate red & blue side autos

public class AutoProgram {
  private final Autos auto;
  private final Function<AutoFactory, Pair<Pose2d, Command>> command;

  public AutoProgram(Autos auto, Function<AutoFactory, Pair<Pose2d, Command>> command) {
    this.auto = auto;
    this.command = command;
  }

  public Autos getAuto() {
    return auto;
  }

  public Pose2d getStartPose(AutoFactory allianceFactory) {
    return command.apply(allianceFactory).getFirst();
  }

  public Command getAutoRoutine(AutoFactory allianceFactory) {
    return command.apply(allianceFactory).getSecond();
  }

  public Pair<Pose2d, Command> getStartPoseAndRoutine(AutoFactory allianceFactory) {
    return command.apply(allianceFactory);
  }
}
