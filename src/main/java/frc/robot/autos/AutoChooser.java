// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import java.util.List;

// AutoChooser manages letting us choose autos before a match & sets start poses
public class AutoChooser {
  private static final List<AutoProgram> AUTO_PROGRAMS =
      List.of(
          new AutoProgram(Autos.NO_AUTO, AutoFactory::createNoAuto),
          new AutoProgram(Autos.LEFT_DOUBLE_NZ_CYCLE, AutoFactory::createLeftDoubleNZAuto),
          new AutoProgram(Autos.LEFT_NZ_OUTPOST_CYCLE, AutoFactory::createLeftNZDepotAuto),
          new AutoProgram(Autos.RIGHT_DOUBLE_NZ_CYCLE, AutoFactory::createRightDoubleNZAuto),
          new AutoProgram(Autos.RIGHT_NZ_OUTPOST_CYCLE, AutoFactory::createRightNZOutpostAuto),
          new AutoProgram(Autos.OUTPOST_ONLY, AutoFactory::createCenterOutpost));

  private final AutoFactory factory;
  private Autos lastSelected = null;
  private Pair<Pose2d, Command> currentAuto = null;
  private final SendableChooser<Autos> autoChooser = new SendableChooser<>();

  public AutoChooser(RobotContainer robotContainer) {

    factory = new AutoFactory(robotContainer);

    currentAuto = AUTO_PROGRAMS.get(0).getStartPoseAndRoutine(factory);
    lastSelected = Autos.NO_AUTO;

    // go through ALL AutoPrograms and put them on the Chooser
    for (AutoProgram program : AUTO_PROGRAMS) {
      if (program.getAuto().equals(Autos.NO_AUTO)) {
        autoChooser.setDefaultOption(program.getAuto().name(), program.getAuto());
        System.out.println("Added default Auto: " + program.getAuto().name());
      } else {
        autoChooser.addOption(program.getAuto().name(), program.getAuto());
        System.out.println("Added auto option: " + program.getAuto().name());
      }
    }

    SmartDashboard.putData("Auto Chooser 26", autoChooser);
  }

  public static AutoChooser create(final RobotContainer robotContainer) {
    return new AutoChooser(robotContainer);
  }

  // runs in disabled periodic
  public void update() {
    Autos selected = autoChooser.getSelected();

    if (selected == null) {
      System.out.println("NO AUTO WAS CHOSEN, selectedAuto is null!");
      return;
    } else if (selected != lastSelected) {
      System.out.println("REBUILDING AUTO: " + selected.name());
      AutoProgram program = findProgram(selected);

      currentAuto =
          (program != null)
              ? program.getStartPoseAndRoutine(factory)
              : AUTO_PROGRAMS.get(0).getStartPoseAndRoutine(factory);

      lastSelected = selected;
    }

    // preview start pose in AdvantageScope while disabled
    Pose2d currentStartPose = currentAuto.getFirst();
    if (currentStartPose != null) {
      RobotState.getInstance().setAutoStartPose(getAllianceAdjustedPose(currentAuto.getFirst()));
    }

    // print to check chooser functionality 2/25
    System.out.println(
        "current auto: "
            + selected.name()
            + " -- wait value: "
            + AutoFactory.waitTimeSubscriber.get()
            + " seconds -- climb side: "
            + AutoFactory.climbSideChooser.get());
  }

  // -------------------------------- HELPERS -------------------------------- //
  public Pose2d getAllianceAdjustedPose(Pose2d bluePose) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      return FlippingUtil.flipFieldPose(bluePose);
    } else {
      return bluePose;
    }
  }

  public Command getAuto() {
    if (currentAuto == null) {
      return Commands.sequence(
          Commands.print("CURRENTAUTO WAS NULL AT RUNTIME."),
          Commands.print(
              "DRIVERSTATION ALLIANCE PRESENT?: " + DriverStation.getAlliance().isPresent()),
          Commands.none());
    }

    Pose2d startPose = currentAuto.getFirst();
    return Commands.sequence(
        Commands.runOnce(() -> DrivetrainSubsystem.getInstance().resetPose(startPose)),
        Commands.runOnce(
            () -> RobotState.getInstance().setAutoStartPose(getAllianceAdjustedPose(startPose))),
        currentAuto.getSecond());
  }

  // matches the AUTO (what is pushed to Elastic by the chooser & empty reference)
  // to the AutoProgram (which can fetch start pose & the commands)
  private AutoProgram findProgram(Autos auto) {
    for (AutoProgram program : AUTO_PROGRAMS) {
      if (program.getAuto().equals(auto)) {
        return program;
      }
    }
    System.out.println(
        "Could not find AutoProgram for the Auto: " + auto.name() + ". Suppling NO_AUTO");
    return AUTO_PROGRAMS.get(0);
  }
}
