package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

// spotless: off

// AutoFactory is where we write out what an auto DOES
public class AutoFactory {
  private final RobotContainer robotContainer;

  static final NetworkTableInstance networkTables = NetworkTableInstance.getDefault();
  static final NetworkTable debugTable = networkTables.getTable("autos networktables tab");
  static final DoubleTopic waitTimeTopic = debugTable.getDoubleTopic("waitTime");
  static final DoublePublisher waitTimePublisher = waitTimeTopic.publish();
  static final DoubleSubscriber waitTimeSubscriber = waitTimeTopic.subscribe(0);

  static final LoggedDashboardChooser climbSideChooser =
      new LoggedDashboardChooser<ClimbSide>(
          "Auto Climb Side Chooser", new SendableChooser<ClimbSide>());

  public AutoFactory(RobotContainer robotContainer) {
    climbSideChooser.addDefaultOption("Climb LEFT", ClimbSide.LEFT_CLIMB);
    climbSideChooser.addOption("Climb RIGHT", ClimbSide.RIGHT_CLIMB);

    waitTimePublisher.set(0); // initial value
    this.robotContainer = robotContainer;
  }

  public enum ClimbSide {
    LEFT_CLIMB,
    RIGHT_CLIMB
  }

  // page navigation
  void navigation() {
    createNoAuto();
    createLeftDoubleNZAuto();
    createLeftNZDepotAuto();
    createRightDoubleNZAuto();
    createRightNZOutpostAuto();
  }

  // -------------------------------- ACTUAL AUTOS -------------------------------- //

  Pair<Pose2d, Command> createNoAuto() {
    return Pair.of(new Pose2d(), Commands.none());
  }

  Pair<Pose2d, Command> createLeftDoubleNZAuto() {
    return Pair.of(
        getStartPose(ChorPaths.LTRENCH_LNEUTRAL1),
        // first cycle
        Commands.sequence(
            Commands.deadline(
                    followPathCommand(ChorPaths.LTRENCH_LNEUTRAL1),
                    followPathCommand(ChorPaths.LNEUTRAL_LTRENCH)),
            // second cycle
            Commands.sequence(
                Commands.deadline(
                    Commands.sequence(
                        followPathCommand(ChorPaths.LTRENCH_LNEUTRAL2),
                        followPathCommand(ChorPaths.LNEUTRAL_LTRENCH)),
                // climb
                followPathCommand(ChorPaths.LTRENCH_CLIMREADY),
                climbChosenSide()))));
  }

  Pair<Pose2d, Command> createLeftNZDepotAuto() {
    return Pair.of(
        getStartPose(ChorPaths.LTRENCH_LNEUTRAL1),
        Commands.sequence(
            // first cycle
            Commands.deadline(
                Commands.sequence(
                    followPathCommand(ChorPaths.LTRENCH_LNEUTRAL1),
                    followPathCommand(ChorPaths.LNEUTRAL_LTRENCH)),
            // depot cycle
                followPathCommand(ChorPaths.LTRENCH_DEPOT),
            // climb
            followPathCommand(ChorPaths.LTRENCH_CLIMREADY),
            climbChosenSide())));
  }

  Pair<Pose2d, Command> createRightDoubleNZAuto() {
    return Pair.of(
        getStartPose(ChorPaths.LTRENCH_LNEUTRAL1),
        // first cycle
        Commands.sequence(
            followPathCommand(ChorPaths.LTRENCH_LNEUTRAL1),
            followPathCommand(ChorPaths.LNEUTRAL_LTRENCH),
            // second cycle
            Commands.sequence(
                    Commands.sequence(
                        followPathCommand(ChorPaths.LTRENCH_LNEUTRAL2),
                        followPathCommand(ChorPaths.LNEUTRAL_LTRENCH)),
                // climb
                followPathCommand(ChorPaths.RTRENCH_CLIMREADY),
                climbChosenSide())));
  }

  Pair<Pose2d, Command> createRightNZOutpostAuto() {
    return Pair.of(
        getStartPose(ChorPaths.RTRENCH_RNEUTRAL1),
        // first cycle
        Commands.sequence(
                Commands.sequence(
                    followPathCommand(ChorPaths.RTRENCH_RNEUTRAL1),
                    followPathCommand(ChorPaths.RNEUTRAL_RTRENCH)),
            // outpost cycle
            followPathCommand(ChorPaths.RTRENCH_OUTPOST),
            // TODO: confirm we can score from outpost
            // climb
            followPathCommand(ChorPaths.RTRENCH_CLIMREADY),
            climbChosenSide()));
  }

  // write out auto logic, creating paths along the way
  Pair<Pose2d, Command> createCenterDepotOutpost() {
    return Pair.of(
        getStartPose(ChorPaths.CENTER_DEPOT),
        // depot cycle
        Commands.sequence(
                followPathCommand(ChorPaths.CENTER_DEPOT),
            // outpost cycle
            followPathCommand(ChorPaths.DEPOT_OUTPOST),
            // climb
            followPathCommand(ChorPaths.RTRENCH_CLIMREADY),
            climbChosenSide()));
  }

  Pair<Pose2d, Command> createCenterOutpost() {
    return Pair.of(
        getStartPose(ChorPaths.CENTER_OUTPOST),
        Commands.sequence(
            followPathCommand(ChorPaths.CENTER_OUTPOST),
            // climb
            followPathCommand(ChorPaths.RTRENCH_CLIMREADY),
            climbChosenSide()));
  }

  Pair<Pose2d, Command> createCenterPreloadClimbAuto() {
    return Pair.of(getStartPose(ChorPaths.CENTER_OUTPOST), Commands.sequence(climbChosenSide()));
  }

  Pair<Pose2d, Command> createLSweepOutpost(){
    // intake out after one second
    return Pair.of(
      getStartPose(ChorPaths.LTRENCH_SWEEP), 
      Commands.sequence(
          followPathCommand(ChorPaths.LTRENCH_SWEEP),
        followPathCommand(ChorPaths.SWEEP_RTRENCH),
      // update for shoot while move?
          followPathCommand(ChorPaths.RTRENCH_OUTPOST).andThen(Commands.waitSeconds(5)),
        // climb
        followPathCommand(ChorPaths.CENTER_OUTPOST))
      );
  }

  Pair<Pose2d, Command> createRSweepOutpost(){
    return Pair.of(
      getStartPose(ChorPaths.RTRENCH_SWEEP), 
      // sweep & score
      Commands.sequence(
          followPathCommand(ChorPaths.RTRENCH_SWEEP),
      followPathCommand(ChorPaths.SWEEP_LTRENCH),
      // update for shoot while move?
      // depot cycle
      followPathCommand(ChorPaths.LTRENCH_DEPOT),
      // climb
      followPathCommand(ChorPaths.LTRENCH_CLIMREADY),
      climbChosenSide()
    ));
  }

  // -------------------------------- FACTORY METHODS -------------------------------- //

  public Command climbChosenSide(){
    return Commands.defer(
      () -> {
          Object selected = climbSideChooser.get();
          ClimbSide side = selected != null ? (ClimbSide) selected : ClimbSide.LEFT_CLIMB;
          ChorPaths climbPath = side == ClimbSide.LEFT_CLIMB ? ChorPaths.CLIMB_LEFT : ChorPaths.CLIMB_RIGHT;
          // TODO: fill w/ auto-align commands
          return followPathCommand(climbPath);
        }, Set.of());
  }

  public Command waitSuppliedSeconds() {
    return Commands.defer(
        () -> {
          double wait = waitTimeSubscriber.get();
          System.out.println("==== AUTO WAIT SECONDS SET: " + wait + " seconds");
          return Commands.waitSeconds(wait);
        },
        Set.of());
  }

  public Command followPathCommand(ChorPaths chorPath) {
    // go from a ChorPath String to a PathPlannerPath
    try {
      PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(chorPath.getPathName());
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      DriverStation.reportError(
          "CANNOT FIND CHOREO PATH NAMED: " + chorPath.getPathName(), e.getStackTrace());
      return Commands.none();
    }
  }

  public Pose2d getStartPose(ChorPaths chorPath) {
    try {
      PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(chorPath.getPathName());
      return path.getStartingHolonomicPose().get();
    } catch (Exception e) {
      DriverStation.reportError(
          "CANNOT FIND CHOREO PATH NAMED: " + chorPath.getPathName(), e.getStackTrace());
      return new Pose2d();
    }
  }
}
// spotless: on
