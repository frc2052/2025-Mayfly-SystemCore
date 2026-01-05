// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.commands.arm.ArmCommandFactory;
import frc.robot.commands.drive.alignment.AlignmentCommandFactory;
import frc.robot.commands.intake.IntakeCommandFactory;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.arm.ArmPivotSubsystem;
import frc.robot.subsystems.arm.ArmRollerSubsystem;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.AlignmentCalculator.FieldElementFace;

/** Add your docs here. */
public class AutoFactory2 {
    // spotless: off
    private final DriverStation.Alliance alliance;
    private final RobotContainer robotContainer;
    private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();

    private boolean firstScored;

    // we create 2 Auto Factory's (blue & red)
    // to handle geometry differences
    public AutoFactory2(DriverStation.Alliance alliance, RobotContainer robotContainer){
        this.alliance = alliance;
        this.robotContainer = robotContainer;

        firstScored = false;
    }

// -------------------------------- FACTORY METHODS -------------------------------- //

    Pair<Pose2d, Command> createNoAuto() {
        return Pair.of(new Pose2d(), Commands.none());
    }

    Pair<Pose2d, Command> createDriveForwardAuto() {
        return Pair.of(
                getChoreoPath(ChorPaths.DRIVE_FORWARD.getPathName()).getStartingHolonomicPose().get(),
                Commands.sequence(followPathCommand(getChoreoPath(ChorPaths.DRIVE_FORWARD.getPathName()))));
    }

    Pair<Pose2d, Command> createLeftLoliLeftFirstAuto() {
        return Pair.of(
                getChoreoPath(ChorPaths.SL_A.getPathName()).getStartingHolonomicPose().get(),
                Commands.sequence(
                    Commands.runOnce(() -> RobotState.getInstance().setDesiredReefFace(FieldElementFace.AB)),

                    // home, raise L3 on the way to B
                    Commands.parallel(
                            Commands.run(() -> followPathCommand(getChoreoPath(ChorPaths.SL_A.getPathName()))),
                            Commands.sequence(
                                    toPosition(TargetAction.HOME),
                                    Commands.waitUntil(() ->
                                            !ElevatorSubsystem.getInstance().isHoming()),
                                    Commands.wait(0.3),
                                    toPosition(TargetAction.L3))),

                    // align & score preload -> A L4
                    Commands.parallel(
                            AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                            () -> AlignOffset.LEFT_BRANCH, FieldElementFace.AB)
                                    .withTimeout(2.25),
                            Commands.wait(0.3),
                            toPosition(TargetAction.L4),
                            score(TargetAction.L4)),

                    // 1st pickup
                    toPosition(TargetAction.INTAKE),
                    Commands.deadline(
                            Commands.sequence(
                                    Commands.wait(0.3), followPathCommand(getChoreoPath(ChorPaths.AB_LOLIPOP_C.getPathName()))),
                            Commands.parallel(IntakeCommandFactory.intake(), ArmCommandFactory.coralIn())),

                        
                    // score 1st pickup --> B L4
                    Commands.defer(
                        () -> {
                            if(haveCoral()){
                                firstScored = true;
                                return Commands.sequence(
                                    Commands.parallel(
                                        AlignmentCommandFactory.getSpecificReefAlignmentCommand(() -> AlignOffset.RIGHT_BRANCH, FieldElementFace.AB).withTimeout(2.25),
                                        Commands.sequence(
                                            Commands.wait(.3),
                                            toPosition(TargetAction.L4)
                                        ),
                                        toPosition(TargetAction.L4)
                                    ),
                                    score(TargetAction.L4)
                                );
                            } else {
                                firstScored = false;
                                return Commands.print("DIDN'T GET CENTER CORAL");
                            }
                        }, 
                        Set.of()),

                    // 2nd pickup
                    toPosition(TargetAction.INTAKE),
                    Commands.deadline(
                            Commands.sequence(
                                    Commands.wait(0.3), followPathCommand(getChoreoPath(ChorPaths.AB_LOLIPOP_L.getPathName()))),
                            Commands.parallel(IntakeCommandFactory.intake(), ArmCommandFactory.coralIn())),

                    // score 2nd pickup -> B L4 OR A L2
                    Commands.defer(
                        () -> {
                            if(haveCoral()){
                                if(firstScored){
                                    return Commands.sequence(
                                    Commands.parallel(
                                        AlignmentCommandFactory.getSpecificReefAlignmentCommand(() -> AlignOffset.RIGHT_BRANCH, FieldElementFace.AB).withTimeout(2.25),
                                        Commands.sequence(
                                            Commands.wait(.3),
                                            toPosition(TargetAction.L4)
                                        ),
                                        toPosition(TargetAction.L4)
                                    ),
                                    score(TargetAction.L4)
                                );
                                } else {
                                    return Commands.sequence(
                                    Commands.parallel(
                                        AlignmentCommandFactory.getSpecificReefAlignmentCommand(() -> AlignOffset.RIGHT_BRANCH, FieldElementFace.AB).withTimeout(2.25),
                                        Commands.sequence(
                                            Commands.wait(.3),
                                            toPosition(TargetAction.L4)
                                        ),
                                        toPosition(TargetAction.L4)
                                    ),
                                    score(TargetAction.L4)
                                );
                                }
                            } else {
                                return Commands.print("DIDN'T GET LEFT CORAL");
                            }
                        }, 
                        Set.of()),

                    // 3rd pickup 
                    toPosition(TargetAction.INTAKE),
                    Commands.deadline(
                            Commands.sequence(
                                    Commands.wait(0.3), followPathCommand(getChoreoPath(ChorPaths.AB_LOLIPOP_R.getPathName()))),
                            Commands.parallel(IntakeCommandFactory.intake(), ArmCommandFactory.coralIn())),

                    // 3rd scorfe--> either didn't score A L4 (do now) or scored all & just trying to pick up
                    Commands.defer(
                        () -> {
                            if(haveCoral()){
                                if(firstScored){
                                    // score l2
                                    return Commands.sequence(
                                    Commands.parallel(
                                        AlignmentCommandFactory.getSpecificReefAlignmentCommand(() -> AlignOffset.LEFT_BRANCH, FieldElementFace.AB).withTimeout(2.25),
                                        Commands.sequence(
                                            Commands.wait(.3),
                                            toPosition(TargetAction.L2)
                                        ),
                                        toPosition(TargetAction.L2)
                                    ),
                                    score(TargetAction.L4));
                                } else {
                                    // try to score L4 again
                                    return Commands.sequence(
                                    Commands.parallel(
                                        AlignmentCommandFactory.getSpecificReefAlignmentCommand(() -> AlignOffset.LEFT_BRANCH, FieldElementFace.AB).withTimeout(2.25),
                                        Commands.sequence(
                                            Commands.wait(.3),
                                            toPosition(TargetAction.L4)
                                        ),
                                        toPosition(TargetAction.L4)
                                    ),
                                    score(TargetAction.L4));
                                }
                            } else {
                                return Commands.print("DIDN'T GET LEFT CORAL");
                            }
                        }, 
                        Set.of())
                )
            );
    }

    Pair<Pose2d, Command> createTestAuto(){
        return Pair.of(
            getChoreoPath(ChorPaths.TEST_PICKUP.getPathName()).getStartingHolonomicPose().get(), 
            Commands.sequence(
                // pickup 
                toPosition(TargetAction.INTAKE),
                Commands.deadline(
                        Commands.sequence(
                                Commands.wait(0.3), followPathCommand(getChoreoPath(ChorPaths.TEST_PICKUP.getPathName()))),
                        Commands.parallel(IntakeCommandFactory.intake(), ArmCommandFactory.coralIn())),
                // have coral? drive forward 2 meters
                // no coral? keep driving backwards
                Commands.defer(() -> {
                    if(haveCoral()){
                        return followPathCommand(getChoreoPath(ChorPaths.TEST_2.getPathName()));
                    } else {
                        return followPathCommand(getChoreoPath(ChorPaths.TEST_3.getPathName()));
                    }
                }, 
                Set.of())
            ));
    }

    // -------------------------------- COMMON FUNCTIONS -------------------------------- //

    Command manualZero() {
        return new InstantCommand(() -> drivetrain.seedFieldCentric());
    }

    public PathPlannerPath getChoreoPath(String chorPathName) {
        try {
            return PathPlannerPath.fromChoreoTrajectory(chorPathName);
        } catch (Exception e) {
            DriverStation.reportError(
                    "FAILED TO GET CHOREO PATH FROM PATHFILE " + chorPathName + e.getMessage(), e.getStackTrace());
            return null;
        }
    }

    Command followPathCommand(PathPlannerPath path) {
        return AutoBuilder.followPath(path);
    }

    // Command getBumpCommand(){
    //     return new ConditionalCommand(
    //         new DefaultDriveCommand(() -> 0.7, () -> 0, () -> 0, () -> true).withDeadline(new WaitCommand(0.4)),
    //         new InstantCommand(),
    //         () -> AutoChooser.getBumpNeeded());
    // }

    // Command delaySelectedTime(){
    //     return new WaitCommand(AutoChooser.getWaitSeconds());
    // }

    Command elevatorToPos(TargetAction position) {
        return new InstantCommand(() -> SuperstructureSubsystem.getInstance().setCurrentAction(position));
    }

    Command score(TargetAction position) {
        return Commands.parallel(
                        IntakeCommandFactory.outtake().withTimeout(0.3),
                        ArmCommandFactory.coralOut().withTimeout(0.5))
                .andThen(() -> RobotState.getInstance().setAlignOffset(AlignOffset.MIDDLE_REEF));
    }

    boolean haveCoral(){
        Commands.print("STOWED: " + (SuperstructureSubsystem.getInstance().getCurrentAction() == TargetAction.L3));
        Commands.print("HAVE CORAL: " + (RobotState.getInstance().getHasCoral()));
        return (SuperstructureSubsystem.getInstance().getCurrentAction().equals(TargetAction.L3)) || RobotState.getInstance().getHasCoral();
    }

    Command toPosAndScore(TargetAction position) {
        return Commands.sequence(
                Commands.runOnce(() -> SuperstructureSubsystem.getInstance().setCurrentAction(position)),
                Commands.runOnce(() -> ArmRollerSubsystem.getInstance().stopMotor()),
                Commands.waitUntil(() -> ElevatorSubsystem.getInstance().atPosition(2.0, position)
                        && ArmPivotSubsystem.getInstance().isAtDesiredPosition()),
                ArmCommandFactory.coralIn().withTimeout(0.2),
                Commands.runOnce(() -> ArmRollerSubsystem.getInstance().stopMotor()),
                ArmCommandFactory.coralOut().withTimeout(0.55));
    }

    Command toPosition(TargetAction position) {
        return Commands.runOnce(() -> SuperstructureSubsystem.getInstance().setCurrentAction(position));
    }

    // TODO: deffered command?
    Command scoreNet() {
        return Commands.sequence(
                ArmCommandFactory.algaeIn().until(() -> ArmPivotSubsystem.getInstance()
                        .isAtPosition(2.0, TargetAction.ALGAE_NET.getArmPivotAngle())),
                ArmCommandFactory.algaeOut().withTimeout(0.5));
    }

    // spotless: on
}
