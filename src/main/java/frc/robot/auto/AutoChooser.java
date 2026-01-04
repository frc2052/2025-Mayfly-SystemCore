package frc.robot.auto;

import java.security.AllPermission;
import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DrivetrainSubsystem;

public class AutoChooser {
    // auto list

    private static final List<AutoProgram> AUTO_PROGRAMS = 
        List.of(
            new AutoProgram(Auto.NO_AUTO, "NO AUTO", AutoFactory2::createNoAuto),
            new AutoProgram(Auto.DRIVE_FORWARD, "DRIVE FORWARD AUTO", AutoFactory2::createDriveForwardAuto),
            new AutoProgram(Auto.LEFT_LOLI, "LEFT LOLI AUTO", AutoFactory2::createLeftLoliLeftFirstAuto)
            // new AutoProgram(Auto.RIGHT_LOLI, "RIGHT LOLI AUTO", AutoFactory2::createRightLoliRightFirstAuto)
        );

    // factories --> needed to swap geometries
    private final AutoFactory2 blueFactory;
    private final AutoFactory2 redFactory;

    private Auto lastSelected = null;

    private Pair<Pose2d, Command> blueAuto = null;
    private Pair<Pose2d, Command> redAuto = null;

    private final LoggedDashboardChooser<Auto> autoChooser = new LoggedDashboardChooser<Auto>("AUTO CHOOSER 2026");

    public AutoChooser(RobotContainer robotContainer){
        blueFactory = new AutoFactory2(Alliance.Blue, robotContainer);
        redFactory = new AutoFactory2(Alliance.Red, robotContainer);

        // populate chooser --> manually for now, chooser problems
        autoChooser.addDefaultOption("NO AUTO", Auto.NO_AUTO);
        autoChooser.addOption("DRIVE FORWARD", Auto.DRIVE_FORWARD);
        autoChooser.addOption("LEFT LOLI", Auto.LEFT_LOLI);
        autoChooser.addOption("RIGHT LOLI", Auto.RIGHT_LOLI);
    }

    public static AutoChooser create(final RobotContainer robotContainer){
        var autoChooser = new AutoChooser(robotContainer);
        return autoChooser;
    }

    public void reset(){
        blueAuto = null;
        redAuto = null;
        lastSelected = null;
    }

    public void update(){
        Auto selected = autoChooser.get();

        if(selected == null){
            System.out.println("NO AUTO WAS CHOSEN --> change to default");
            selected = Auto.NO_AUTO;
        } else if ((selected != lastSelected)){
            System.out.println("REBUILDING AUTO: " + selected.name());

            AutoProgram program = findProgram(selected);
            blueAuto = program.getCommandAndPose(blueFactory);
            redAuto = program.getCommandAndPose(redFactory);

            lastSelected = selected;
        } else {
            System.out.println("NO AUTO CHANGE. CHOSEN " + selected.name());
        }
    }
    
    // -------------------------------- HELPERS -------------------------------- //

    // return auto command for current alliance
    // called @ autonomousInit
    // when you choose a new auto,
    // it creates a blueAuto and redAuto version,
    // WHEN AUTO BEGINS getAuto() is called and you run either red or blue @ runtime because both are updated
    public Command getAuto() {
        if (DriverStation.getAlliance().isPresent()) {
            DriverStation.Alliance alliance = DriverStation.getAlliance().get();
            if (alliance == DriverStation.Alliance.Blue) {
                System.out.println("BLUE SIDE AUTO DETECTED, RETURNING COMMAND");
                new InstantCommand(() -> DrivetrainSubsystem.getInstance().resetPose(blueAuto.getFirst()));
                return blueAuto.getSecond(); // returns command
            } else {
                System.out.println("RED SIDE AUTO DETECTED, RETURNING COMMAND");
                new InstantCommand(() -> DrivetrainSubsystem.getInstance().resetPose(blueAuto.getFirst()));
                return redAuto.getSecond();
            }
        }
        System.out.println("DRIVERSTATION ALLIANCE NOT PRESENT FOR AUTOS");
        return null;
    }

    // FIND THE AUTO PROGRAM that matches the enum - loop instead of streams from b4
    private AutoProgram findProgram(Auto dAuto) {
        for (AutoProgram program : AUTO_PROGRAMS) {
            if (program.getAuto() == dAuto) {
                return program;
            }
        }
        System.out.println("Could not find auto program for: " + dAuto.name());
        return null;
    }
}
