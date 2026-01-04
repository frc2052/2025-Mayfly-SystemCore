// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.function.Function;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

// represents a single auto routine
    // start pose & command

// bridge between 
    // (1) Auto.java enum identifier 
    // (2) custom readable label for dashboard
    // (3) AutoFactory method in java (actual auto Command)

// Function is a method reference, stores w/o calling it
// this is because we have TWO autofactorys for Red vs Blue
// we don't call the path until we know which factory should be used
// most reliable call time: right at autonomousInit()
// we need this delay because start poses (needed to reset odom)
    // differ per side

public class AutoProgram {
    private final Auto auto;
    private final String name;
    private final Function<AutoFactory2, Pair<Pose2d, Command>> command;

    public AutoProgram(
        Auto auto,
        String name,
        Function<AutoFactory2, Pair<Pose2d, Command>> command
    ){
        this.auto = auto;
        this.name = name;
        this.command = command;
    }

    public Auto getAuto(){
        return auto;
    }

    public String getName(){
        return name;
    }

    public Pair<Pose2d, Command> getCommandAndPose(AutoFactory2 factory){
        return command.apply(factory);
    }

    public Pose2d getPose(AutoFactory2 factory){
        return command.apply(factory).getFirst();
    }

    public Command getCommand(AutoFactory2 factory){
        return command.apply(factory).getSecond();
    }
}
