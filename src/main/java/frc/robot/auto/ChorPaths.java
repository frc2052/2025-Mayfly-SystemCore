// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.units.measure.Angle;
import frc.robot.RobotState;

/** Add your docs here. */
public enum ChorPaths {

    DRIVE_FORWARD("DRIVE FORWARD"),
    SL_A("SL A"),
    AB_LOLIPOP_C("AB LOLIPOP-C"),
    AB_LOLIPOP_L("AB LOLIPOP-L"),
    AB_LOLIPOP_R("AB LOLIPOP-R"),
    TEST_PICKUP("TEST-PICKUP"),
    TEST_2("TEST-2"),
    TEST_3("TEST-3");

    public String pathName;

    private ChorPaths(String pName){
        pathName = pName;
    }

    public String getPathName(){
        return pathName;
    }

}
