package frc.robot.autos;

public enum ChorPaths {
  // choreo paths are the ones that end in .traj
  // enum value name (blue): what we call the path in code
  // String parameter (salmon): EXACT case-sensitive name of the path in Choreo

  // start paths
  LTRENCH_LNEUTRAL1("am_LT_NZ1"), // confirmed
  LNEUTRAL_LTRENCH("am_LN_LT"), // confirmed
  LTRENCH_LNEUTRAL2("am_LT_NZ2"), // confirmed
  LTRENCH_CLIMREADY("am_LT_CLIMB"), // confirmed

  LTRENCH_DEPOT("am_LT_DEPOT"), // confirmed

  RTRENCH_RNEUTRAL1("am_RT_NZ1"), // confirmed
  RNEUTRAL_RTRENCH("am_RN_RT"), // confirmed
  RTRENCH_RNEUTRAL2("am_RT_NZ2"), // confirmed
  RTRENCH_CLIMREADY("am_RT_CLIMB"), // confirmed

  RTRENCH_OUTPOST("am_RT_OUTPOST"), // confirmed

  // center
  CENTER_DEPOT("am_C_DEPOT"), // confirmed
  CENTER_OUTPOST("am_C_OUTPOST"), // confirmed
  DEPOT_OUTPOST("am_DEPOT_OUTPOST"), // confirmed

  // sweep
  LTRENCH_SWEEP("am_LT_SWEEP"), // confirmed
  SWEEP_RTRENCH("am_SWEEP_RT"), // confirmed
  RTRENCH_SWEEP("am_RT_SWEEP"),
  SWEEP_LTRENCH("am_SWEEP_LT"),

  // climb
  CLIMB_RIGHT("am_CLIMB_RIGHT"),
  CLIMB_LEFT("am_CLIMB_LEFT");


  public String pathName;

  private ChorPaths(String pName) {
    pathName = pName;
  }

  public String getPathName() {
    return pathName;
  }
}
