package frc.robot.autos;

// Autos just keeps track of our auto list
public enum Autos {
  NO_AUTO, 

  LEFT_DOUBLE_NZ_CYCLE, // start left, 2 pickup & score cycles in 1/4 neutral zone, climb
  LEFT_NZ_OUTPOST_CYCLE, // start left, 1 neutral zone pickup & score cycle, score outpost, climb
  
  RIGHT_DOUBLE_NZ_CYCLE, // start right, 2 pickup & score cycles in 1/4 neutral zone, climb
  RIGHT_NZ_OUTPOST_CYCLE, // start right, 1 neutral zone pickup & score cycle, score depot, climb
  
  CENTER_OUTPOST_DEPOT, // start center, pickup & score outpost & depot
  OUTPOST_ONLY, // start center 
  CENTER_PRELOAD_CLIMB, // BACKUP - score preload & climb
  
  SL_SWEEP_OUTPOST,  // start left, complete neutral zone sweep, score, exit RT, outpost, score & climb
  SR_SWEEP_DEPOT  // start right, complete neutral zone sweep, score, exit LT, depot, score & climb
}
