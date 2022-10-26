// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.flywheel;

public class ShotSelecter {

  private static final ShotRecipe[] shotRecipes = {
    new ShotRecipe(1780, .83, 2.726),
    // new ShotRecipe(, .6,3.03),
    new ShotRecipe(1650, .6, 2.87),
    new ShotRecipe(1700, .6, 3.01),
    new ShotRecipe(1735, .6, 3.29),
    new ShotRecipe(1775, .6, 3.74),
    new ShotRecipe(1890, .6, 4.59),
  };

  private static double interpolatedRPM = 1650;
  private static ShotRecipe closest = shotRecipes[0], lastClosest = shotRecipes[0];
  private static ShotRecipe kicker = shotRecipes[1], lastKicker = shotRecipes[1];

  public ShotSelecter() {}
  // function that takes distance and returns the rpm of the shot with the closest distance
  public static double bestShot(double distance) {
    // return shot with closest distance
    for (ShotRecipe shot : shotRecipes) {
      if (Math.abs(shot.getDistance() - distance) < Math.abs(closest.getDistance() - distance)
          && Math.abs(shot.getDistance() - distance) * 2
              < Math.abs(lastClosest.getDistance() - distance)) {
        closest = shot;
      }
    }
    lastClosest = closest;
    return closest.getRPM();
  }

  public static double bestKickerPercentOutput(double distance) {
    // return shot with closest distance
    for (ShotRecipe shot : shotRecipes) {
      if (Math.abs(shot.getDistance() - distance) < Math.abs(kicker.getDistance() - distance)
          && Math.abs(shot.getDistance() - distance) * 2
              < Math.abs(lastKicker.getDistance() - distance)) {
        kicker = shot;
      }
    }
    lastClosest = closest;
    return closest.getRPM();
  }

  public static double interpolateRPM(double distance) {
    distance = Math.min(distance, 7.0);
    if (distance > 0) {
      interpolatedRPM =
          Math.round(
                  Math.max(-2.38731 * distance * distance + (141.668) * distance + 1288.02, 1650)
                      / 10.0)
              * 10;
    }
    return interpolatedRPM;
  }
}
