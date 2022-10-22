// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.flywheel;

public class ShotSelecter {

  // private static final ShotRecipe[] shotRecipes = {
  //   new ShotRecipe(1550, .6, 2.80),
  //   new ShotRecipe(1650, .6, 3.85),
  //   new ShotRecipe(1780, .6, 4.87),
  //   new ShotRecipe(1850, .3, 5.55),
  //   new ShotRecipe(2050, .1, 6.53),
  // };

  private static final ShotRecipe[] shotRecipes = {
    new ShotRecipe(1550, .6, 2.80),
    // new ShotRecipe(, .6,3.03),
    new ShotRecipe(1650, .6, 3.85),
    new ShotRecipe(1780, .6, 4.87),
    new ShotRecipe(1850, .3, 5.55),
    new ShotRecipe(2050, .1, 6.53),
  };

  private static double interpolatedRPM = 1650;
  private static ShotRecipe closest = shotRecipes[0], lastClosest = shotRecipes[0];

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

  public static double interpolateRPM(double distance) {
    distance = Math.min(distance, 7.0);
    if (distance > 0) {
      interpolatedRPM =
          Math.round(
                  Math.max(16.5033 * distance * distance + (-22.7286) * distance + 1487.78, 1600)
                      / 10.0)
              * 10;
    }
    return interpolatedRPM;
  }
}
