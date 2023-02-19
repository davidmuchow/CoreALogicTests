// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Random;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
    Random rand = new Random();

    int numOfChecks = 50000;
    SwerveModuleState[][] stateChecks = new SwerveModuleState[numOfChecks][4];

    for (int i = 0; i < numOfChecks; i++) {
      stateChecks[i] = Constants.swerveKinematics.toSwerveModuleStates(
          ChassisSpeeds.fromFieldRelativeSpeeds(Math.random() * 50 * (rand.nextBoolean() ? -1 : 1), Math.random() * 50 * (rand.nextBoolean() ? -1 : 1), Math.random() * 12.6 * (rand.nextBoolean() ? -1 : 1),
              Rotation2d.fromDegrees(Math.random() * 108000 * (rand.nextBoolean() ? -1 : 1))));
    }

    int count = 1;
    boolean allPassed = true;

    for (SwerveModuleState[] check : stateChecks) {
      boolean passed = true;
      Rotation2d pastRotation = Rotation2d.fromDegrees(new Random().nextDouble() * 720 * (rand.nextBoolean() ? -1 : 1));
      for (SwerveModuleState toCheck : check) {
        if (!Tests.test1(toCheck, pastRotation)) {
          passed = false;
        }
      }
      if (passed) {
        System.out.println("Test " + count + " passed");
      } else {
        System.out.println("Test " + count + " failed");
        allPassed = false;
      }
      count++;
    }

    if (allPassed) {
      System.out.println("\nAll tests passed");
      System.out.println("Old in " + Tests.getAverageOldTime());
      System.out.println("New in " + Tests.getAverageNewTime());
    }
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
