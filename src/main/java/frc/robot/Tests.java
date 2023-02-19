package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Tests {
    private static int numTimesCompleted = 0;
    private static double totalOldComplete = 0.0;
    private static double totalNewComplete = 0.0;

    public static boolean test1(SwerveModuleState desiredState, Rotation2d currentAngle) {
        numTimesCompleted++;
        double startOldTime = System.nanoTime();
        SwerveModuleState oldCodeResponse = oldCodeTest(desiredState, currentAngle);
        totalOldComplete += System.nanoTime() - startOldTime;

        double startNewTime = System.nanoTime();
        SwerveModuleState newCodeResponse = newCodeTest(desiredState, currentAngle);
        totalNewComplete += System.nanoTime() - startNewTime;

        if (oldCodeResponse.compareTo(newCodeResponse) == 0) {
            return true;
        } else {
            System.out.println("Results: \n expected: " + oldCodeResponse + "\n got: " + newCodeResponse);
            return false;
        }
    }

    public static SwerveModuleState oldCodeTest(SwerveModuleState desiredState, Rotation2d currentAngle) {
        return CTREModuleState.optimize(desiredState, currentAngle);
    }

    public static SwerveModuleState newCodeTest(SwerveModuleState desiredState, Rotation2d currentAngle) {
        return optimize(desiredState, currentAngle);
    }

    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();
        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }


    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }

        double newestAngle = MathUtil.inputModulus(newAngle, lowerBound, upperBound);
        
        if (newestAngle - scopeReference > 180) {
            newestAngle -= 360;
        } else if (newestAngle - scopeReference < -180) {
            newestAngle += 360;
        }
        return newestAngle;
    }

    public static double getAverageOldTime() {
        return (totalOldComplete / (double)numTimesCompleted) / 1000000000;
    }

    public static double getAverageNewTime() {
        return (totalNewComplete / (double)numTimesCompleted) / 1000000000;
    }
}
