package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.MetricRR.MetricConversions.toMeters;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class KeyPointsAndValues {

    // RED ROBOT POSE POSITIONS
    public static Pose2d lowerRedStart = new Pose2d(toMeters(-37.5), toMeters(-63), Math.toRadians(90));
    public static Pose2d upperRedStart = new Pose2d(9.5, -63, Math.toRadians(90));
    public static Pose2d redCarouselPose = new Pose2d(-55,-55, Math.toRadians(180+45));
    public static Pose2d redStorageUnitPark = new Pose2d(-60, -35.5, Math.toRadians(90));
    public static Pose2d redPipesPose = new Pose2d(toMeters(10), toMeters(-46), 0);

    // BLUE ROBOT POSE POSITIONS
    public static Pose2d lowerBlueStart = new Pose2d(-33.1, 63, Math.toRadians(270));
    public static Pose2d upperBlueStart = new Pose2d(9.5, 63, Math.toRadians(270)); // NEEDS FIX!
    public static Pose2d blueCarouselPose = new Pose2d(-55,55, Math.toRadians(180-45));
    public static Pose2d blueStorageUnitPark = new Pose2d(-60, 35.5, Math.toRadians(270));
    public static Pose2d bluePipesPose = new Pose2d(10, 46, 0);

    // CONSTANTS FOR BOTH RED AND BLUE
    public static double pipeDriveDistance = toMeters(35);

    //////////////////
    public static double lowerBarcodeL1x = -44;
    public static double lowerBarcodeL2x = -35.5;
    public static double lowerBarcodeL3x = -27;

    // ADJUSTABLE START
    public static long delayStart = 0;

    public static double determineBarcodeXLower(DuckPositionsAndLevels duckPosition)
    {
        if (duckPosition == DuckPositionsAndLevels.LEFT_LEVEL1)
            return lowerBarcodeL1x;
        else if (duckPosition == DuckPositionsAndLevels.MIDDLE_LEVEL2)
            return lowerBarcodeL2x;
        else
            return lowerBarcodeL3x;
    }
}
