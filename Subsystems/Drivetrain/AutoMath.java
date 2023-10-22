package org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

import static java.lang.Math.toRadians;

import org.firstinspires.ftc.teamcode.Subsystems.DuckPositionsAndLevels;

public class AutoMath
{

    public static final long ADJUSTABLE_START = 0; // milliseconds

    private static final double adj1 = 4;
    private static final double adj2 = 2;
    private static final double adj3 = 3;

    public static double determineAdj(DuckPositionsAndLevels level)
    {
        if (level == DuckPositionsAndLevels.LEFT_LEVEL1)
            return adj1;
        else if (level == DuckPositionsAndLevels.MIDDLE_LEVEL2)
            return adj2;
        else
            return adj3;
    }

//    private static final double lowerRed1adj = 8.5;
//    private static final double lowerRed2adj = 7.5;
//    private static final double lowerRed3adj = 9.75;
//
//    public static double determineLowerRedAdj(DuckPositionsAndLevels level)
//    {
//        if (level == DuckPositionsAndLevels.LEFT_LEVEL1)
//            return lowerRed1adj;
//        else if (level == DuckPositionsAndLevels.MIDDLE_LEVEL2)
//            return lowerRed2adj;
//        else
//            return lowerRed3adj;
//    }
//
//    private static final double lowerBlue1adj = 3.75;
//    private static final double lowerBlue2adj = 2.75;
//    private static final double lowerBlue3adj = 5.0;
//
//    public static double determineLowerBlueAdj(DuckPositionsAndLevels level)
//    {
//        if (level == DuckPositionsAndLevels.LEFT_LEVEL1)
//            return lowerBlue1adj;
//        else if (level == DuckPositionsAndLevels.MIDDLE_LEVEL2)
//            return lowerBlue2adj;
//        else
//            return lowerBlue3adj;
//    }
//
//    private static final double upperRed1adj = lowerBlue1adj;
//    private static final double upperRed2adj = lowerBlue2adj;
//    private static final double upperRed3adj = lowerBlue3adj;
//
//    public static double determineUpperRedAdj(DuckPositionsAndLevels level)
//    {
//        if (level == DuckPositionsAndLevels.LEFT_LEVEL1)
//            return upperRed1adj;
//        else if (level == DuckPositionsAndLevels.MIDDLE_LEVEL2)
//            return upperRed2adj;
//        else
//            return upperRed3adj;
//    }
//
//    private static final double upperBlue1adj = lowerBlue1adj;
//    private static final double upperBlue2adj = lowerBlue2adj;
//    private static final double upperBlue3adj = lowerBlue3adj;
//
//    public static double determineUpperBlueAdj(DuckPositionsAndLevels level)
//    {
//        if (level == DuckPositionsAndLevels.LEFT_LEVEL1)
//            return upperBlue1adj;
//        else if (level == DuckPositionsAndLevels.MIDDLE_LEVEL2)
//            return upperBlue2adj;
//        else
//            return upperBlue3adj;
//    }
}
