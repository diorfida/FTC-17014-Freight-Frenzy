package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LEDLights
{
    private LinearOpMode opMode;

    private enum GameTime
    {
        DRIVER_CONTROLLED,
        FIVE_UNTIL_ENDGAME,
        ENDGAME,
        LAST_5_SECONDS,
        MATCH_OVER
    }

    private RevBlinkinLedDriver lights;
    private ElapsedTime clock;
    private static boolean started = false;

    public LEDLights(LinearOpMode opMode)
    {
        this.opMode = opMode;
        lights = opMode.hardwareMap.get(RevBlinkinLedDriver.class, "lights");
    }

    public void initLightsAuto()
    {
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    public void initLightsTele(boolean poseTransfer)
    {
        if (poseTransfer)
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        else
        {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_FOREST_PALETTE);
            opMode.telemetry.addData("WARNING:", "Pose not transferred from auto.");
            opMode.telemetry.addLine("Automated capabilities are unavailable.");
            opMode.telemetry.addLine();
        }
    }

    public void autonomousLights(Alliance alliance)
    {
        switch (alliance)
        {
            case RED:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                break;
            case BLUE:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                break;
        }
    }

    public void startTeleOp()
    {
        clock = new ElapsedTime();
        started = true;
    }

    public void run(Alliance alliance)
    {
        if (!started)
            startTeleOp();

        switch (determineMatchTime())
        {
            case DRIVER_CONTROLLED:
                switch (alliance)
                {
                    case RED:
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                        break;
                    case BLUE:
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                        break;
                }
                break;
            case FIVE_UNTIL_ENDGAME:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE);
                break;
            case ENDGAME:
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);
                break;
            case LAST_5_SECONDS:
                switch (alliance)
                {
                    case RED:
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_RED);
                        break;
                    case BLUE:
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_BLUE);
                        break;
                }
                break;
            case MATCH_OVER:
                switch (alliance)
                {
                    case RED:
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
                        break;
                    case BLUE:
                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
                        break;
                }
                break;
        }
    }

    private GameTime determineMatchTime()
    {
        if (clock.seconds() > 85 && clock.seconds() <= 90)
            return GameTime.FIVE_UNTIL_ENDGAME;
        else if (clock.seconds() > 90 && clock.seconds() <= 115)
            return GameTime.ENDGAME;
        else if (clock.seconds() > 115 && clock.seconds() <= 120)
            return  GameTime.LAST_5_SECONDS;
        else if (clock.seconds() > 120 && clock.seconds() <= 122)
            return GameTime.MATCH_OVER;
        else
            return GameTime.DRIVER_CONTROLLED;
    }
}
