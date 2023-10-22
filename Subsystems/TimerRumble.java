package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TimerRumble {

    private LinearOpMode opMode;

    public enum MatchTime
    {
        DRIVER_CONTROLLED,
        FIVE_UNTIL_ENDGAME,
        ENDGAME,
        LAST_5_SECONDS,
        MATCH_OVER
    }

    private ElapsedTime clock;
    private static boolean started;

    public TimerRumble(LinearOpMode opMode)
    {
        this.opMode = opMode;
    }

    public void start()
    {
        clock = new ElapsedTime();
        started = true;
    }

    public void run()
    {
        if (!started)
            start();

        switch (determineMatchTime())
        {
            case FIVE_UNTIL_ENDGAME:
                boolean fiveUntilCondition = (clock.seconds() > 85 && clock.seconds() <= 85.5) ||
                        (clock.seconds() > 86 && clock.seconds() <= 86.5) ||
                        (clock.seconds() > 87 && clock.seconds() <= 87.5) ||
                        (clock.seconds() > 88 && clock.seconds() <= 88.5) ||
                        (clock.seconds() > 89 && clock.seconds() <= 89.5);
                if (fiveUntilCondition && !opMode.gamepad1.isRumbling())
                    opMode.gamepad1.rumble(0.5, 0.5, 500);
                break;
            case ENDGAME:
                boolean endgameCondition = clock.seconds() > 90 && clock.seconds() <= 90.5;
                if (endgameCondition && !opMode.gamepad1.isRumbling())
                    opMode.gamepad1.rumble(1, 1, 1000);
                break;
            case LAST_5_SECONDS:
                boolean almostOverCondition = (clock.seconds() > 115 && clock.seconds() <= 115.5) ||
                        (clock.seconds() > 116 && clock.seconds() <= 116.5) ||
                        (clock.seconds() > 117 && clock.seconds() <= 117.5) ||
                        (clock.seconds() > 118 && clock.seconds() <= 118.5) ||
                        (clock.seconds() > 119 && clock.seconds() <= 119.5);
                if (almostOverCondition && !opMode.gamepad1.isRumbling())
                    opMode.gamepad1.rumble(0.5, 0.5, 500);
                break;
            case MATCH_OVER:
                boolean gameOverCondition = clock.seconds() > 120 && clock.seconds() <= 122;
                if (gameOverCondition && !opMode.gamepad1.isRumbling())
                    opMode.gamepad1.rumble(1,1,2000);
        }
    }

    private MatchTime determineMatchTime()
    {
        if (clock.seconds() > 85 && clock.seconds() <= 90)
            return MatchTime.FIVE_UNTIL_ENDGAME;
        else if (clock.seconds() > 90 && clock.seconds() <= 115)
            return MatchTime.ENDGAME;
        else if (clock.seconds() > 115 && clock.seconds() <= 120)
            return MatchTime.LAST_5_SECONDS;
        else if (clock.seconds() > 120 && clock.seconds() <= 122)
            return MatchTime.MATCH_OVER;
        else
            return MatchTime.DRIVER_CONTROLLED;
    }
}
