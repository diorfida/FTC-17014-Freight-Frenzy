package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Alliance;

@TeleOp(name = "Red TeleOp", group = "red")
public class RedClass extends LinearOpMode
{

    private static final Alliance alliance = Alliance.RED;

    @Override
    public void runOpMode() throws InterruptedException
    {
        MainTeleOp teleOp = new MainTeleOp(this, alliance);

        waitForStart();

        teleOp.start();

        while (opModeIsActive() && !isStopRequested())
        {
            teleOp.run();
        }
//        teleOp.terminate();
    }
}
