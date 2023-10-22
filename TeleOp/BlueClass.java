package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Alliance;

@TeleOp(name = "Blue TeleOp", group = "blue")
public class BlueClass extends LinearOpMode
{

    private static final Alliance alliance = Alliance.BLUE;

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
