package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class DetectorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Detector detector = new Detector(this);

        telemetry.addLine("Ready! Press Start to begin");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            detector.detect();
        }
    }
}
