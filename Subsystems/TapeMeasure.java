package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TapeMeasure
{
    private DcMotorEx tapeMeasure;
    private static final double threshold = 0.2;

    public TapeMeasure(HardwareMap hardwareMap)
    {
        tapeMeasure = hardwareMap.get(DcMotorEx.class, "tapeMeasure");
    }

    public void run(double power)
    {
        if (Math.abs(power) > threshold)
            tapeMeasure.setPower(power);
        else
            tapeMeasure.setPower(0);
    }
}
