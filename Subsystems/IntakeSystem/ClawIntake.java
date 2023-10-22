package org.firstinspires.ftc.teamcode.Subsystems.IntakeSystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class ClawIntake
{
    private ServoImplEx claw;
    private final PwmControl.PwmRange clawRange = new PwmControl.PwmRange(950, 1200);

//    private ServoImplEx rightClaw;
//    private final PwmControl.PwmRange rightClawRange = new PwmControl.PwmRange(500, 2500);

    public ClawIntake(HardwareMap hardwareMap, boolean autonomous)
    {
        claw = hardwareMap.get(ServoImplEx.class, "claw");
        claw.setDirection(Servo.Direction.FORWARD);
        claw.setPwmRange(clawRange);

//        rightClaw = hardwareMap.get(ServoImplEx.class, "rightClaw");
//        rightClaw.setDirection(Servo.Direction.FORWARD);
//        leftClaw.setPwmRange(rightClawRange);

        runClaw(autonomous);
    }

    public void runClaw(boolean closed)
    {
        if (closed)
            claw.setPosition(1);
        else
            claw.setPosition(0);
    }
}
