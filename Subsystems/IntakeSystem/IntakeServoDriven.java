package org.firstinspires.ftc.teamcode.Subsystems.IntakeSystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeServoDriven
{

    private CRServo intakeLeft;
    private CRServo intakeRight;

    public IntakeServoDriven(HardwareMap hardwareMap)
    {
        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");
        intakeRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void runIntake()
    {
        intakeLeft.setPower(1);
        intakeRight.setPower(1);
    }

    public void stopIntake()
    {
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
    }

    public void outtake()
    {
        intakeLeft.setPower(-1);
        intakeRight.setPower(-1);
    }
}
