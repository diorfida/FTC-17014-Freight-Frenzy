package org.firstinspires.ftc.teamcode.Subsystems.IntakeSystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeMotorDriven
{

    private DcMotorEx intake1;
    private DcMotorEx intake2;

    public IntakeMotorDriven(HardwareMap hardwareMap)
    {
        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake1.setDirection(DcMotorSimple.Direction.REVERSE);

        intake2 = hardwareMap.get(DcMotorEx.class, "intake2");
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void runIntake()
    {
        intake1.setPower(1);
        intake2.setPower(1);
    }
    public void stopIntake()
    {
        intake1.setPower(0);
        intake2.setPower(0);
    }
    public void outtake()
    {
        intake1.setPower(-1);
        intake2.setPower(-1);
    }
}
