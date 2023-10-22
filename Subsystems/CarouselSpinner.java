package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CarouselSpinner
{

    private LinearOpMode opMode;

    // TODO: change depending on the stats!
    private static final double WHEEL_DIAMETER = 4; // inches
    private static final double TICKS_PER_REV = ((((1+(46.0/17))) * (1+(46.0/17))) * 28); // 384.5
    private static final double MAX_POWER = .5; // the max power that our motor can go without flinging the duck

    private static final double CAROUSEL_DIAMETER = 15; // inches

    private static final int ONE_FULL_REV = (int)((CAROUSEL_DIAMETER / WHEEL_DIAMETER) * TICKS_PER_REV);

    private DcMotorEx spinMotor;

    public CarouselSpinner(LinearOpMode opMode)
    {
        this.opMode = opMode;

        spinMotor = opMode.hardwareMap.get(DcMotorEx.class, "spinMotor");
        spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        spinMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        spinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinMotor.setTargetPosition(spinMotor.getCurrentPosition());
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinMotor.setPower(0);
    }

    public void carouselRevolutionAuto(Alliance alliance)
    {
        if (alliance == Alliance.BLUE)
        {
            spinMotor.setTargetPosition(spinMotor.getCurrentPosition() + ONE_FULL_REV + 300);
            spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spinMotor.setPower(MAX_POWER);
            while (spinMotor.isBusy() && opMode.opModeIsActive()) {
                opMode.telemetry.addLine("Duck Incoming!");
                opMode.telemetry.update();
            }
            stopMotion();
        }
        else if (alliance == Alliance.RED)
        {
            spinMotor.setTargetPosition(spinMotor.getCurrentPosition() - ONE_FULL_REV - 150);
            spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spinMotor.setPower(MAX_POWER);
            while (spinMotor.isBusy() && opMode.opModeIsActive()) {
                opMode.telemetry.addLine("Duck Incoming!");
                opMode.telemetry.update();
            }
            stopMotion();
        }
    }

    public void spin(Alliance alliance)
    {
        spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (alliance == Alliance.BLUE)
            spinMotor.setPower(MAX_POWER);
        else if (alliance == Alliance.RED)
            spinMotor.setPower(-MAX_POWER);
    }

    public void stopMotion()
    {
        spinMotor.setPower(0);
    }
}
