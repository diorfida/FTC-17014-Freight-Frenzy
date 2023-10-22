package org.firstinspires.ftc.teamcode.Subsystems.IntakeSystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Subsystems.DuckPositionsAndLevels;

public class RotationArm {


    private LinearOpMode opMode;

    private DcMotorEx rotation;
    private static final double TICKS_PER_REV = (((((1+(46.0/11))) * (1+(46.0/11))) * (1+(46.0/11))) * 28);

    private static final int floorPos = 0;
    private static final int level1Pos = 955;
    private static final int level2Pos = 1650;
    private static final int level3Pos = 2500;
    private static final int backHighPos = 6050;
    private static final int backSharedPos = 7000;
    // DOWN POSITION == 7000

    private static final double POWER = 1;

    public RotationArm(LinearOpMode opMode)
    {
        this.opMode = opMode;
        rotation = opMode.hardwareMap.get(DcMotorEx.class, "rotation");
        rotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotation.setDirection(DcMotorSimple.Direction.REVERSE);
        rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotation.setTargetPosition(floorPos);
        rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotation.setPower(POWER);
    }

    public void run(DuckPositionsAndLevels position, boolean auto)
    {
        if (position != DuckPositionsAndLevels.NONE) {
            if (position == DuckPositionsAndLevels.RIGHT_LEVEL3)
                rotation.setTargetPosition(level3Pos);
            else if (position == DuckPositionsAndLevels.MIDDLE_LEVEL2)
                rotation.setTargetPosition(level2Pos);
            else if (position == DuckPositionsAndLevels.LEFT_LEVEL1)
                rotation.setTargetPosition(level1Pos);
            else if (position == DuckPositionsAndLevels.SHARED_BACK)
                rotation.setTargetPosition(backSharedPos);
            else if (position == DuckPositionsAndLevels.HIGH_BACK)
                rotation.setTargetPosition(backHighPos);
            else
                rotation.setTargetPosition(floorPos);

            rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotation.setPower(POWER);

            if (auto) {
                while (opMode.opModeIsActive() && rotation.isBusy()) {
                    opMode.telemetry.addData("Rotation Arm Position:", rotation.getCurrentPosition());
                    opMode.telemetry.update();
                }
            }
            opMode.telemetry.addData("Rotation Arm Position:", rotation.getCurrentPosition());
        }
    }

    public void manualOverride(double power)
    {
        rotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (Math.abs(power) > .25)
        {
            rotation.setPower(power);
        }
        else
            rotation.setPower(0);

    }
}
