package org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class TeleopDrive
{
    private LinearOpMode opMode;

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    private static final double POWER_SCALE = Math.tan(1);
    private static final double TRACK_WIDTH = 2; //12

    public TeleopDrive(LinearOpMode opMode)
    {
        this.opMode = opMode;

        frontLeft = opMode.hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = opMode.hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = opMode.hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = opMode.hardwareMap.get(DcMotorEx.class, "backRight");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive()
    {
        double trigger = Math.atan(opMode.gamepad1.right_trigger * POWER_SCALE);
        double xSpeed = -opMode.gamepad1.left_stick_y * trigger;
        double rotationSpeed = -opMode.gamepad1.right_stick_x * trigger;

        Pose2d powerPose = new Pose2d(xSpeed, 0, rotationSpeed);
        setWeightedDrivePower(powerPose);
    }

    public void setWeightedDrivePower(Pose2d drivePower)
    {
        double VX_WEIGHT = 1;
        double OMEGA_WEIGHT = 1;
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getHeading()) > 1)
        {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    0,
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    private void setDrivePower(Pose2d power)
    {
        double leftPower = power.getX() - TRACK_WIDTH / 2 * power.getHeading();
        double rightPower = power.getX() + TRACK_WIDTH / 2 * power.getHeading();

        setMotorPowers(leftPower, rightPower);
    }

    private void setMotorPowers(double left, double right)
    {
        frontLeft.setPower(left);
        backLeft.setPower(left);

        frontRight.setPower(right);
        backRight.setPower(right);
    }
}
