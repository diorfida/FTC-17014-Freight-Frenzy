package org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class AutoDrive
{
    private LinearOpMode opMode;

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    private BNO055IMU imu;

    private static final double COUNTS_PER_MOTOR_REV = ((((1+(46.0/17))) * (1+(46.0/11))) * 28);
    private static final double WHEEL_DIAMETER = 94.5/25.4; // inches 115

    private static final double GEARBOX_REDUCTION = (36.0/24); //(24.0/36) (36.0/24)

    private static final double ENCODER_COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * GEARBOX_REDUCTION) / (WHEEL_DIAMETER * Math.PI);

    public static final double DRIVE_SPEED = 0.5;
    public static final double TURN_SPEED = 0.5;

    private static final double HEADING_THRESHOLD = 1;
    private static final double P_TURN_COEFFICIENT = 0.325; // Larger is more responsive, but also less stable
    private static final double P_DRIVE_COEFFICIENT = 0.1; // Larger is more responsive, but also less stable

    public AutoDrive(LinearOpMode opMode)
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

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void gyroDrive(double distance, double angle)
    {
        double speed = DRIVE_SPEED;
        int newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget, newRightBackTarget;
        int moveCounts;
        double max, error, steer;
        double leftSpeed, rightSpeed;

        if (opMode.opModeIsActive())
        {
            moveCounts = (int)(distance * ENCODER_COUNTS_PER_INCH);

            newLeftFrontTarget = frontLeft.getCurrentPosition() + moveCounts;
            newLeftBackTarget = backLeft.getCurrentPosition() + moveCounts;
            newRightFrontTarget = frontRight.getCurrentPosition() + moveCounts;
            newRightBackTarget = backRight.getCurrentPosition() + moveCounts;

            frontLeft.setTargetPosition(newLeftFrontTarget);
            backLeft.setTargetPosition(newLeftBackTarget);
            frontRight.setTargetPosition(newRightFrontTarget);
            backRight.setTargetPosition(newRightBackTarget);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            frontLeft.setPower(speed);
            backLeft.setPower(speed);
            frontRight.setPower(speed);
            backRight.setPower(speed);

            while (opMode.opModeIsActive() &&
                    (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()))
            {
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFFICIENT);

                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                frontLeft.setPower(leftSpeed);
                backLeft.setPower(leftSpeed);
                frontRight.setPower(rightSpeed);
                backRight.setPower(rightSpeed);

                opMode.telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                opMode.telemetry.addData("Target",  "%7d:%7d:7d:%7d",      newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget, newRightBackTarget);
                opMode.telemetry.addData("Actual",  "%7d:%7d",      frontLeft.getCurrentPosition(),
                        backLeft.getCurrentPosition(), frontRight.getCurrentPosition(), backRight.getCurrentPosition());
                opMode.telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                opMode.telemetry.update();
            }

            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);

            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            gyroHold(speed, angle, 0.5);
        }
    }

    public void gyroTurn(double angle)
    {
        double speed = TURN_SPEED;
        while (opMode.opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFFICIENT))
            opMode.telemetry.update();

        gyroHold(speed, angle, 0.5);
    }

    public void gyroHold(double speed, double angle, double holdTime)
    {
        ElapsedTime holdTimer = new ElapsedTime();

        holdTimer.reset();
        while (opMode.opModeIsActive() && (holdTimer.seconds() < holdTime))
        {
            onHeading(speed, angle, P_TURN_COEFFICIENT);
            opMode.telemetry.update();
        }

        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    private boolean onHeading(double speed, double angle, double PCoefficient)
    {
        double error, steer;
        boolean onTarget = false;
        double leftSpeed, rightSpeed;

        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD)
        {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else
        {
            steer = getSteer(error, PCoefficient);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        frontLeft.setPower(leftSpeed);
        backLeft.setPower(leftSpeed);

        frontRight.setPower(rightSpeed);
        backRight.setPower(rightSpeed);

        opMode.telemetry.addData("Target", "%5.2f", angle);
        opMode.telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        opMode.telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    private double getError(double targetAngle)
    {
        double robotError;

        robotError = targetAngle - imu.getAngularOrientation().firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;

        return robotError;
    }

    private double getSteer(double error, double PCoefficient)
    {
        return Range.clip(error * PCoefficient, -1, 1);
    }
}
