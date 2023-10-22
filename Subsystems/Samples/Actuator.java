//package org.firstinspires.ftc.teamcode.Subsystems.Samples;
//
//import com.qualcomm.hardware.motors.GoBILDA5202Series;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
//
//public class Actuator
//{
//    private static final double MAX_ENCODER_POSITION = 1000;
//    private static final double MIN_ENCODER_POSITION = 0;
//
//    private DcMotorEx actuatorMotor;
//
//    public Actuator(HardwareMap hardwareMap)
//    {
//        // TODO: replace string with motor name
//        actuatorMotor = hardwareMap.get(DcMotorEx.class, "actuator");
//        actuatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        // TODO: if necessary, reverse the motor so "out" is positive
//        // motor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        actuatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        actuatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    public void runTele(double speed)
//    {
//        if ((speed > 0 && actuatorMotor.getCurrentPosition() < MAX_ENCODER_POSITION)
//        || (speed < 0 && actuatorMotor.getCurrentPosition() > MIN_ENCODER_POSITION))
//            actuatorMotor.setPower(speed);
//        else
//            actuatorMotor.setPower(0);
//    }
//}
