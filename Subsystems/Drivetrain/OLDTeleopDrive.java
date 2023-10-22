//package org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.RRThings.FierceTankDrive;
//
//public class OLDTeleopDrive extends FierceTankDrive {
//
//    private LinearOpMode opMode;
//    private static final double POWER_SCALE = Math.tan(1);
//
//    public OLDTeleopDrive(LinearOpMode opMode)
//    {
//        super(opMode.hardwareMap);
//        this.opMode = opMode;
//
////        if (determineAutoPoseExistence())
////            setPoseEstimate(PoseStorage.storedPose);
//    }
//
////    public boolean determineAutoPoseExistence()
////    {
////        return !PoseStorage.storedPose.equals(new Pose2d());
////    }
//
////    public void actuallyDrive()
////    {
////        double trigger = Math.atan(opMode.gamepad1.right_trigger * POWER_SCALE);
////        double xSpeed = -opMode.gamepad1.left_stick_y * trigger;
////        double ySpeed = -opMode.gamepad1.right_stick_x * trigger;
////        double rotationSpeed = -opMode.gamepad1.right_stick_x * trigger;
////
//////        if (Math.abs(rotationSpeed) > 0)
//////            setMotorPowers(-rotationSpeed, rotationSpeed);
//////        else if (Math.abs(xSpeed) > 0)
//////            setMotorPowers(xSpeed, xSpeed);
//////        else
//////            setMotorPowers(0,0);
////
////        Pose2d powerPose = new Pose2d(xSpeed, ySpeed, rotationSpeed);
////        setWeightedDrivePower(powerPose);
////
////        update();
////    }
//
//    public void drive()
//    {
//        double sensitivityTrigger = Math.atan(opMode.gamepad1.right_trigger * POWER_SCALE);
//        double xSpeed = -opMode.gamepad1.left_stick_y * sensitivityTrigger;
//        double rotationSpeed = -opMode.gamepad1.right_stick_x * sensitivityTrigger;
//
//        setWeightedDrivePower(new Pose2d(xSpeed, 0, rotationSpeed));
//
//        update();
//    }
//}