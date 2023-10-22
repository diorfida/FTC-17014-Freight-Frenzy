//package org.firstinspires.ftc.teamcode.Subsystems.Samples.RotaryArm;
//
//import com.acmerobotics.roadrunner.util.NanoClock;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
///**
// * Simple test of motion-profiled arm autonomous operation. The arm should move *smoothly*
// * between random angles.
// */
//@Disabled
//@Autonomous(group = "arm")
//public class ArmTest extends LinearOpMode
//{
//    @Override
//    public void runOpMode() throws InterruptedException
//    {
//        Arm arm = new Arm(hardwareMap);
//        NanoClock clock = NanoClock.system();
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        while (!isStopRequested())
//        {
//            arm.setAngle(Arm.MAX_ANGLE * Math.random());
//
//            double startTime = clock.seconds();
//            while (!isStopRequested() && (clock.seconds() - startTime) < 5)
//                arm.update();
//        }
//    }
//}
