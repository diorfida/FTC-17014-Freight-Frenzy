//package org.firstinspires.ftc.teamcode.Autonomous;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.Subsystems.Alliance;
//import org.firstinspires.ftc.teamcode.RRThings.FierceTankDrive;
//import org.firstinspires.ftc.teamcode.Subsystems.LEDLights;
//
//// TODO: Remove the disabled tag to activate the opMode
//@Disabled
//@Autonomous(name = "auto name here", group = "alliance color (either red or blue)")
//public class AutoTemplate extends LinearOpMode
//{
//    //TODO: Set the alliance
//    Alliance alliance = Alliance.RED;
//
//    @Override
//    public void runOpMode() throws InterruptedException
//    {
//        // TODO: Hardware goes here
//        FierceTankDrive drive = new FierceTankDrive(hardwareMap);
//        LEDLights lights = new LEDLights(this);
//
//        lights.initLightsAuto();
//        waitForStart();
//        lights.autonomousLights(alliance);
//
//        // TODO: Auto runs here!
//
//        // Don't forget to update the pose estimate for use in tele!
//        // Although with the bump this is probably not necessary
////        drive.update();
////        PoseStorage.storedPose = drive.getPoseEstimate();
//    }
//}
