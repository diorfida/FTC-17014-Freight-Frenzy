//package org.firstinspires.ftc.teamcode.Subsystems.Samples.RotaryArm;
//
//import com.acmerobotics.roadrunner.util.NanoClock;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcore.internal.system.Misc;
//import org.firstinspires.ftc.teamcode.RRThings.Util.LoggingUtil;
//import org.firstinspires.ftc.teamcode.RRThings.Util.RegressionUtil;
//
//import java.util.ArrayList;
//import java.util.List;
//
///**
// * Op mode for computing kV, kStatic, and kA from various arm motions. Note: for those using the
// * built-in PID, **kStatic and kA should not be tuned**. For the curious, here's an outline of the
// * basic procedure:
// *   1. Slowly ramp the motor power and record encoder values along the way.
// *   2. Run a linear regression on the encoder velocity vs. motor power plot to obtain a slope (kV)
// *      and an optional intercept (kStatic).
// *   3. Accelerate the arm (apply constant power) and record the encoder counts.
// *   4. Adjust the encoder data based on the velocity tuning data and find kA with another linear
// *      regression.
// */
//@Disabled
////@Config
//@Autonomous(group = "arm")
//public class ArmFeedforwardTuner extends LinearOpMode
//{
//    public static final double MAX_POWER = 0.7;
//    public static final double ANGLE = .75 * Arm.MAX_ANGLE;
//
//    @Override
//    public void runOpMode() throws InterruptedException
//    {
//        Arm arm = new Arm(hardwareMap);
//
//        NanoClock clock = NanoClock.system();
//
//        telemetry.log().add("Press play to begin the feedforward tuning routine");
//        telemetry.update();
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        telemetry.log().clear();
//        telemetry.log().add("Would you like to fit kStatic?");
//        telemetry.log().add("Press (A) for yes, (B) for no");
//        telemetry.update();
//
//        boolean fitIntercept = false;
//        while (!isStopRequested()) {
//            if (gamepad1.a) {
//                fitIntercept = true;
//                while (!isStopRequested() && gamepad1.a) {
//                    idle();
//                }
//                break;
//            } else if (gamepad1.b) {
//                while (!isStopRequested() && gamepad1.b) {
//                    idle();
//                }
//                break;
//            }
//            idle();
//        }
//
//        telemetry.log().clear();
//        telemetry.log().add("Press (A) to begin");
//        telemetry.update();
//
//        while (!isStopRequested() && !gamepad1.a) {
//            idle();
//        }
//        while (!isStopRequested() && gamepad1.a) {
//            idle();
//        }
//
//        telemetry.log().clear();
//        telemetry.log().add("Running...");
//        telemetry.update();
//
//        double maxVel = Arm.rpmToVelocity(Arm.getMaxRpm());
//        double finalVel = MAX_POWER * maxVel;
//        double accel = (finalVel * finalVel) / (2.0 * ANGLE);
//        double rampTime = Math.sqrt(2.0 * ANGLE / accel);
//
//        List<Double> timeSamples = new ArrayList<>();
//        List<Double> positionSamples = new ArrayList<>();
//        List<Double> powerSamples = new ArrayList<>();
//
//        double startTime = clock.seconds();
//        while (!isStopRequested()) {
//            double elapsedTime = clock.seconds() - startTime;
//            if (elapsedTime > rampTime) {
//                break;
//            }
//            double vel = accel * elapsedTime;
//            double power = vel / maxVel;
//
//            timeSamples.add(elapsedTime);
//            positionSamples.add(arm.getCurrentAngle());
//            powerSamples.add(power);
//
//            arm.setPower(power);
//        }
//        arm.setPower(0);
//
//        RegressionUtil.RampResult rampResult = RegressionUtil.fitRampData(
//                timeSamples, positionSamples, powerSamples, fitIntercept,
//                LoggingUtil.getLogFile(Misc.formatInvariant(
//                        "ArmRampRegression-%d.csv", System.currentTimeMillis()
//                ))
//        );
//
//        telemetry.log().clear();
//        telemetry.log().add("Quasi-static ramp up test complete");
//        if (fitIntercept) {
//            telemetry.log().add(Misc.formatInvariant("kV = %.5f, kStatic = %.5f (R^2 = %.2f)",
//                    rampResult.kV, rampResult.kStatic, rampResult.rSquare));
//        } else {
//            telemetry.log().add(Misc.formatInvariant("kV = %.5f (R^2 = %.2f)",
//                    rampResult.kV, rampResult.rSquare));
//        }
//        telemetry.log().add("Would you like to fit kA?");
//        telemetry.log().add("Press (A) for yes, (B) for no");
//        telemetry.update();
//
//        boolean fitAccelFF = false;
//        while (!isStopRequested()) {
//            if (gamepad1.a) {
//                fitAccelFF = true;
//                while (!isStopRequested() && gamepad1.a) {
//                    idle();
//                }
//                break;
//            } else if (gamepad1.b) {
//                while (!isStopRequested() && gamepad1.b) {
//                    idle();
//                }
//                break;
//            }
//            idle();
//        }
//
//        if (fitAccelFF) {
//            telemetry.log().clear();
//            telemetry.log().add("Press (A) to continue");
//            telemetry.update();
//
//            while (!isStopRequested() && !gamepad1.a) {
//                idle();
//            }
//            while (!isStopRequested() && gamepad1.a) {
//                idle();
//            }
//
//            telemetry.log().clear();
//            telemetry.log().add("Running...");
//            telemetry.update();
//
//            double maxPowerTime = 0.75 * ANGLE / maxVel; // 0.75 = "safety factor"
//
//            timeSamples.clear();
//            positionSamples.clear();
//            powerSamples.clear();
//
//            arm.setPower(-MAX_POWER);
//
//            startTime = clock.seconds();
//            while (!isStopRequested()) {
//                double elapsedTime = clock.seconds() - startTime;
//                if (elapsedTime > maxPowerTime) {
//                    break;
//                }
//                timeSamples.add(elapsedTime);
//                positionSamples.add(arm.getCurrentAngle());
//                powerSamples.add(MAX_POWER);
//            }
//            arm.setPower(0);
//
//            RegressionUtil.AccelResult accelResult = RegressionUtil.fitAccelData(
//                    timeSamples, positionSamples, powerSamples, rampResult,
//                    LoggingUtil.getLogFile(Misc.formatInvariant(
//                            "ArmAccelRegression-%d.csv", System.currentTimeMillis()
//                    ))
//            );
//
//            telemetry.log().clear();
//            telemetry.log().add("Constant power test complete");
//            telemetry.log().add(Misc.formatInvariant("kA = %.5f (R^2 = %.2f)",
//                    accelResult.kA, accelResult.rSquare));
//            telemetry.update();
//        }
//
//        while (!isStopRequested()) {
//            idle();
//        }
//    }
//}
