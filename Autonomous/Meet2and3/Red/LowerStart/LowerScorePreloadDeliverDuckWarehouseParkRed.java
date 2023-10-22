package org.firstinspires.ftc.teamcode.Autonomous.Meet2and3.Red.LowerStart;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Alliance;
import org.firstinspires.ftc.teamcode.Subsystems.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.AutoDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.AutoMath;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSystem.ClawIntake;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSystem.RotationArm;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.Detector;
import org.firstinspires.ftc.teamcode.Subsystems.DuckPositionsAndLevels;

@Autonomous(name = "Lower Score Preload Deliver Duck Warehouse Park Red", group = "red", preselectTeleOp = "Red TeleOp")
public class LowerScorePreloadDeliverDuckWarehouseParkRed extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        AutoDrive drive = new AutoDrive(this);
        RotationArm rotationArm = new RotationArm(this);
        ClawIntake intake = new ClawIntake(hardwareMap, true);
        CarouselSpinner carouselSpinner = new CarouselSpinner(this);
        Detector detector = new Detector(this);
        DuckPositionsAndLevels position = DuckPositionsAndLevels.LEFT_LEVEL1;

        while (!isStarted())
        {
            position = detector.detect();

            telemetry.addLine("Ready!");
            telemetry.addLine();
            telemetry.addLine("Reminder: Start on the lower barcode to the left of the tile.");
            telemetry.addLine();
            telemetry.addData("Adjustable Start:", AutoMath.ADJUSTABLE_START);
            telemetry.addLine();
            telemetry.addData("Marker Position", position);
            telemetry.update();
        }

        waitForStart();

        sleep(AutoMath.ADJUSTABLE_START);

        drive.gyroDrive(39.0, 0.0);

        rotationArm.run(position, true);

        drive.gyroTurn(-90.0);

        drive.gyroDrive(AutoMath.determineAdj(position), -90.0);
        intake.runClaw(false);
        sleep(1000);
        drive.gyroDrive(-AutoMath.determineAdj(position), -90.0);

        drive.gyroTurn(0.0);
        drive.gyroDrive(-20, 0.0);

        drive.gyroTurn(-46.0);
        drive.gyroDrive(-28.0, -46.0);

        carouselSpinner.carouselRevolutionAuto(Alliance.RED);

        drive.gyroDrive(23.5, -46.0);
        drive.gyroTurn(90.0);
        drive.gyroDrive(-85.0, 100.0);

        rotationArm.run(DuckPositionsAndLevels.FLOOR, true);
    }
}
