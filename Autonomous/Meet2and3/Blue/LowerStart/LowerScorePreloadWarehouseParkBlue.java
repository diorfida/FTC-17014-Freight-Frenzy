package org.firstinspires.ftc.teamcode.Autonomous.Meet2and3.Blue.LowerStart;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.AutoDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.AutoMath;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSystem.ClawIntake;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSystem.RotationArm;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.Detector;
import org.firstinspires.ftc.teamcode.Subsystems.DuckPositionsAndLevels;

@Autonomous(name = "Lower Score Preload Warehouse Park Blue", group = "blue", preselectTeleOp = "Blue TeleOp")
public class LowerScorePreloadWarehouseParkBlue extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        AutoDrive drive = new AutoDrive(this);
        RotationArm rotationArm = new RotationArm(this);
        ClawIntake intake = new ClawIntake(hardwareMap, true);
        Detector detector = new Detector(this);
        DuckPositionsAndLevels position = DuckPositionsAndLevels.LEFT_LEVEL1;

        while (!isStarted())
        {
            position = detector.detect();

            telemetry.addLine("Ready!");
            telemetry.addLine();
            telemetry.addLine("Reminder: Start on the lower barcode to the right of the tile.");
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

        drive.gyroTurn(90.0);

        drive.gyroDrive(AutoMath.determineAdj(position), 90.0);
        intake.runClaw(false);
        sleep(1000);
        drive.gyroDrive(-AutoMath.determineAdj(position), 90.0);

        //drive.gyroHold(AutoDrive.DRIVE_SPEED, 90.0, 5.0);

        drive.gyroTurn(0.0);
        drive.gyroDrive(-24.0, 0.0);
        drive.gyroTurn(-80.0);
        drive.gyroDrive(-87.5, -80.0);

        rotationArm.run(DuckPositionsAndLevels.FLOOR, true);
    }
}
