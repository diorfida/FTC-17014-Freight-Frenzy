package org.firstinspires.ftc.teamcode.Autonomous.Meet2and3.Blue.LowerStart;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.AutoDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.AutoMath;
import org.firstinspires.ftc.teamcode.Subsystems.DuckPositionsAndLevels;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSystem.RotationArm;

@Autonomous(name = "Lower Warehouse Park Blue", group = "blue", preselectTeleOp = "Blue TeleOp")
public class LowerWarehouseParkBlue extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        AutoDrive drive = new AutoDrive(this);
        RotationArm rotationArm = new RotationArm(this);

        telemetry.addLine("Ready!");
        telemetry.addLine();
        telemetry.addLine("Reminder: Start on the lower barcode to the right of the tile.");
        telemetry.addLine();
        telemetry.addData("Adjustable Start:", AutoMath.ADJUSTABLE_START);
        telemetry.addLine();
        telemetry.addLine("Non-detecting opmode :)");
        telemetry.update();

        waitForStart();

        sleep(AutoMath.ADJUSTABLE_START);

        drive.gyroDrive(20.0, 0.0);
        drive.gyroTurn(-90.0);

        rotationArm.run(DuckPositionsAndLevels.LEFT_LEVEL1, true);

        drive.gyroDrive(-80.0, -90.0);

        rotationArm.run(DuckPositionsAndLevels.FLOOR, true);
    }
}
