package org.firstinspires.ftc.teamcode.Autonomous.Meet2and3.Red.LowerStart;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.AutoDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.AutoMath;

@Autonomous(name = "Lower Storage Unit Park Red", group = "red", preselectTeleOp = "Red TeleOp")
public class LowerStorageUnitParkRed extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        AutoDrive drive = new AutoDrive(this);

        telemetry.addLine("Ready!");
        telemetry.addLine();
        telemetry.addLine("Reminder: Start on the lower barcode to the left of the tile.");
        telemetry.addLine();
        telemetry.addData("Adjustable Start:", AutoMath.ADJUSTABLE_START);
        telemetry.addLine();
        telemetry.addLine("Non-detecting opmode :)");
        telemetry.update();

        waitForStart();

        sleep(AutoMath.ADJUSTABLE_START);

        drive.gyroDrive(27.5, 0.0);
        drive.gyroTurn(-80.0);
        drive.gyroDrive(-22.5, -80.0);
    }
}
