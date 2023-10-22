package org.firstinspires.ftc.teamcode.Autonomous.Meet2and3.Blue.LowerStart;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Alliance;
import org.firstinspires.ftc.teamcode.Subsystems.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.AutoDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.AutoMath;

@Disabled
@Deprecated
@Autonomous(name = "Lower Deliver Duck Storage Unit Park Blue", group = "blue", preselectTeleOp = "Blue TeleOp")
public class LowerDeliverDuckStorageUnitParkBlue extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        AutoDrive drive = new AutoDrive(this);
        CarouselSpinner carouselSpinner = new CarouselSpinner(this);

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

        drive.gyroDrive(10.0, 0.0);
        drive.gyroTurn(65.0);
        drive.gyroDrive(-27.5, 65);

        carouselSpinner.carouselRevolutionAuto(Alliance.BLUE);

        drive.gyroDrive(27.5, 65);
        drive.gyroTurn(0.0);
        drive.gyroDrive(17.5, 0.0);
        drive.gyroTurn(90.0);
        drive.gyroDrive(-27.5, 90.0);
    }
}
