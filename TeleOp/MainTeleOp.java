package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Alliance;
import org.firstinspires.ftc.teamcode.Subsystems.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.TeleopDrive;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSystem.ClawIntake;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSystem.RotationArm;
import org.firstinspires.ftc.teamcode.Subsystems.TapeMeasure;
import org.firstinspires.ftc.teamcode.Subsystems.TimerRumble;
import org.firstinspires.ftc.teamcode.Subsystems.DuckPositionsAndLevels;

public class MainTeleOp
{
    private LinearOpMode opMode;
    private Alliance alliance;

    private TeleopDrive drive;
//    private boolean driveToggle = false;
//    private boolean driveDoubleClick = false;

    private TimerRumble timerRumble;

    private RotationArm rotationArm;
    private DuckPositionsAndLevels targetPosition = DuckPositionsAndLevels.FLOOR;

    //private IntakeMotorDriven intake;
    private ClawIntake intake;
    private boolean clawToggle = false;
    private boolean clawDoubleClick = false;

    private CarouselSpinner carouselSpinner;

    private TapeMeasure tapeMeasure;

    //private LEDLights lights;

    public MainTeleOp(LinearOpMode opMode, Alliance alliance)
    {
        opMode.telemetry.addLine("Initializing...");
        opMode.telemetry.update();
        this.opMode = opMode;
        this.alliance = alliance;

        drive = new TeleopDrive(opMode);
        timerRumble = new TimerRumble(opMode);
        rotationArm = new RotationArm(opMode);
        intake = new ClawIntake(opMode.hardwareMap, false);
        carouselSpinner = new CarouselSpinner(opMode);
        //lights = new LEDLights(opMode);
        tapeMeasure = new TapeMeasure(opMode.hardwareMap);

        // initialize starting positions and stuff if necessary

        //lights.initLightsTele(drive.determineAutoPose());

        opMode.telemetry.addLine("OpMode Initialized!");
        opMode.telemetry.addLine();
        opMode.telemetry.addData("Alliance: ", alliance);
        // other things that may want to go here not sure yet
        opMode.telemetry.update();
    }

    public void start()
    {
        timerRumble.start();
        //lights.startTeleOp();
        // maybe start other things?
    }

    public void run()
    {
        // DRIVETRAIN
        drive.drive();

        // THIS IS FOR FIELD-ORIENTED DRIVE MECANUMS ONLY
        // I'll leave it here as an example of toggling tho

//        if (opMode.gamepad1.start && opMode.gamepad1.y && !driveToggle && !driveDoubleClick)
//            driveToggle = true;
//        else if (opMode.gamepad1.start && opMode.gamepad1.y && driveToggle && !driveDoubleClick)
//            driveToggle = false;
//        driveDoubleClick = opMode.gamepad1.start && opMode.gamepad1.y;

        timerRumble.run();

        // ROTATION ARM
        if (opMode.gamepad2.y)
            targetPosition = DuckPositionsAndLevels.RIGHT_LEVEL3;
        else if (opMode.gamepad2.x)
            targetPosition = DuckPositionsAndLevels.MIDDLE_LEVEL2;
        else if (opMode.gamepad2.a)
            targetPosition = DuckPositionsAndLevels.LEFT_LEVEL1;
        else if (opMode.gamepad2.b)
            targetPosition = DuckPositionsAndLevels.FLOOR;
        else if (opMode.gamepad2.dpad_up)
            targetPosition = DuckPositionsAndLevels.HIGH_BACK;
        else if (opMode.gamepad2.dpad_right)
            targetPosition = DuckPositionsAndLevels.SHARED_BACK;
        else if (Math.abs(opMode.gamepad2.right_stick_y) > .25)
        {
            targetPosition = DuckPositionsAndLevels.NONE;
            rotationArm.manualOverride(-opMode.gamepad2.right_stick_y);
        }

        if (targetPosition == DuckPositionsAndLevels.NONE && Math.abs(opMode.gamepad2.right_stick_y) <= .25)
            rotationArm.manualOverride(0);

        rotationArm.run(targetPosition, false);
        opMode.telemetry.addData("Pos:", targetPosition);

        // INTAKE
        intake.runClaw(clawToggle);
        if (opMode.gamepad2.left_bumper && !clawToggle && !clawDoubleClick)
            clawToggle = true;
        else if (opMode.gamepad2.left_bumper && clawToggle && !clawDoubleClick)
            clawToggle = false;
        clawDoubleClick = opMode.gamepad2.left_bumper;

        // OLD INTAKE STUFF FOR MOTOR DRIVEN
//        if (opMode.gamepad2.right_trigger > .5)
//            intake.runIntake();
//        else if (opMode.gamepad2.left_trigger > .5)
//            intake.outtake();
//        else
//            intake.stopIntake();

        // CAROUSEL WHEEL
        if (opMode.gamepad2.dpad_down)
            carouselSpinner.spin(alliance);
        else
            carouselSpinner.stopMotion();

        tapeMeasure.run( opMode.gamepad2.left_stick_y);

        // UPDATE THE TELEMETRY
        opMode.telemetry.update();
    }

//    public void terminate()
//    {
//        PoseStorage.storedPose = new Pose2d();
//    }
}
