package org.firstinspires.ftc.teamcode.MetricRR;

import static org.firstinspires.ftc.teamcode.MetricRR.MetricConversions.toMeters;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

// EVERYTHING NEEDS TO BE IN METERS FOR RAMSETE!

@Config
public class DrivetrainConstants {

    public static final double TICKS_PER_REV = ((((1+(46.0/17))) * (1+(46.0/11))) * 28);
    public static final double MAX_RPM = 312;

    public static final boolean RUN_USING_ENCODER = false;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(35, 0, 12,
            13.8);

    public static double WHEEL_RADIUS = (115.0/2)/1000; // meters
    public static double GEAR_RATIO = (24.0/36.0) * (toMeters(51) / 1.609742867537); // output (wheel) speed / input (encoder) speed
    public static double TRACK_WIDTH = toMeters(27.895);

    public static double kV = 0.745;//1.0 / rpmToVelocity(MAX_RPM);
    public static double kA = 0.085;
    public static double kStatic = 0.06381;

    public static double MAX_VEL = toMeters(30); // 45
    public static double MAX_ACCEL = toMeters(30); //37.5
    public static double MAX_ANG_VEL = Math.toRadians(90); //180
    public static double MAX_ANG_ACCEL = Math.toRadians(90); //180

    public static double encoderTicksToMeters(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}
