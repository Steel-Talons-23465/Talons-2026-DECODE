package org.firstinspires.ftc.teamcode.midas.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
@Config
@TeleOp(name = "RPM PIDF SINGLE", group = "LAUNCH TUNERS")
public class RPM_PIDF_DASHBOARD_SINGLE extends LinearOpMode {

    private DcMotorEx shooter;
    private VoltageSensor voltageSensor;

    // REV HD Hex encoder (no gearbox)
    private static final double TICKS_PER_REV = 28;

    // Tunable PIDF values (from dashboard)
    public static double kP = 20.0;
    public static double kI = 0.0;
    public static double kD = 2.0;
    public static double kF = 12.0;

    public static boolean RPM_MODE = false;
    // Individual target RPMs
    public static double targetVel = 0;
    public static double targetRPM = 0;
    public static double volts;
    // Direction flags (true = REVERSE, false = FORWARD)
    public static boolean reverse = false;
    @Override
    public void runOpMode() {
        // Hardware mapping
        shooter = hardwareMap.get(DcMotorEx.class, "launch");
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        // Reset encoders
        shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        FtcDashboard dashboard = FtcDashboard.getInstance();
        waitForStart();

        while (opModeIsActive()) {
            // Set directions from dashboard
            shooter.setDirection(reverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

            // Update PIDF for all motors
            shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER , new PIDFCoefficients(kP , kI , kD , kF));
            // Convert RPM to ticks/sec

            // Set motor velocities
            shooter.setVelocity(RPM_MODE ? rpmToTicksPerSecond(targetRPM) : targetVel);

            volts = voltageSensor.getVoltage();
            // Telemetry packet for Dashboard
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Target VEL", targetVel);
            packet.put("volts", volts);

            packet.put("Target RPM" , targetRPM);
            packet.put("Actual Velocity", shooter.getVelocity());

            dashboard.sendTelemetryPacket(packet);

            // Driver Station telemetry
            telemetry.addData("Target VEL", targetVel);
            telemetry.addData("Target RPM" , targetRPM);
            packet.put("volts", volts);

            telemetry.addData("Actual Velocity", shooter.getVelocity());

            telemetry.update();
        }
    }

    private double rpmToTicksPerSecond(double rpm) {
        return (rpm / 60.0) * TICKS_PER_REV;
    }
}