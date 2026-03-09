package org.firstinspires.ftc.teamcode.midas.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Configurable
@Config
@TeleOp
        (name = "SWIVEL [DOESN'T WORK]" , group = "TURRET TUNERS")
public class SWIVEL_PIDF_DASHBOARD extends LinearOpMode {
    public static double p, i, d, f;
    public static boolean RPM_MODE = false;
    // Individual target RPMs
    public static double targetVel = 0;
    public static double targetRPM = 0;
    public static boolean reverse;
//    private static double TICKS_PER_REV = 28;

    DcMotorEx turret = null;
    public void runOpMode() throws InterruptedException{

        turret = hardwareMap.get(DcMotorEx.class , "turret");
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FtcDashboard dashboard = FtcDashboard.getInstance();


        waitForStart();

        while (opModeIsActive()){
            turret.setDirection(reverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

            // Update PIDF for all motors
            turret.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER , new PIDFCoefficients(p , i , d , f));
            // Convert RPM to ticks/sec

            // Set motor velocities
            turret.setVelocity(targetVel);


            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Target VEL", targetVel);

            packet.put("Target RPM" , targetRPM);
            packet.put("Actual Velocity", turret.getVelocity());

            dashboard.sendTelemetryPacket(packet);

            // Driver Station telemetry
            telemetry.addData("Target VEL", targetVel);
            telemetry.addData("Target RPM" , targetRPM);

            telemetry.addData("Actual Velocity", turret.getVelocity());

            telemetry.update();

        }

    }
}

