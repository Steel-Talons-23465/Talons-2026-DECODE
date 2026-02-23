package org.firstinspires.ftc.teamcode.hornet.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Configurable
@TeleOp(name = "RPM PIDF Panels", group = "LAUNCHING TUNERS")
public class RPM_PIDF_PANELS extends LinearOpMode {

    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    private PanelsTelemetry panels = PanelsTelemetry.INSTANCE;
    // REV HD Hex encoder (no gearbox)
    private static final double TICKS_PER_REV = 28;

    // Tunable PIDF values (from dashboard)
    public static double kP = 20.0;
    public static double kI = 0.0;
    public static double kD = 2.0;
    public static double kF = 12.0;

    // Individual target RPMs
    public static double targetRPM_left = 2000;
    public static double targetRPM_right = 2000;
    public static double targetRPM_intake = 1500;

    // Direction flags (true = REVERSE, false = FORWARD)
    public static boolean leftReverse = false;
    public static boolean rightReverse = true;

    @Override
    public void runOpMode() {
        // Hardware mapping
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftLaunch");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightLaunch");

        // Reset encoders
        for (DcMotorEx motor : new DcMotorEx[]{leftMotor, rightMotor}) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }


        waitForStart();

        while (opModeIsActive()) {
            // Set directions from dashboard
            leftMotor.setDirection(leftReverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            rightMotor.setDirection(rightReverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

            // Update PIDF for all motors
            for (DcMotorEx motor : new DcMotorEx[]{leftMotor, rightMotor}) {
                motor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
            }

            // Convert RPM to ticks/sec
            double leftVelocity = rpmToTicksPerSecond(targetRPM_left);
            double rightVelocity = rpmToTicksPerSecond(targetRPM_right);

            // Set motor velocities
            leftMotor.setVelocity(leftVelocity);
            rightMotor.setVelocity(rightVelocity);

            // Telemetry for Dashboard
            panels.getTelemetry().addData("Target RPM Left", targetRPM_left);
            panels.getTelemetry().addData("Target RPM Right", targetRPM_right);

            panels.getTelemetry().addData("Left target Vel" , leftVelocity);
            panels.getTelemetry().addData("Right target Vel" , rightVelocity);

            panels.getTelemetry().addData("Left Actual Velocity", leftMotor.getVelocity());
            panels.getTelemetry().addData("Right Actual Velocity", rightMotor.getVelocity());
            panels.getTelemetry().update();

            // Driver Station telemetry
            telemetry.addData("Target RPM Left", targetRPM_left);
            telemetry.addData("Target RPM Right", targetRPM_right);
            telemetry.addData("Target RPM Intake", targetRPM_intake);

            telemetry.addData("Left target Vel" , leftVelocity);
            telemetry.addData("Right target Vel" , rightVelocity);


            telemetry.addData("Left Actual Velocity", leftMotor.getVelocity());
            telemetry.addData("Right Actual Velocity", rightMotor.getVelocity());
            telemetry.update();
        }
    }

    private double rpmToTicksPerSecond(double rpm) {
        return (rpm / 60.0) * TICKS_PER_REV;
    }
}
