package org.firstinspires.ftc.teamcode.midas;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Motor Direction Test")
public class MotorDirectionTest extends LinearOpMode {

    private DcMotorEx turret = null;


    @Override
    public void runOpMode() {
        turret = hardwareMap.get(DcMotorEx.class, "turret");

        // Put initialization blocks here.
        waitForStart();
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive()) {
            turret.setPower(.5);
            turret.setTargetPosition(150);
            telemetry.addData("currentpos", turret.getCurrentPosition());
            telemetry.update();
        }
    }
}
