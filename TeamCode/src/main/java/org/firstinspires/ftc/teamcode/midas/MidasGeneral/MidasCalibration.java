package org.firstinspires.ftc.teamcode.midas.MidasGeneral;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hornet.General.SharedData;
import org.firstinspires.ftc.teamcode.hornet.General.Side;

@TeleOp
public class MidasCalibration extends LinearOpMode {

    boolean shootF;
    boolean startF;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        SharedData.reset();
        while(opModeIsActive())
        {
            if(gamepad1.a || gamepad2.a)
                SharedData.side = Side.BLUE;
            else if(gamepad1.b || gamepad2.b)
                SharedData.side = Side.RED;
            else if((gamepad1.x || gamepad2.x) && (gamepad1.x || gamepad2.x) != shootF)
                SharedData.shootFar = !SharedData.shootFar;
            else if((gamepad1.y || gamepad2.y) && (gamepad1.y || gamepad2.y) != startF)
                SharedData.startFar = !SharedData.startFar;
            shootF = gamepad1.x || gamepad2.x;
            startF = gamepad1.y || gamepad2.y;
            telemetry.addLine("B to set to red\nA to set to blue\nY to change start spot\nX to change shoot spot\n");
            telemetry.addData("Side", SharedData.side);
            telemetry.addData("Shooting", SharedData.shootFar ? "far" : "close");
            telemetry.addData("Starting", SharedData.startFar ? "far" : "close");
            telemetry.update();
        }

    }
}
