package org.firstinspires.ftc.teamcode.midas.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp
public class DriveTrainTest extends LinearOpMode {
    private DcMotorEx leftFront = null, leftBack= null, rightFront= null, rightBack= null;

    public void runOpMode() throws InterruptedException{

        leftFront = hardwareMap.get(DcMotorEx.class , "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class , "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class , "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class , "rightBack");


        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.a) {
                leftFront.setPower(.5);
                telemetry.addData("currently running" , "leftFront");
                telemetry.update();
            }
            else leftFront.setPower(0);
            if (gamepad1.b) {

                rightFront.setPower(.5);
                telemetry.addData("currently running" , "rightFront");
                telemetry.update();
            }
            else rightFront.setPower(0);
            if (gamepad1.x) {
                leftBack.setPower(.5);
                telemetry.addData("currently running" , "leftBack");
                telemetry.update();
            }
            else leftBack.setPower(0);
            if (gamepad1.y) {
                rightBack.setPower(.5);
                telemetry.addData("currently running" , "rightBack");
                telemetry.update();
            }
            else rightBack.setPower(0);
        }

    }

}
