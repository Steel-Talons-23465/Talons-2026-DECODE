package org.firstinspires.ftc.teamcode.midas.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp
        (name = "TURRET TRACKING" , group = "TURRET TUNERS")
public class TurretTurningManualTest extends LinearOpMode {

    DcMotorEx turret = null;
int pow;
    @Override
    public void runOpMode() throws InterruptedException {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.dpad_up){
                pow+=.05;
            }
            else if (gamepad1.dpad_down){
                pow -= .05;
            }

            if (gamepad1.a){
                turret.setPower(pow);
                telemetry.addData("running ", "positive turret");
                telemetry.update();
            }
            else if (gamepad1.b){
                turret.setPower(-pow);
                telemetry.addData("running ", "negative turret");
                telemetry.update();
            }
            else turret.setPower(0);
            telemetry.addData("running ", "no turret");
            telemetry.update();



        }
    }
}
