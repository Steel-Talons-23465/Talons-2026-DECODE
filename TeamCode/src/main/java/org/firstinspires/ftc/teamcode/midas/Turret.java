package org.firstinspires.ftc.teamcode.midas;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Turret extends LinearOpMode {
DcMotorEx launch = null;
DcMotorEx turret = null;
Servo angleLeft = null;
Servo angleRight = null;
Limelight3A limelight = null;

int targetVelocity;
boolean launchActive;
boolean a;
boolean up;
boolean down;
boolean right;
boolean left;

    @Override
    public void runOpMode() throws InterruptedException {
        launch = hardwareMap.get(DcMotorEx.class, "launch");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        angleLeft = hardwareMap.get(Servo.class, "angleLeft");
        angleRight = hardwareMap.get(Servo.class, "angleRight");

        launch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launch.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setPower(.05);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while(opModeIsActive()){

            if(launchActive)
                startLaunch(targetVelocity);
            else stopLaunch();

            if(gamepad1.a && !a)
                launchActive = !launchActive;
            a = gamepad1.a;

            if(gamepad1.dpad_up && !up)
                targetVelocity += 50;
            else if(gamepad1.dpad_down && !down)
                targetVelocity -= 50;
            else if(gamepad1.dpad_right && !right)
                turret.setTargetPosition(turret.getTargetPosition()+30);
            else if(gamepad1.dpad_left && !left)
                turret.setTargetPosition(turret.getTargetPosition()-30);
            up = gamepad1.dpad_up;
            down = gamepad1.dpad_down;
            right = gamepad1.dpad_right;
            left = gamepad1.dpad_left;

            telemetry.addData("Launching", launchActive);
            telemetry.addData("TargetVelocity", targetVelocity);
            telemetry.addData("Velocity", launch.getVelocity());
            telemetry.addData("Turret Pos", turret.getTargetPosition());
            telemetry.update();

        }
    }

    public void startLaunch(int velocity){
        launch.setVelocity(velocity);

    }
    public void stopLaunch(){
        launch.setVelocity(0);
    }
}
