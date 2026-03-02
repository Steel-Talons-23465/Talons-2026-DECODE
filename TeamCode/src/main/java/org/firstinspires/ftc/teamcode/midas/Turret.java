package org.firstinspires.ftc.teamcode.midas;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp (name = "Turret" , group = "Turret")
public class Turret extends LinearOpMode {
DcMotorEx launch = null;
DcMotorEx turret = null;
Servo angleLeft = null;
Servo angleRight = null;
Limelight3A limelight = null;

int targetVelocity , targetPos;
double angle = .25;
boolean launchActive;
boolean a;
boolean b;
boolean y;
boolean up;
boolean down;
boolean right;
boolean left;

boolean trackingEnabled;
LLResult result;
double tX, tY;
double error, setpoint, currentPosition, totalError, deltaTime, lastError ,deltaError, output   ;
double kP, kI,kD,kF;


    @Override
    public void runOpMode() throws InterruptedException {
        launch = hardwareMap.get(DcMotorEx.class, "launch");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        angleLeft = hardwareMap.get(Servo.class, "angleLeft");
        angleRight = hardwareMap.get(Servo.class, "angleRight");
        limelight = hardwareMap.get(Limelight3A.class , "limelight");
        launch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launch.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setPower(.75);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        limelight.start();
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
                turret.setTargetPosition(turret.getTargetPosition()-30);
            else if(gamepad1.dpad_left && !left)
                turret.setTargetPosition(turret.getTargetPosition()+30);
            up = gamepad1.dpad_up;
            down = gamepad1.dpad_down;
            right = gamepad1.dpad_right;
            left = gamepad1.dpad_left;


            if(gamepad1.y && !y) {
                angle += .05;
                if(angle > .65)
                    angle = .65;
            }
            else if(gamepad1.b && !b) {
                angle -= .05;
                if (angle < .25)
                    angle = .25;
            }
            y = gamepad1.y;
            b = gamepad1.b;
            setAngle(angle);


            senseTag();

            trackingEnabled = gamepad1.right_bumper;



//            error = setpoint - currentPosition;
//            totalError += error * deltaTime; // Integral term
//            deltaError = (error - lastError) / deltaTime; // Derivative term
//
//            output = (kP * error) + (kI * totalError) + (kD * deltaError) + kF;
//
//            lastError = error;


            if (Math.abs(tX) > 1 && trackingEnabled){
                targetPos = turret.getCurrentPosition() - (int)(5.771*tX);
                turret.setTargetPosition(targetPos);
            }
            else {
                targetPos = turret.getTargetPosition();
                turret.setTargetPosition(targetPos);
            }


            telemetry.addData("Launching", launchActive);
            telemetry.addData("TargetVelocity", targetVelocity);
            telemetry.addData("Velocity", launch.getVelocity());
            telemetry.addData("Turret Pos", turret.getTargetPosition());
            telemetry.addData("angle", angle);
            telemetry.update();

            telemetry.update();

        }
    }

    public void startLaunch(int velocity){
        launch.setVelocity(velocity);

    }
    public void stopLaunch(){
        launch.setVelocity(0);
    }

    public void senseTag(){
        result = limelight.getLatestResult();

        try {
            tX = result.getFiducialResults().get(0).getTargetXDegrees();
            tY = result.getFiducialResults().get(0).getTargetYDegrees();
            telemetry.addData("Reading Apriltag" , result.getFiducialResults().get(0).getFiducialId() );
            telemetry.addData("TagX" , tX);
            telemetry.addData("TagY" , tY);

        }
        catch (Exception e){
            tX = 0;
            tY = 0;
            telemetry.addData("No AprilTag Detected" , "-1");
        }

    }

    public void setAngle(double position){
        angleLeft.setPosition(position); //0 -.6
        angleRight.setPosition(1-position); //.25-1
    }
}