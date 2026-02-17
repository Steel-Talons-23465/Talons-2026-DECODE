package org.firstinspires.ftc.teamcode.General;

import  org.firstinspires.ftc.robotcore.external.JavaUtil;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.TouchSensor;

public class Robot {
    public static DcMotorEx fan;
    public static DcMotorEx rightLaunch;
    public static DcMotorEx leftLaunch;
    public static DcMotorEx intake;
//    public static CRServo intakeServo;
    public static Servo hammer;
    public static Servo liftRight, liftLeft;
    public static Servo intakeKicker;
    private static RevColorSensorV3 colorRight;
    private static RevColorSensorV3 colorLeft;
    private static TouchSensor touchSensor;
    private Limelight3A limelight;
    private LLResult result;
    private static LED slotZeroGreen;
    private static LED slotOneGreen;
    private static LED slotTwoGreen;
    private static LED slotZeroRed;
    private static LED slotOneRed;
    private static LED slotTwoRed;

    ColorSensed[] display = {ColorSensed.NO_COLOR, ColorSensed.NO_COLOR, ColorSensed.NO_COLOR};
    double launchTargetVelocity;
    int slotGoal;
    boolean launched;


     public void initialize(HardwareMap hwMp){
         touchSensor = hwMp.get(TouchSensor.class, "touchSensor");
        colorRight = hwMp.get(RevColorSensorV3.class, "colorRight");
        hammer = hwMp.get(Servo.class, "hammer");
        intakeKicker = hwMp.get(Servo.class, "kickerRight");
        rightLaunch = hwMp.get(DcMotorEx.class, "rightLaunch");
        leftLaunch = hwMp.get(DcMotorEx.class, "leftLaunch");
        colorLeft = hwMp.get(RevColorSensorV3.class, "colorLeft");

        liftLeft = hwMp.get(Servo.class , "liftLeft");
        liftRight = hwMp.get(Servo.class, "liftRight");

        intake = hwMp.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
//         intakeServo = hwMp.get(CRServo.class, "intakeServo");
         fan = hwMp.get(DcMotorEx.class, "fan");


        slotZeroGreen = hwMp.get(LED.class, "slotZeroGreen");
        slotOneGreen = hwMp.get(LED.class, "slotOneGreen");
        slotTwoGreen = hwMp.get(LED.class, "slotTwoGreen");
         slotZeroRed = hwMp.get(LED.class, "slotZeroRed");
         slotOneRed = hwMp.get(LED.class, "slotOneRed");
         slotTwoRed = hwMp.get(LED.class, "slotTwoRed");

         leftLaunch.setDirection(DcMotorSimple.Direction.REVERSE);
         leftLaunch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
         leftLaunch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         rightLaunch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
         rightLaunch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

         fan.setTargetPosition(fan.getCurrentPosition());
         fan.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         fan.setPower(1);
         fan.setTargetPositionTolerance(3);

         limelight = hwMp.get(Limelight3A.class, "limelight");
         LLResult result = limelight.getLatestResult();
    }

    public void setStoragePos(int slot, boolean intake) {
        int ticks = 1425;
        int absolutePos = fan.getCurrentPosition();
        int relativePos = absolutePos % ticks;
        int rotationOffset = absolutePos-relativePos;
        slotGoal = slot;
        if (intake) {
            if (slot == 0) {
                if(relativePos <= ticks/2)
                    fan.setTargetPosition(rotationOffset);
                else
                    fan.setTargetPosition(rotationOffset + ticks);
            } else if (slot == 1) {
                if(relativePos <= 5*ticks/6)
                    fan.setTargetPosition(rotationOffset + ticks/3);
                else
                    fan.setTargetPosition(rotationOffset + 4*ticks/3);
            } else if (slot == 2) {
                if(relativePos >= ticks/6)
                    fan.setTargetPosition(rotationOffset + 2*ticks/3);
                else
                    fan.setTargetPosition(rotationOffset - ticks/3);

            }
        }
        else {
            if (slot == 0) {
                fan.setTargetPosition(rotationOffset + ticks/2);
            } else if (slot == 1) {
                if(relativePos >= ticks/3)
                    fan.setTargetPosition(rotationOffset + 5*ticks/6);
                else
                    fan.setTargetPosition(rotationOffset - ticks/6);
            } else if (slot == 2) {
                if(relativePos >= 2*ticks/3)
                    fan.setTargetPosition(rotationOffset + 7*ticks/6);
                else
                    fan.setTargetPosition(rotationOffset + ticks/6);
            }
        }
    }

    public void setStoragePosOneRotation(int slot, boolean intake){
         int ticks = 1425;
         slotGoal = slot;
         if(intake){
             if(slot == 0)
                 fan.setTargetPosition(0);
             if(slot == 1)
                 fan.setTargetPosition(ticks/3);
             if(slot == 2)
                 fan.setTargetPosition(2*ticks/3);
         }else{
             if(slot == 0)
                 fan.setTargetPosition(ticks/2);
             if(slot == 1)
                 fan.setTargetPosition(5*ticks/6);
             if(slot == 2)
                 fan.setTargetPosition(7*ticks/6);
         }
    }

    public void startIntake(boolean in, double pow) {intake.setPower(in ? pow : -1*pow);}
    public void stopIntake() {intake.setPower(0);}

    public void startLaunchMotors(boolean far) {
         launchTargetVelocity = far ? 2100: 1550;
         leftLaunch.setVelocity(launchTargetVelocity);
         rightLaunch.setVelocity(launchTargetVelocity);
    }
	
	public void startLaunchMotorsAlt(boolean far) {
		launchTargetVelocity = far ? 2150 : 1550;
		leftLaunch.setVelocity(launchTargetVelocity);
		rightLaunch.setVelocity(launchTargetVelocity);
	}

    public double getLaunchTargetVelocity(){
         return launchTargetVelocity;
    }

    public void stopLaunchMotors() {
         launchTargetVelocity = 0;
         leftLaunch.setVelocity(0);
         rightLaunch.setVelocity(0);
    }


    public boolean atTargetVelocity() {
        return leftLaunch.getVelocity() >= launchTargetVelocity - 10 && rightLaunch.getVelocity() >= launchTargetVelocity - 10;
    }

    public boolean atSortTarget() {return Math.abs(fan.getCurrentPosition() - fan.getTargetPosition()) <= 3;}

    public boolean atSortTargetLenient(){return Math.abs(fan.getCurrentPosition() - fan.getTargetPosition()) <= 100;}

    public int getSlotGoal() {return slotGoal;}

    public ColorSensed detectColor() {
         double saturationR = JavaUtil.rgbToSaturation(colorRight.red(), colorRight.green(), colorRight.blue());
         double hueR = JavaUtil.rgbToHue(colorRight.red(), colorRight.green(), colorRight.blue());

         ColorSensed  right=  (hueR > 160 && saturationR < .3) ? ColorSensed.PURPLE : ((hueR < 150 && saturationR > .65) ? ColorSensed.GREEN : ColorSensed.INCONCLUSIVE);

         double saturationL = JavaUtil.rgbToSaturation(colorLeft.red(), colorLeft.green(), colorLeft.blue());
         double hueL = JavaUtil.rgbToHue(colorLeft.red(), colorLeft.green(), colorLeft.blue());

          ColorSensed left =  (hueL > 200 && saturationL < .7 && saturationL < .6) ? ColorSensed.PURPLE : ((hueL < 180 && saturationL > .7) ? ColorSensed.GREEN : ColorSensed.INCONCLUSIVE);

          return right == left ? right : left == ColorSensed.INCONCLUSIVE ? right : left;
    }

    public String returnColor(){
        double saturationR = JavaUtil.rgbToSaturation(colorRight.red(), colorRight.green(), colorRight.blue());
        double hueR = JavaUtil.rgbToHue(colorRight.red(), colorRight.green(), colorRight.blue());
        double saturationL = JavaUtil.rgbToSaturation(colorLeft.red(), colorLeft.green(), colorLeft.blue());
        double hueL = JavaUtil.rgbToHue(colorLeft.red(), colorLeft.green(), colorLeft.blue());
        return "Right Saturation" + saturationR + "\nRight Hue" + hueR + "\nLeft Saturation" + saturationL + "\nLeft Hue"+ hueL;
    }

    public void launch() {
         hammer.setPosition(1);
        launched = true;
    }
    public void resetHammer() {
        hammer.setPosition(0);
    }
    public void resetLaunch() {launched = false;}
    public boolean hammerAtLaunch(){
         return hammer.getPosition() == 1;
    }

    public boolean isLaunched(){return launched;}

    public boolean buttonPressed() {return touchSensor.isPressed();}



    public void updateLED(ColorSensed[] order) {

         if(order[0] == ColorSensed.PURPLE || order[0] == ColorSensed.INCONCLUSIVE)
            slotZeroGreen.on();
         else slotZeroGreen.off();
        if(order[0] == ColorSensed.GREEN || order[0] == ColorSensed.INCONCLUSIVE)
            slotZeroRed.on();
        else slotZeroRed.off();
        if(order[1] == ColorSensed.PURPLE || order[1] == ColorSensed.INCONCLUSIVE)
            slotOneGreen.on();
        else slotOneGreen.off();
        if(order[1] == ColorSensed.GREEN || order[1] == ColorSensed.INCONCLUSIVE)
            slotOneRed.on();
        else slotOneRed.off();
        if(order[2] == ColorSensed.PURPLE || order[2] == ColorSensed.INCONCLUSIVE)
            slotTwoGreen.on();
        else slotTwoGreen.off();
        if(order[2] == ColorSensed.GREEN || order[2] == ColorSensed.INCONCLUSIVE)
            slotTwoRed.on();
        else slotTwoRed.off();

    }
    public void updateLED(int inMotif){
         int green = 0;
         int purple = 0;
         int inconclusive = 0;
         for(int j = 0; j < SharedData.storage.length; j++){
             if(SharedData.storage[j] == ColorSensed.GREEN)
                 green++;
             else if (SharedData.storage[j] == ColorSensed.PURPLE)
                 purple++;
             else if (SharedData.storage[j] == ColorSensed.INCONCLUSIVE)
                 inconclusive++;
         }
         for(int j = 0; j < display.length; j++){
             if(inMotif == SharedData.greenIndex){
                 if(green > 0) {
                     display[j] = ColorSensed.GREEN;
                     green--;
                 }
                 else if(inconclusive > 0){
                     display[j] = ColorSensed.INCONCLUSIVE;
                     inconclusive--;
                 }
                 else if(purple > 0){
                     display[j] = ColorSensed.PURPLE;
                     purple--;
                 }
                 else{
                     display[j] = ColorSensed.NO_COLOR;
                 }
             }
             else{
                 if(purple > 0) {
                     display[j] = ColorSensed.PURPLE;
                     purple--;
                 }
                 else if(inconclusive > 0){
                     display[j] = ColorSensed.INCONCLUSIVE;
                     inconclusive--;
                 }
                 else if(green > 0){
                     display[j] = ColorSensed.GREEN;
                     green--;
                 }
                 else{
                     display[j] = ColorSensed.NO_COLOR;
                 }
             }
             inMotif = inMotif == 2 ? 0 : inMotif + 1;
         }
         updateLED(display);
    }

    public void disableLED()
    {
        slotZeroGreen.off();
        slotZeroRed.off();
        slotOneGreen.off();
        slotOneRed.off();
        slotTwoGreen.off();
        slotTwoRed.off();

    }

    public void kickIntake(){
         intakeKicker.setPosition(1);
    }
    public void returnIntakeKicker()
    {
        intakeKicker.setPosition(.5);
    }

    public void liftServos(boolean lifting){
         liftRight.setPosition(lifting ? 0 : .5);
         liftLeft.setPosition(lifting ? 1 : .5);
    }

// PIDFS FOR LATER
//    public void startLaunchMotorsPIDF(boolean far){
//         launchTargetVelocity = rpmToTicksPerSecond(far ? 1025 : 600);
//         rightLaunch.setVelocityPIDFCoefficients(20,0,2, 12);
//         leftLaunch.setVelocityPIDFCoefficients(20,0,2,12);
//
//         rightLaunch.setVelocity(launchTargetVelocity);
//         leftLaunch.setVelocity(launchTargetVelocity);
//    }
//
//    private double rpmToTicksPerSecond(double rpm) {
//        return ( rpm / 60.0) * 112.0;
//    }


//    public void getTargetArea(){
//         if (result.isValid()){
//
//         }
//    }
//
//    public void getResult(){
//
//        if (this.result.isValid() && result != null){
//
//        }
//    }







}
