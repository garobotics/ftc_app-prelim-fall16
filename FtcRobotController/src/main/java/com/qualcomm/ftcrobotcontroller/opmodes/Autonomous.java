package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.text.SimpleDateFormat;
import java.util.Date;

public class Autonomous extends LinearOpMode {

// one revolution of wheels brings robot forward 20.5 inches
// convert wheel revolutions to inches
    private final double inchesPerRevolution = 12.5;
    private final int numClicks = 1120; //# of clicks per rotation
    private final int wheelRotations = 3; //# of wheel rotations per 360-deg spin of robot
    private final int fullRCTurns = numClicks * wheelRotations; // # of clicks it takes to spin in a circle
    private final int oneDegClicks = fullRCTurns / 360; // # of clicks it takes to turn 1 degree

    DcMotor motorRightRear;
    DcMotor motorRightFront;
    DcMotor motorLeftRear;
    DcMotor motorLeftFront;

    public Autonomous() {
    }

    @Override public void runOpMode() throws InterruptedException {

        // why do rear motors not work even though they are commented in

        motorRightFront = hardwareMap.dcMotor.get("rf");
        motorRightRear = hardwareMap.dcMotor.get("rb");
        motorLeftFront = hardwareMap.dcMotor.get("lf");
        motorLeftRear = hardwareMap.dcMotor.get("lb");

        motorLeftRear.setDirection(DcMotor.Direction.FORWARD);
        motorLeftFront.setDirection(DcMotor.Direction.FORWARD);
        motorRightFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightRear.setDirection(DcMotor.Direction.REVERSE);

        setDriveMode(DcMotorController.RunMode.RESET_ENCODERS);

        waitForStart();
        sleep(15000);
        setDriveMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        driveForwardForInches(105);

        //stop();
        /*driveBackwardsForInches(90);
        turnRightForDeg(90);
        turnLeftForDeg(90);*/
    }

// drive forward method
    public void driveForwardForInches(float inches) throws InterruptedException {

// convert inches to rotations to clicks
        double rotations = inches / inchesPerRevolution;
        double clicks = 1120 * rotations;
        int target = (int)clicks;

        int curPosLF = motorLeftFront.getCurrentPosition();
//        int curPosLR = motorLeftRear.getCurrentPosition();
        int curPosRF = motorRightFront.getCurrentPosition();
//        int curPosRR = motorRightRear.getCurrentPosition();

        int targetLF = target + curPosLF;
//        int targetLR = target + curPosLR;
        int targetRF = target + curPosRF;
//        int targetRR = target + curPosRR;

        setDrivePower(1, 1);

        while (motorLeftFront.getCurrentPosition() < targetLF
                && motorRightFront.getCurrentPosition() < targetRF
//                 && motorRightRear.getCurrentPosition() < targetRR
//                && motorLeftRear.getCurrentPosition() < targetLR
                ) {
// see if need to change less than sign above for driving backwards

            waitOneFullHardwareCycle();
        }

        setDrivePower(0, 0);
    }
/*
// drive backwards method
    public void driveBackwardsForInches(float inches) throws InterruptedException {

// convert inches to rotations to clicks
        double rotations = inches / inchesPerRevolution;
        double clicks = 1120 * rotations;
        int target = (int)(-clicks);
// clicks is negative because this function drives backwards

        int curPosLF = motorLeftFront.getCurrentPosition();
      //  int curPosLR = motorLeftRear.getCurrentPosition();
        int curPosRF = motorRightFront.getCurrentPosition();
      //  int curPosRR = motorRightRear.getCurrentPosition();

        int targetLF = target + curPosLF;
       // int targetLR = target + curPosLR;
        int targetRF = target + curPosRF;
       // int targetRR = target + curPosRR;

        setDrivePower(1, 1);
// might have to change (1,1) to (-1,-1) --> check how negative encoder values work

        while (motorLeftFront.getCurrentPosition() > targetLF
                && motorRightFront.getCurrentPosition() > targetRF) {
// see if need to change less than sign above for driving backwards

            waitOneFullHardwareCycle();
        }

        setDrivePower(0, 0);

    }

// turn right method
    public void turnRightForDeg (float degree) throws InterruptedException {


// convert clicks to degrees
        double clicks = oneDegClicks * degree;
        int target = (int) clicks;

        int curPosLF = motorLeftFront.getCurrentPosition();
        //int curPosLR = motorLeftRear.getCurrentPosition();
        int curPosRF = motorRightFront.getCurrentPosition();
       // int curPosRR = motorRightRear.getCurrentPosition();

        int targetLF = target + curPosLF;
       // int targetLR = target + curPosLR;
        int targetRF = -target + curPosRF;
       // int targetRR = -target + curPosRR;

        setDrivePower(1,1);
// might have to change (1,1) to (1,-1) --> check how negative encoder values work


        while (motorLeftFront.getCurrentPosition() < targetLF
                && motorRightFront.getCurrentPosition() > targetRF) {
// see if need to change less than sign above for driving backwards

            waitOneFullHardwareCycle();
        }

        setDrivePower(0, 0);

    }

// turn left method
    public void turnLeftForDeg (float degree) throws InterruptedException {


// convert clicks to degrees
        double clicks = oneDegClicks * degree;
        int target = (int) clicks;

        int curPosLF = motorLeftFront.getCurrentPosition();
      //  int curPosLR = motorLeftRear.getCurrentPosition();
        int curPosRF = motorRightFront.getCurrentPosition();
      //  int curPosRR = motorRightRear.getCurrentPosition();

        int targetLF = -target + curPosLF;
     //   int targetLR = -target + curPosLR;
        int targetRF = target + curPosRF;
     //   int targetRR = target + curPosRR;

// might have to change (1,1) to (-1,1) --> check how negative encoder values work
        setDrivePower(1,1);

        while (motorLeftFront.getCurrentPosition() > targetLF
                && motorRightFront.getCurrentPosition() < targetRF) {

// see if need to change less than sign above for driving backwards

            waitOneFullHardwareCycle();
        }

        setDrivePower(0, 0);

    }
*/

    public void setDrivePower(double left, double right) {
// This assumes power is given as -1 to 1
// The range clip will make sure it is between -1 and 1
// An incorrect value can cause the program to exception
        right = right/2.0;
        left = left/2.0;
        motorLeftRear.setPower(Range.clip(left, -1.0, 1.0));
        motorLeftFront.setPower(Range.clip(left, -1.0, 1.0));
        motorRightFront.setPower(Range.clip(right, -1.0, 1.0));
        motorRightRear.setPower(Range.clip(right, -1.0, 1.0));
    }


    public void setDriveMode(DcMotorController.RunMode mode) {
//sets motors to the correct mode if they're not already on it

     //   if (motorLeftRear.getChannelMode() != mode) {
     //       motorLeftRear.setChannelMode(mode);
     //   }

        if (motorLeftFront.getMode() != mode) {
            motorLeftFront.setMode(mode);
        }

        if (motorRightFront.getMode() != mode) {
            motorRightFront.setMode(mode);
        }

     //   if (motorRightRear.getChannelMode() != mode) {
     //       motorRightRear.setChannelMode(mode);
      //  }
    }

}