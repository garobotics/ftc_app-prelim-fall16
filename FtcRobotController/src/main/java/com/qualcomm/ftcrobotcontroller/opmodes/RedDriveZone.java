/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Uses time-based
 */
public class RedDriveZone extends LinearOpMode {

    DcMotor motorRightRear;
    DcMotor motorRightFront;
    DcMotor motorLeftRear;
    DcMotor motorLeftFront;
    DcMotor motorSlideTilt;
    DcMotor linearSlide;


    @Override
    public void runOpMode() throws InterruptedException {

//        setting names to motors
        motorRightFront = hardwareMap.dcMotor.get("rf");
        motorRightRear = hardwareMap.dcMotor.get("rb");

        motorLeftFront = hardwareMap.dcMotor.get("lf");
        motorLeftRear = hardwareMap.dcMotor.get("lb");

        motorSlideTilt = hardwareMap.dcMotor.get("tiltMotor");
        linearSlide = hardwareMap.dcMotor.get("expandMotor");

        motorLeftRear.setDirection(DcMotor.Direction.FORWARD);
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightRear.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

//      commands using methods of moving the robot


        driveBackwardForSecs(0.5);
        turnLeft(55);
        driveBackwardForSecs(2);
        stopDriving();


//
//        turnRight(90);
//        turnLeft(90);
//
//        extendArm(0.5);
//        retractArm(0.5);
//
//        raiseArm(1);
//        lowerArm(1);
    }

    public void stopDriving() throws InterruptedException {
        motorLeftFront.setPower(0);
        motorLeftRear.setPower(0);
        motorRightFront.setPower(0);
        motorRightRear.setPower(0);
    }
    public void driveForwardForSecs(double secs) throws InterruptedException {
        motorLeftFront.setPower(1);
        motorLeftRear.setPower(1);
        motorRightFront.setPower(1);
        motorRightRear.setPower(1);
        long ms = (long) (secs*1000);
        sleep(ms);
    }

    public void driveBackwardForSecs(double secs) throws InterruptedException {
        motorLeftFront.setPower(-1);
        motorLeftRear.setPower(-1);
        motorRightFront.setPower(-1);
        motorRightRear.setPower(-1);
        long ms = (long) (secs*1000);
        sleep(ms);
    }

    double fullRT = 2750;
    double oneDegTime = fullRT / 360.0;

    public void turnRight(double degree) throws InterruptedException {
        // make function that changes degrees into motor power for turning right


        motorLeftFront.setPower(1);
        motorLeftRear.setPower(1);
        motorRightFront.setPower(-1);
        motorRightRear.setPower(-1);
        long sleepTime = (long) (oneDegTime*degree);
        sleep(sleepTime);

    }

    public void turnLeft(double degree) throws InterruptedException {
        // make function that changes degrees into motor power for turning left

        motorLeftFront.setPower(-1);
        motorLeftRear.setPower(-1);
        motorRightFront.setPower(1);
        motorRightRear.setPower(1);
        long sleepTime = (long) (oneDegTime * degree);
        sleep(sleepTime);


    }

        double fullExtendTime = 2.0;


    public void extendArm(double position) {

        ElapsedTime timerEA = new ElapsedTime();
        timerEA.reset();

        while (timerEA.time() <= (fullExtendTime * position)) {
            linearSlide.setPower(1);
        }

//        double rotation = linearSlide.getCurrentPosition();
//
//        while (rotation <= position) {
//            linearSlide.setPower(0.5);
//        }

    }

    public void retractArm(double position) {
        ///use encoders to bring arm back to starting position

        ElapsedTime timerRA = new ElapsedTime();
        timerRA.reset();

        while (timerRA.time() <= (fullExtendTime * position)) {
            linearSlide.setPower(-1);
        }
    }

    double fullRaiseTime = 2.0;

    public void raiseArm(double position) {
        ElapsedTime timerRA = new ElapsedTime();
        timerRA.reset();

        while (timerRA.time() <= (fullRaiseTime * position)) {
            motorSlideTilt.setPower(1);
        }
    }

    public void lowerArm(double position) {
        ElapsedTime timerLA = new ElapsedTime();
        timerLA.reset();

        while (timerLA.time() <= (fullRaiseTime * position)) {
            motorSlideTilt.setPower(-1);
        }
    }
}
//    public void sensorInput (double /* sensorInput) {
//        // code goes here for sensor responsible for turning
//
//
