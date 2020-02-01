/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a K9 robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Servo channel:  Servo to raise/lower arm: "arm"
 * Servo channel:  Servo to open/close claw: "claw"
 *
 * Note: the configuration of the servos is such that:
 *   As the arm servo approaches 0, the arm position moves up (away from the floor).
 *   As the claw servo approaches 0, the claw opens up (drops the game element).
 */
public class SampleRobot
{
    /* Public OpMode members. */
    //Create Motors

    DcMotor leftFront = null;
    DcMotor leftBack = null;
    DcMotor rightFront = null;
    DcMotor rightBack = null;
    DcMotor liftMotor = null;
    Servo extender = null;
    Servo shifter = null;
    CRServo armRound = null;
    Servo gripper = null;
    Servo dropper = null;
    Servo block = null;
    Servo foundation = null;
    Servo armDown = null;
    //CRServo uArm = null;
    public static final double SHIFTER_HOME = 0.7;    
    HardwareMap hwMap = null;
    double infinity = 1000000000;
    /* Local OpMode members. */
    /*HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public SampleRobot() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFront  = hwMap.get(DcMotor.class , "leftFront");
        leftBack   = hwMap.get(DcMotor.class , "leftBack");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        rightBack  = hwMap.get(DcMotor.class, "rightBack");
        liftMotor  = hwMap.get(DcMotor.class, "liftMotor");
        extender = hwMap.get(Servo.class, "extender");
        shifter = hwMap.get(Servo.class, "shifter");
        armRound = hwMap.get(CRServo.class, "armRound");
        gripper = hwMap.get(Servo.class, "gripper");
        dropper = hwMap.get(Servo.class, "dropper");
        block = hwMap.get(Servo.class, "block");
        foundation = hwMap.get(Servo.class, "foundation");
        armDown = hwMap.get(Servo.class, "armDown");
        //uArm = hwMap.get(CRServo.class, "uArm");

        // Set all motors to zero power
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        liftMotor.setPower(0);
        extender.setPosition(0.0);
        shifter.setPosition(SHIFTER_HOME);
        armRound.setPower(0.0);
        gripper.setPosition(0.0);
        dropper.setPosition(1.0);
        block.setPosition(1.0);
        foundation.setPosition(1.0);
        armDown.setPosition(0.0);
        //uArm.setPower(0.0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        // Define and initialize ALL installed servos.

    }  
}
