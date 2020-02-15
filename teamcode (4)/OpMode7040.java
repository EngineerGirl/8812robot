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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.Map;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="OpMode7040", group="Linear Opmode")

public class OpMode7040 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftBack  = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor liftMotor = null;
    private Servo block = null;
    private Servo foundation = null;
    private Servo grip = null;
    private Servo slide = null;
    private Servo dropper = null;
    //Robot7040 bot = new Robot7040();
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");
        liftMotor  = hardwareMap.get(DcMotor.class, "liftMotor");
        block = hardwareMap.get(Servo.class, "block");
        foundation = hardwareMap.get(Servo.class, "foundation");
        grip = hardwareMap.get(Servo.class, "grip");
        slide = hardwareMap.get(Servo.class, "slide");
        dropper = hardwareMap.get(Servo.class, "dropper");
        
        
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        /*leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);*/
        
        

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double fLeftPower;
            double bLeftPower = 0;
            double fRightPower;
            double bRightPower = 0;
            double liftPower = 0;
            double blockPower;
            double foundationPower;
            double slidePower;
            double gripPower = 0;
            double dropperPower = 0;
            double drop = 0.000001;
            if (gamepad2.right_bumper){
                dropperPower = drop * 1;
            }
            else {
                dropperPower = 1/drop;
            }
            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            //double drive = -gamepad1.left_stick_y;
            //double turn  =  gamepad1.right_stick_x;
            //leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            //rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            fLeftPower  = (gamepad1.left_stick_y);
            fRightPower = -gamepad1.right_stick_y;
            bLeftPower = (gamepad1.left_stick_y);
            bRightPower = (-gamepad1.right_stick_y);
            liftPower  = -gamepad2.right_stick_y;
            foundationPower = 0.1/(gamepad2.right_trigger);
            blockPower = 0.1/(gamepad2.left_trigger);
            slidePower = -gamepad2.left_stick_y;
            //gripPower = gamepad2.left_bumper;
            /*dropperPower = 1/drop;
           // if (gamepad2.right_bumper){
                //dropperPower = 1;
            */
            /* Controls for Gamepad 2
            block: left trigger
            foundation: right trigger
            lift: right joystick
            slide/linear grip: left joystick
            grip mechanism: left bumper
            dropper: right bumper
            */ 
            
            if (gamepad1.right_bumper){
                leftFront.setPower(-0.75);
                rightBack.setPower(0.75);
                rightFront.setPower(-1);
                leftBack.setPower(1);
                
                /*
                fLeftPower = -gamepad1.left_stick_x;
                bRightPower = gamepad1.right_stick_x;
                bLeftPower = gamepad1.left_stick_x;
                fRightPower = -gamepad1.right_stick_x;*/
            }
             else if (gamepad1.left_bumper){
                leftFront.setPower(0.75);
                rightBack.setPower(-0.75);
                rightFront.setPower(1);
                leftBack.setPower(-1);
                /*fLeftPower = gamepad1.left_stick_x;
                bRightPower = -gamepad1.right_stick_x;
                bLeftPower = -gamepad1.left_stick_x;
                fRightPower = gamepad1.right_stick_x;*/
            }
            if (gamepad2.left_bumper){
                gripPower = drop*1;
            }
            else{
                gripPower = 1/drop;
            }
            // Send calculated power to wheels
            leftFront.setPower(fLeftPower);
            leftBack.setPower(bLeftPower);
            rightFront.setPower(fRightPower);
            rightBack.setPower(bRightPower);
            if (gamepad2.right_stick_y<0){
                liftMotor.setPower(liftPower*2);
            }
            else if (gamepad2.right_stick_y>0){
                liftMotor.setPower(liftPower*0.5);
            }
            else{
                liftMotor.setPower(liftPower);
            }
            foundation.setPosition(foundationPower);
            block.setPosition(blockPower);
            slide.setPosition(slidePower);
            grip.setPosition(gripPower);
            dropper.setPosition(dropperPower);
            
        
            
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "leftFront (%.2f), rightFront (%.2f), leftBack (%.2f), rightBack (%.2f), lift (%.2f), block (%.2f), foundation (%.2f), dropper (%.2f)", fLeftPower, fRightPower, bLeftPower, bRightPower, liftPower, blockPower, foundationPower, dropperPower);
            telemetry.update();
        }
    }
}
