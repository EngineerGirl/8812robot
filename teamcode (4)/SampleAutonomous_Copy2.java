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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
@Autonomous(name="test auto", group="Linear Opmode")

public class SampleAutonomous_Copy2 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
     private DcMotor leftFront = null;
     private DcMotor leftBack  = null;
     private DcMotor rightFront = null;
     private DcMotor rightBack = null;
     private Servo block = null;
    //SampleRobot bot = new SampleRobot();
     public void moveRobot(double leftF, double rightF, double leftB, double rightB, long duration){
         leftFront.setPower(-leftF);
         rightFront.setPower(rightF);
         leftBack.setPower(-leftB);
         rightBack.setPower(rightB);
         sleep(duration);
     }
    
    //Method to stop robot
     public void stopRobot(long duration){
         leftFront.setPower(0);
         rightFront.setPower(0);
        leftBack.setPower(0);
         rightBack.setPower(0);
         sleep(duration);        
 }
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        block = hardwareMap.get(Servo.class, "block");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

         // Setup a variable for each drive wheel to save power level for telemetry
            // double leftPower;
            // double rightPower;

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
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            //***this autonomous is the BLUE foundation autonomous***
            // Send calculated power to wheels
            //forward
            // foundation.setPosition(1/0.000000000001);
            // //***this autonomous is the RED foundation autonomous***
            // // Send calculated power to wheels
            // //forward
            
            // moveRobot(0.52, 0.58, 0.52, 0.58, 2250);
            // stopRobot(1000);
            
            // //grab and move back, first grab
            // foundation.setPosition(0.1);
            // stopRobot(1000);
   
            // //strafe left so the foundation doesn't hit the wall
            // moveRobot(0.5, -0.5, -0.5, 0.5, 1500);  
            // stopRobot(1000);
            
            // //move back
            // foundation.setPosition(0.1);
            // moveRobot(-0.6, -0.6, -0.6, -0.6, 2100);
            
            // //turn to the RIGHT
            // foundation.setPosition(0.1);
            // moveRobot(0.6, -0.6, 0.6, -0.6, 1500);

            // //move forward to position the foundation in the corner
            // moveRobot(0.52, 0.58, 0.52, 0.58, 1000);
            // stopRobot(1000);           
            
            // //let go of foundation
            // foundation.setPosition(0.1/gamepad1.left_trigger);
            // //extender.setPosition(0);
            
            // //park
            // moveRobot(-0.5, -0.5, -0.5, -0.5, 2000);
            // stopRobot(20000);
            
            moveRobot(-0.8, -0.8, -0.8, -0.8, 1000);
            telemetry.addData("1000 ms over", "Run Time: " + runtime.toString());
            sleep(29000);
            
            
            
            // /*leftFront.setPower(0.52);
            // rightFront.setPower(-0.58);
            // leftBack.setPower(0.52);
            // rightBack.setPower(-0.58);
            // sleep(2250);
            // leftFront.setPower(0);
            // rightFront.setPower(0);
            // leftBack.setPower(0);
            // rightBack.setPower(0);
            // sleep(1000);
            // //grab and move back
            // foundation.setPosition(0.1);
            // leftFront.setPower(0);
            // rightFront.setPower(0);
            // leftBack.setPower(0);
            // rightBack.setPower(0);
            // sleep(1000);
            // foundation.setPosition(0.1);
            // leftFront.setPower(-0.6);
            // rightFront.setPower(0.6);
            // leftBack.setPower(-0.6);
            // rightBack.setPower(0.6);
            // sleep(2100);
            // //turn to the left
            // foundation.setPosition(0.1);
            // leftFront.setPower(0.6);
            // rightFront.setPower(0.6);
            // leftBack.setPower(0.6);
            // rightBack.setPower(0.6);
            // sleep(1500);
            // foundation.setPosition(0.1/gamepad1.left_trigger);
            // leftFront.setPower(-1);
            // rightFront.setPower(1);
            // leftBack.setPower(-1);
            // rightBack.setPower(1);
            // sleep(1000);
            // leftFront.setPower(0);
            // rightFront.setPower(0);
            // leftBack.setPower(0);
            // rightBack.setPower(0);
            // sleep(1000);
            // foundation.setPosition(0.1);
            // leftFront.setPower(0.5);
            // rightFront.setPower(0.5);
            // leftBack.setPower(0.5);
            // rightBack.setPower(0.5);
            // sleep(3000);
            // leftFront.setPower(0);
            // rightFront.setPower(0);
            // leftBack.setPower(0);
            // rightBack.setPower(0);
            // sleep(17150);*/
            
            block.setPosition(1.0);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}
