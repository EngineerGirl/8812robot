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
import com.qualcomm.robotcore.robot.Robot;
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
@Autonomous(name="Foundation BLUE", group="Linear Opmode")

public class FoundationBlue7040 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftBack  = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private Servo foundation = null;
    private Servo extender = null;
    private Servo grip = null;
    //SampleRobot bot = new SampleRobot();
    
    //Method to move the robot for a specified duration
    public void moveRobot(double leftF, double rightF, double leftB, double rightB, long duration){
        leftFront.setPower(-leftF);
        rightFront.setPower(rightF);
        leftBack.setPower(-leftB);
        rightBack.setPower(rightB);
        grip.setPosition(100000);
        sleep(duration);
    }
    
    //Method to stop robot
    public void stopRobot(long duration){
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        grip.setPosition(100000);
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
        foundation = hardwareMap.get(Servo.class, "foundation");
        grip = hardwareMap.get(Servo.class, "grip");
        //extender = hardwareMap.get(Servo.class, "extender");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            
            foundation.setPosition(1/0.000000000001);
            //***this autonomous is the RED foundation autonomous***
            // Send calculated power to wheels
            
            //strafe LEFT to grab foundation in center
            moveRobot(-0.5, 0.5, 0.5, -0.5, 1200);  
            stopRobot(1000);
            
            //strafe right to end up in middle
            //moveRobot(0.5, -0.5, -0.5, 0.5, 1200);  
            //stopRobot(1000);
            
            
            //forward
            moveRobot(0.52, 0.58, 0.52, 0.58, 2250);
            stopRobot(1000);
            
            
            //grab and move back, first grab
            foundation.setPosition(0.1);
            stopRobot(1000);
            
   
            /*//strafe LEFT so the foundation doesn't hit the wall
            moveRobot(-0.5, 0.5, 0.5, -0.5, 1200);  
            stopRobot(1000);*/
            
            //move back
            foundation.setPosition(0.1);
            moveRobot(-0.65, -0.65, -0.65, -0.65, 1950);
            
            //turn to the RIGHT
            foundation.setPosition(0.1);
            moveRobot(-0.3, 0.3, -0.3, 0.3, 1500);

            //strafe right
            moveRobot(1.0, -1.0, -1.0, 1.0, 500);
            
            //tur IllegalThreadStateException
            moveRobot(-0.3, 0.3, -0.3, 0.3, 3000);
            /*//move forward to position the foundation in the corner
            moveRobot(0.52, 0.58, 0.52, 0.58, 1000);
            stopRobot(1000);        */   
            
            //let go of foundation
            foundation.setPosition(10000000);
            //extender.setPosition(0);
            
            //turn to the RIGHT
            //foundation.setPosition(0.1);
            moveRobot(-0.4, 0.4, -0.4, 0.4, 500);
            
            //strafe RIGHT so the foundation doesn't hit the wall
            moveRobot(1.0, -1.0, -1.0, 1.0, 2500);  
            stopRobot(19200);
            
           /* //park
            moveRobot(-0.5, -0.5, -0.5, -0.5, 2500);
            stopRobot(19200);
            //change to 17000 if uncommenting strafe right code*/

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}
