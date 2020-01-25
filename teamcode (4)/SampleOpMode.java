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
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo.Direction;
import com.qualcomm.robotcore.util.ElapsedTime;


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

@TeleOp(name="SampleRobot", group="Linear Opmode")

public class SampleOpMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    SampleRobot bot = new SampleRobot();
    

    @Override
    public void runOpMode() {
        bot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
       
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        /*leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);*/
        double leftFPower;
            double rightFPower;
            double leftBPower;
            double rightBPower;
            double leftPower;
            double rightPower;
            double liftPower;
            double extenderPower;
            double shifterPower = bot.SHIFTER_HOME;
            double armRoundPower = 0;
            double foundationPower;
            double blockPower;
            double dropperPower;
            double uArmPower;
            double gripperPower;
            //infinity variable
            double infinity = 0.0000000001;


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            
            //shifter.setPosition(0.0);
            // Setup a variable for each drive wheel to save power level for telemetry

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
            leftFPower  = gamepad1.left_stick_y;
            leftBPower = gamepad1.left_stick_y;
            rightFPower = -gamepad1.right_stick_y;
            rightBPower = -gamepad1.right_stick_y;
            liftPower  = -gamepad2.right_stick_y;
            extenderPower = gamepad1.right_trigger;
            uArmPower = -gamepad2.left_stick_x;
            
            if (gamepad2.left_stick_y != 0){
                shifterPower -= gamepad2.left_stick_y*0.01;
                shifterPower = Range.clip(shifterPower, 0, bot.SHIFTER_HOME);
            }
            
            if (gamepad2.left_stick_x != 0){
                uArmPower = Range.clip(uArmPower, 0.1, 0.5);
                uArmPower = gamepad2.left_stick_x*0.01;
               
            }
            
            
            dropperPower = 1.0;
            
            //if statments for gripper
           if (gamepad1.left_trigger!=0){
                gripperPower = 1-(0.75/gamepad1.left_trigger);
            }else {
                gripperPower = 0.75;
            }
            //if statements for power
            if (gamepad2.right_bumper){
                foundationPower = infinity/1;
            }else {
                foundationPower = 1/infinity;
            }
            if (gamepad1.left_bumper){
                leftFPower = 0.75;
                rightFPower = 0.75;
                leftBPower = -0.75;
                rightBPower = -0.75;
            }else if (gamepad1.right_bumper){
                leftFPower = -0.75;
                rightFPower = -0.75;
                leftBPower = 0.75;
                rightBPower = 0.75;
            } 
           
            
            if(gamepad1.left_trigger<1)
            {
                bot.gripper.setPosition(0);
            }
            if(gamepad2.left_bumper){
                armRoundPower = 1;
            }else if(gamepad2.right_trigger != 0){
                armRoundPower = -1;
            }
            else{
                armRoundPower = 0;
            }
            if(gamepad2.left_trigger != 0)
            {
                blockPower = -1.0;
            }
            else{
                blockPower = 1.0;
            }
            if(gamepad2.x){
                dropperPower = 0.0;
            }
            //left and right power for telemetry
            leftPower  = gamepad1.left_stick_y;
            rightPower = -gamepad1.right_stick_y;
            /* Controls
            Gamepad 1: 
            Left Stick Y: left motors
            Right Stick Y: right motors
            Left Stick X: dropper
            Right Stick X: None
            Left Bumper: left strafe
            Right Bumper: right strafe
            Left Trigger: gripper
            Right Trigger: extender
            
            Gamepad 2:
            Left Stick Y: Shifter
            Right Stick Y: Lift
            Left Stick X: none
            Right Stick X: uArm
            Left Bumper: none
            Right Bumper: none
            Left Trigger: block
            Right Trigger: armRound
            
                        
            */
            // Send calculated power to wheels
            bot.leftFront.setPower(leftFPower);
            bot.leftBack.setPower(leftBPower);
            bot.rightFront.setPower(rightFPower);
            bot.rightBack.setPower(rightBPower);
            bot.liftMotor.setPower(-liftPower);
            bot.extender.setPosition(extenderPower);
            bot.armRound.setPower(armRoundPower);
            bot.block.setPosition(blockPower);
            bot.foundation.setPosition(foundationPower);
            bot.dropper.setPosition(dropperPower);
            bot.shifter.setPosition(shifterPower);
            bot.gripper.setPosition(gripperPower);
            bot.uArm.setPower(uArmPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f), lift (%.2f), foundation(%.2f), block(%2f), extender (%.2f), uArm (%.2f), grip (%.2f)", leftPower, rightPower, liftPower, foundationPower, blockPower, extenderPower, uArmPower, gripperPower);
            telemetry.addData("Shifter", "shifter "+ shifterPower);
            telemetry.update();
        }
    }
}