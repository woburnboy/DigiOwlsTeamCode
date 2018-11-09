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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Digital Owls TeleOp Mode", group="DigiOwls")
//@Disabled
public class DO_TeleOpMode extends LinearOpMode {
    private FramedDOBot robot = new FramedDOBot();   // Use a Pushbot's hardware
    private  Boolean stopGripLeftBumper = false;
    private  Boolean stopGripRightBumper = false;
    private Boolean powerRearWheels = false;
    static double     DRIVE_SPEED             = 0.6;
    static double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.latchLockServo.setPosition(FramedDOBot.END_LATCH_SERVO);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Run the robot in the telemode (FWD, BWD and turn)
            if(gamepad1.left_stick_y != 0) { // support FWD and BWD movement
                MoveRobot(powerRearWheels);
            }
            else if(gamepad1.right_stick_x != 0){
                TurnRobot(powerRearWheels);
            }
            else if( (gamepad1.left_trigger != 0) || (gamepad1.right_trigger != 0)) { //Moving the pully for latching and unlatching
                MovePully(); //latching and unlatching
            }
            else if ((gamepad1.right_bumper) || (gamepad1.left_bumper)) {
                LateralMove(DRIVE_SPEED);
            }
            else if (gamepad1.x){
                robot.latchLockServo.setPosition(FramedDOBot.END_LATCH_SERVO);
            }
            else if(gamepad1.b) {
                robot.latchLockServo.setPosition(FramedDOBot.ZERO_LATCH_SERVO);
            }
            else if(gamepad1.a) // toggel the power for rear wheel
            {
                if(powerRearWheels)
                    powerRearWheels = false;
                else
                    powerRearWheels = true;
            }
            else if (gamepad2.right_stick_y != 0) {
                double elbowOffset = Range.clip(gamepad2.right_stick_y/2, -0.5, 0.5);
                robot.leftElbow.setPosition(robot.MID_SERVO + elbowOffset);
                robot.rightElbow.setPosition(robot.MID_SERVO - elbowOffset);
                telemetry.addData("Joystick ",  "Joy value %f, Offset %f, Servo1 %f, Servo 2 %f",
                        gamepad2.right_stick_y, elbowOffset,robot.leftElbow.getPosition(), robot.rightElbow.getPosition() );
                telemetry.update();
            }
            else if(gamepad2.left_stick_y != 0){
                double dist  =  Range.clip(gamepad2.left_stick_y, -1.0, 1.0) ;
                robot.shoulder.setPower(-dist/1.3);
            }
            else if (gamepad2.left_bumper){
                if(stopGripLeftBumper)
                {
                    robot.palm.setPower(0);
                    stopGripLeftBumper = false;
                }
                else
                {
                    robot.palm.setDirection(DcMotor.Direction.FORWARD);
                    robot.palm.setPower(DRIVE_SPEED + .2);
                    stopGripLeftBumper = true;
                }
            }
            else if (gamepad2.right_bumper) {
                if(stopGripRightBumper)
                {
                    robot.palm.setPower(0);
                    stopGripRightBumper = false;
                }
                else
                {
                    robot.palm.setDirection(DcMotor.Direction.REVERSE);
                    robot.palm.setPower(DRIVE_SPEED + .2);
                    stopGripRightBumper = true;
                }
            }
            else {
                robot.AllDrivesSetPower(0, true);
                robot.latchMotor.setPower(0);
                robot.shoulder.setPower(0);
            }
        }
        robot.latchLockServo.setPosition(FramedDOBot.ZERO_LATCH_SERVO);
    }

    private void MovePully(){
        //Latch motor is for reverse direction
        if(gamepad1.left_trigger != 0) {
            robot.latchMotor.setPower(-Range.clip(gamepad1.left_trigger, -1.0, 1.0));
        }
        else if (gamepad1.right_trigger != 0){
            robot.latchMotor.setPower(Range.clip(gamepad1.right_trigger, -1.0, 1.0));
        }
        else
        {
            robot.latchMotor.setPower(0);
        }
    }

    private void MoveRobot(boolean powerRearWheels) {
        // Send calculated power to wheels
        double dist  =  Range.clip(gamepad1.left_stick_y, -1.0, 1.0) ;
        robot.leftDrive.setPower(-dist);
        robot.rightDrive.setPower(-dist);
        if(powerRearWheels) {
            robot.leftDriveBack.setPower(-dist);
            robot.rightDriveBack.setPower(-dist);
        }
    }

    private void TurnRobot(boolean powerRearWheels) {
        robot.leftDrive.setPower(Range.clip(gamepad1.right_stick_x, -1.0, 1.0));
        robot.rightDrive.setPower(-Range.clip(gamepad1.right_stick_x, -1.0, 1.0));
        if(powerRearWheels){
            robot.leftDriveBack.setPower(Range.clip(gamepad1.right_stick_x, -1.0, 1.0));
            robot.rightDriveBack.setPower(-Range.clip(gamepad1.right_stick_x, -1.0, 1.0));
        }
    }

    private void LateralMove(double speed){
        if(gamepad1.right_bumper) {
            speed = (-1) * speed;
        }
        robot.leftDrive.setPower(-speed);
        robot.leftDriveBack.setPower(speed);
        robot.rightDrive.setPower(speed);
        robot.rightDriveBack.setPower(-speed);
    }
}
