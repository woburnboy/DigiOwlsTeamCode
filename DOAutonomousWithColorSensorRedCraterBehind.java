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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@Autonomous (name = "With ColorSeSor-Crater is Behind: Auto", group = "DigiOwls")
//@Disabled
public class DOAutonomousWithColorSensorRedCraterBehind extends LinearOpMode {
    DORobotOperationsDelegate robotOpsDelegate = new DORobotOperationsDelegate();
    static double     DRIVE_SPEED             = 0.6;
    static double     TURN_SPEED              = 0.5;
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;

    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robotOpsDelegate.robot.init(hardwareMap);

        robotOpsDelegate.robot.AllDrivesSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotOpsDelegate.robot.AllDrivesSetMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "sensorColor");
        sensorColor.enableLed(true);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // reset the timeout time and start motion.
        robotOpsDelegate.runtime.reset();
        RunInToPath1();
    }


    private void RunInToPath1() {
        telemetry.setAutoClear(false);

        //Unlatch robot
//        //First we move a bit up
//        robotOpsDelegate.UnLatchRobot(this, DRIVE_SPEED, -0.8, 5, "Unlatching the robot");
//        //Then we unlock the latch Servo
//        robotOpsDelegate.unlockLatchServo(this,5);
//        // then we bring down the robot
//        robotOpsDelegate.UnLatchRobot(this, DRIVE_SPEED, 3.5, 5, "Unlatching the robot");


        //run lateral
        robotOpsDelegate.encoderDrive(this, DRIVE_SPEED,-4,4,4,-4, 5.0, "Move away (right) from notch", true);

        //run forward
        robotOpsDelegate.encoderDrive(this, 0.9,12,12,12,12, 5.0, "Move forward", false);

        //Delegate move the robot to 3 locations, find the yellow and push it,
        // then it should come back to the same place. Whereever started from. (so encoder
        robotOpsDelegate.findAndPushYellow (this, sensorColor, sensorDistance);

        //run lateral to left for 10inch to hit all the jewels
//        //Gyanesh MoveLeftScanAndHitJewel();
//
        //Turn to left
        robotOpsDelegate.encoderDrive(this, DRIVE_SPEED,-20, 20, -20, 20, 5000, "Lateral left", true);

        //run towards wall
        robotOpsDelegate.encoderDrive(this, DRIVE_SPEED,48, 48, 48, 48, 5000, "Lateral right", false);

//        //Turn slight left
        robotOpsDelegate.encoderDrive(this, DRIVE_SPEED,-16, 16, -16, 16, 5000, "slight left", true);

        //run towards team's zone
        robotOpsDelegate.encoderDrive(this, 0.9,60, 60, 60,60, 5000, "Reaching zone", false);
//
//        //Finally, go and park in crater
//        robotOpsDelegate.encoderDrive(this, DRIVE_SPEED,30, 30, 30,30, 5, "Parking to carater", false);

        //Start the reverse Journey
        robotOpsDelegate.encoderDrive(this, 0.9,-60, -60, -60,-60, 5000, "Coming Back ", false);

        //Turn slight Right
        robotOpsDelegate.encoderDrive(this, DRIVE_SPEED,16, -16, 16, -16, 5000, "slight Right", true);

        //run towards middle
        robotOpsDelegate.encoderDrive(this, DRIVE_SPEED,-44, -44, -44, -44, 5000, "Run to Middle", false);

        //Turn to crater
        robotOpsDelegate.encoderDrive(this, DRIVE_SPEED,20, -20, 20, -20, 5000, "Lateral right", true);

        //run forward and touch the wall of crater
        robotOpsDelegate.encoderDrive(this, 0.9,30,30,30,30, 5.0, "Move forward", false);

    }
}