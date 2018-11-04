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
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name = "Red Crater is Behind: Auto", group = "DigiOwls")
//@Disabled
public class DOAutonomousRedCraterBehind extends LinearOpMode {
    DORobotOperationsDelegate robotOpsDelegate = new DORobotOperationsDelegate();
    static double     DRIVE_SPEED             = 0.6;
    static double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robotOpsDelegate.robot.init(hardwareMap);
        robotOpsDelegate.robot.AllDrivesSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotOpsDelegate.robot.AllDrivesSetMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // reset the timeout time and start motion.
        robotOpsDelegate.runtime.reset();
        RunInToPath1();
    }


    private void RunInToPath1() {
        telemetry.setAutoClear(false);

        //Unlatch robot
        robotOpsDelegate.UnLatchRobot(this, DRIVE_SPEED, 5, 5, "Unlatching the robot");

        //run lateral
        robotOpsDelegate.encoderDrive(this, DRIVE_SPEED,2,-2,-2,2, 5.0, "Move away (right) from notch", true);

        //run forward
        robotOpsDelegate.encoderDrive(this, DRIVE_SPEED,10,10,10,10, 5.0, "Move forward", false);

        //run lateral to left for 10inch to hit all the jewels
//        //Gyanesh MoveLeftScanAndHitJewel();
//
        //Turn to right
        robotOpsDelegate.encoderDrive(this, DRIVE_SPEED,5, -5, 5, -5, 5, "Lateral right", false);

        //run towards wall
        robotOpsDelegate.encoderDrive(this, DRIVE_SPEED,-20, -20, -20, -20, 5, "Lateral right", false);

//        //Turn right parallel to wall
        robotOpsDelegate.encoderDrive(this, DRIVE_SPEED,5, -5, 5, -5, 5, "Left for clearance", false);

        //run towards team's zone
        robotOpsDelegate.encoderDrive(this, DRIVE_SPEED,-20, -20, -20,-20, 5, "Reaching zone", false);
//
//        //Finally, go and park in crater
        robotOpsDelegate.encoderDrive(this, DRIVE_SPEED,30, 30, 30,30, 5, "Parking to carater", false);
    }
}