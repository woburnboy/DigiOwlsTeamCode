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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DORobotOperationsDelegate  {
    /* Declare OpMode members. */
    FramedDOBot             robot                   = new FramedDOBot();   // Use a Pushbot's hardware
    public ElapsedTime     runtime                 = new ElapsedTime();
    private static final double     COUNTS_PER_MOTOR_REV    = 1120;//1040 ;    // eg: AndyMark NeverRest40
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private static final double     COUNTS_PER_MOTOR_REV_Latch    = 1120;
    private static final double     DRIVE_GEAR_REDUCTION_Latch    = 1.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES_Latch   = 1.45 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH_Latch         = (COUNTS_PER_MOTOR_REV_Latch * DRIVE_GEAR_REDUCTION_Latch) /
            (WHEEL_DIAMETER_INCHES_Latch * 3.1415);

    public void unlockLatchServo(LinearOpMode mode, double timeOut){
        // Ensure that the opmode is still active
        if (mode.opModeIsActive()) {
            robot.latchLockServo.setPosition(FramedDOBot.END_LATCH_SERVO);

            runtime.reset();
            while (mode.opModeIsActive() &&
                   (robot.latchMotor.isBusy() ) &&
                   (runtime.seconds() < timeOut)){
            }
            mode.telemetry.addData("unlatching the lock servo %.1f s", runtime.seconds());
            mode.telemetry.update();
        }
    }

    public void UnLatchRobot(LinearOpMode mode, double speed, double inches, double timeOut, String operation){
        // Ensure that the opmode is still active
        int newUnlatchTarget = 0;
        if (mode.opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newUnlatchTarget = robot.latchMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH_Latch);
            robot.latchMotor.setTargetPosition(newUnlatchTarget);

            // Turn On RUN_TO_POSITION
            robot.latchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.latchMotor.setPower(Math.abs(speed));
            runtime.reset();
            while (mode.opModeIsActive() &&
                    (robot.latchMotor.isBusy() ) &&
                    (runtime.seconds() < timeOut)){

            }
            // Stop all motion;
            robot.latchMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.latchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            mode.telemetry.addData(operation, "takes %.1f s", runtime.seconds());
            mode.telemetry.update();
        }
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(LinearOpMode mode, double speed, double leftInches, double rightInches,
                              double leftBackInches, double rightBackInches, double timeOut, String operation, Boolean goLateral) {
        int newLeftTarget, newLeftBackTarget;
        int newRightTarget, newRightBackTarget;

        // Ensure that the opmode is still active
        if (mode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget       = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget      = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftBackTarget   = robot.leftDriveBack.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newRightBackTarget  = robot.rightDriveBack.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);

            mode.telemetry.addData(operation, "LTgt %d, RTgt %d, LBTgt %d, RBTgt %d", newLeftTarget, newRightTarget, newLeftBackTarget, newRightBackTarget);
            mode.telemetry.update();

            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);
            robot.leftDriveBack.setTargetPosition(newLeftBackTarget);
            robot.rightDriveBack.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.AllDrivesSetMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.AllDrivesSetPower(Math.abs(speed), goLateral);

            runtime.reset();
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ( mode.opModeIsActive() &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() /*&&
                    robot.leftDriveBack.isBusy() && robot.rightDriveBack.isBusy()*/) &&
                    (runtime.seconds() < timeOut)){
                //sleep(100);
            }
            mode.telemetry.addData(operation, "takes %.1f s", runtime.seconds());
            mode.telemetry.update();
        }
        if(!mode.opModeIsActive()){
            // Stop all motion;
            robot.AllDrivesSetPower(0, true);
            // Turn off RUN_TO_POSITION
            robot.AllDrivesSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}