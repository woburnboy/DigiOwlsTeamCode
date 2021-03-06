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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Digital Owl Robot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class FramedDOBot
{
    /* Public OpMode members. */
    public DcMotor leftDrive, rightDrive, leftDriveBack, rightDriveBack = null;
    public DcMotor latchMotor   = null;
    public DistanceSensor sensorDistance = null;
    public DcMotor  shoulder    = null;
    public DcMotor  palm        = null;
    public Servo    leftElbow    = null;
    public Servo    rightElbow   = null;
    public Servo    latchLockServo = null;
    public Servo    tokenServo = null;


    public static final double MID_LATCH_SERVO       =  0.5 ;
    public static final double ZERO_LATCH_SERVO       =  0 ;
    public static final double END_LATCH_SERVO       =  .9 ;
    public static final double MID_SERVO       =  0.5 ;

    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    private HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public FramedDOBot(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
//        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
//        rightDrive = hwMap.get(DcMotor.class, "right_drive");
//        leftArm    = hwMap.get(DcMotor.class, "left_arm");
//        gyro = hwMap.get(Gyroscope.class, "imu");
//        testMotor = hwMap.get(DcMotor.class, "testMotor39530");
        leftElbow       = hwMap.get(Servo.class, "lefthand");
        rightElbow      = hwMap.get(Servo.class, "righthand");
        latchLockServo  = hwMap.get(Servo.class, "latchLockServo");
        tokenServo      = hwMap.get(Servo.class, "token");
        shoulder        = hwMap.get(DcMotor.class,   "shoulder"); //Commented by Amit
        palm            = hwMap.get(DcMotor.class,"palm");
        latchMotor      = hwMap.get(DcMotor.class, "latchMotor");
        leftDrive       = hwMap.get(DcMotor.class,    "leftdrive");
        leftDriveBack   = hwMap.get(DcMotor.class,"leftdriveBack");
        rightDrive      = hwMap.get(DcMotor.class,   "rightdrive");
        rightDriveBack  = hwMap.get(DcMotor.class,"rightdriveBack");

        shoulder.setDirection(DcMotor.Direction.REVERSE); //Commented by Amit
        palm.setDirection(DcMotor.Direction.REVERSE); //Commented by Amit
        latchMotor.setDirection(DcMotor.Direction.REVERSE);     // Set to REVERSE if using AndyMark motors
        leftDrive.setDirection(DcMotor.Direction.REVERSE);      // Set to REVERSE if using AndyMark motors
        leftDriveBack.setDirection(DcMotor.Direction.REVERSE);  // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.FORWARD);     // Set to FORWARD if using AndyMark motors
        rightDriveBack.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors


//        digitalTouch = hwMap.get(DigitalChannel.class, "digitalTouch");
        // Set all motors to zero power
        AllDrivesSetPower(0, true);
        latchMotor.setPower(0);
        shoulder.setPower(0); //Commented by Amit
        palm.setPower(0);
        //        leftArm.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        latchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        leftElbow.setPosition(0);
        rightElbow.setPosition(1);
        latchLockServo.setPosition(ZERO_LATCH_SERVO);
        tokenServo.setPosition(0);
    }

    public void AllDrivesSetPower(double power, Boolean turnRearMotor) {
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        if(turnRearMotor) {
            leftDriveBack.setPower(power);
            rightDriveBack.setPower(power);
        }
        else{
            leftDriveBack.setPower(0);
            rightDriveBack.setPower(0);
        }
    }
    public void AllDrivesSetMode(DcMotor.RunMode mode){
        leftDrive.setMode(mode);
        rightDrive.setMode(mode);
        leftDriveBack.setMode(mode);
        rightDriveBack.setMode(mode);
    }
}

