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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
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
public class HardwarePushbotSN
{
    /* Public OpMode members. */
    public DcMotor  ch_m_fl   = null;
    public DcMotor  ch_m_bl  = null;
    public DcMotor  ch_m_fr  = null;
    public DcMotor  ch_m_br  = null;
    public DcMotor  rp_m_a  = null;
    public DcMotor  LS_M_A  = null;


    public DcMotor  leftArm     = null;
    public Servo    leftClaw    = null;
    public Servo    rightClaw   = null;

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwarePushbotSN(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        ch_m_fl  = hwMap.get(DcMotor.class, "CH_M_FL");
        ch_m_bl  = hwMap.get(DcMotor.class, "CH_M_BL");
        ch_m_fr = hwMap.get(DcMotor.class, "CH_M_FR");
        ch_m_br = hwMap.get(DcMotor.class, "CH_M_BR");
        rp_m_a = hwMap.get(DcMotor.class, "RP_M_A");
        LS_M_A = hwMap.get(DcMotor.class, "LS_M_A");

        ch_m_fl.setDirection(DcMotor.Direction.FORWARD);
        ch_m_bl.setDirection(DcMotor.Direction.FORWARD);
        rp_m_a.setDirection(DcMotor.Direction.FORWARD);

        ch_m_br.setDirection(DcMotor.Direction.REVERSE);
        ch_m_fr.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        ch_m_fl.setPower(0);
        ch_m_bl.setPower(0);
        ch_m_fr.setPower(0);
        ch_m_br.setPower(0);
        rp_m_a.setPower(0);
        LS_M_A.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        ch_m_fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ch_m_bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ch_m_fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ch_m_br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rp_m_a.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LS_M_A.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Define and initialize ALL installed servos.
        /*leftClaw  = hwMap.get(Servo.class, "left_hand");
        rightClaw = hwMap.get(Servo.class, "right_hand");
        leftClaw.setPosition(MID_SERVO);
        rightClaw.setPosition(MID_SERVO);*/
    }
 }

