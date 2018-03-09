package frc.team2186.robot.lib.common

import edu.wpi.first.wpilibj.Joystick

fun Joystick.pressedButtons() = (1 until buttonCount).filter { getRawButton(it) }
val Joystick.buttons: List<Int>
    get() = pressedButtons()
operator fun Joystick.get(index: Int) = getRawAxis(index)