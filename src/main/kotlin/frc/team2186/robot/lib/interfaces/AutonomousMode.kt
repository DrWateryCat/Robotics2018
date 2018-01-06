package frc.team2186.robot.lib.interfaces

import edu.wpi.first.wpilibj.Timer

abstract class AutonomousMode {
    abstract fun init()
    abstract fun update()

    val initialTime: Double by lazy { Timer.getFPGATimestamp() }
    var deltaTime = Timer.getFPGATimestamp() - initialTime
}