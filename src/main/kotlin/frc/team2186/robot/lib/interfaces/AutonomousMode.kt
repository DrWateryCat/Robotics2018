package frc.team2186.robot.lib.interfaces

import edu.wpi.first.wpilibj.Timer

abstract class AutonomousMode (val name: String, val default: Boolean = false){
    abstract fun init()
    abstract fun update()

    abstract fun done(): Boolean

    val initialTime: Double by lazy { Timer.getFPGATimestamp() }
    var deltaTime = Timer.getFPGATimestamp() - initialTime
}