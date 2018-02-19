package frc.team2186.robot.autonomous

import frc.team2186.robot.lib.interfaces.AutonomousMode

class DoNothing : AutonomousMode("Do Nothing", true) {

    override fun update() {}

    override fun init() {}

    override fun done(): Boolean {
        return true
    }
}
