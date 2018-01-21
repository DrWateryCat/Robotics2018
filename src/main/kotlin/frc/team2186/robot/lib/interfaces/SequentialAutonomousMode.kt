package frc.team2186.robot.lib.interfaces

import frc.team2186.robot.lib.common.ActionRunner

abstract class SequentialAutonomousMode : AutonomousMode() {
    abstract val actions: ActionRunner

    override fun init() {
        actions.init()
    }

    override fun update() {
        actions.update()
    }

    override fun done() = actions.done
}