package frc.team2186.robot.common

import com.google.gson.GsonBuilder
import com.google.gson.JsonArray
import com.google.gson.JsonObject
import frc.team2186.robot.Robot
import frc.team2186.robot.lib.interfaces.Subsystem
import frc.team2186.robot.subsystems.Drive
import java.io.File

object RecordingManager : Subsystem() {
    const val PATH_PREFIX = "/home/lvuser/paths/"

    data class TimeData(val timestamp: Double, val leftRPM: Double, val rightRPM: Double)
    data class RecordedPath(val name: String, val elements: List<TimeData>) {
        fun toJson() = JsonObject().apply {
            addProperty("name", name)
            add("elements", JsonArray().apply {
                elements.forEach {
                    add(JsonObject().apply {
                        addProperty("timestamp", it.timestamp)
                        addProperty("leftRPM", it.leftRPM)
                        addProperty("rightRPM", it.rightRPM)
                    })
                }
            })
        }
    }
    enum class Paths {
        LeftSwitchLeft,
        LeftSwitchRight,
        MiddleSwitchLeft,
        MiddleSwitchRight,
        RightSwitchLeft,
        RightSwitchRight,
        Baseline
    }
    private val recordingFileNames = mapOf(
        Paths.LeftSwitchLeft to "recording-left-switch-left.json",
        Paths.LeftSwitchRight to "recording-left-switch-right.json",
        Paths.MiddleSwitchLeft to "recording-mid-switch-left.json",
        Paths.MiddleSwitchRight to "recording-mid-switch-right.json",
        Paths.RightSwitchLeft to "recording-right-switch-left.json",
        Paths.RightSwitchRight to "recording-right-switch-right.json",
        Paths.Baseline to "recording-baseline.json"
    )
    private val gson
            by lazy { GsonBuilder().apply {
        setPrettyPrinting()
    }.create() }

    private var recording = false
    private var playing = false
    private var recordingBuffer = ArrayList<TimeData>()

    var finished = false
        private set

    fun record() {
        recording = true
    }

    fun play(path: Paths) {
        val p = loadPath(path)
        recordingBuffer.addAll(p.elements)
        playing = true
    }

    fun save() {
        savePath(Robot.SelectedPath, recordingBuffer)
    }

    fun stop() {
        recording = false
        playing = false
    }

    fun loadPath(path: Paths): RecordedPath {
        val filename = recordingFileNames[path]
        return try {
            val f = File(PATH_PREFIX + filename)
            require(f.isFile and f.exists() and f.canRead())
            val fileContents = f.readText()
            gson.fromJson(fileContents, RecordedPath::class.java)!!
        } catch (e: Exception) {
            e.printStackTrace()
            RecordedPath("null", arrayListOf(TimeData(0.0, 0.0, 0.0)))
        }
    }

    private fun deleteFile(f: String) {
        try {
            val file = File(f)
            file.delete()
        } catch (e: Exception) {
            e.printStackTrace()
        }
    }

    fun savePath(path: Paths, value: List<TimeData>) {
        val filename = recordingFileNames[path]
        return try {
            deleteFile(PATH_PREFIX + filename)
            val f = File(PATH_PREFIX + filename)
            val pathObj = RecordedPath(filename!!, value)
            val jsonContents = gson.toJson(pathObj.toJson())
            println(jsonContents)
            f.printWriter().use {
                it.println(jsonContents)
            }
        } catch (e: Exception) {
            e.printStackTrace()
        }
    }

    override fun update() {
        if (recording) {
            recordingBuffer.add(TimeData(0.0, Drive.leftSpeed, Drive.rightSpeed))
        } else if(playing) {
            try {
                val currentData = recordingBuffer[0]
                Drive.setLeftRightVelocity(currentData.leftRPM, -currentData.rightRPM)
                recordingBuffer.removeAt(0)
            } catch (e: Exception) {
                playing = false
                finished = true
            }
        }
    }
}