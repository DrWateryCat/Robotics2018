package frc.team2186.robot.lib.common

fun thread(start: Boolean = true, isDaemon: Boolean = false, contextLoader: ClassLoader? = null, name: String? = null, priority: Int = -1, block: () -> Unit): Thread {
    val thread = object : Thread() {
        override fun run() = block()
    }

    if (isDaemon)
        thread.isDaemon = true
    if (priority > 0)
        thread.priority = priority
    if (name != null)
        thread.name = name
    if (contextLoader != null)
        thread.contextClassLoader = contextLoader
    if (start)
        thread.start()
    return thread
}