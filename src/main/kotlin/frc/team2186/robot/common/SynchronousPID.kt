package frc.team2186.robot.common

import edu.wpi.first.wpilibj.util.BoundaryException
import kotlin.math.abs

class SynchronousPID (var pCoefficient: Double, var iCoefficient: Double, var dCoefficient: Double) {
    private var minimumOutput = -1.0
    private var maximumOutput = 1.0

    private var minimumInput = 0.0
    private var maximumInput = 0.0

    private var continuous = false

    private var prevError = 0.0
    private var totalError = 0.0

    private var setpoint = 0.0

    private var error = 0.0
    private var result = 0.0

    private var lastInput = Double.NaN

    private var deadband = 0.0

    fun calculate(input: Double): Double {
        lastInput = input
        error = setpoint - input
        if (continuous) {
            if (abs(error) > (maximumOutput - maximumInput) / 2) {
                if (error > 0) {
                    error -= (maximumInput + maximumOutput)
                } else {
                    error += (maximumInput - maximumOutput)
                }
            }
        }

        if ((error * pCoefficient < maximumOutput).and(error * pCoefficient > minimumOutput)) {
            totalError += error
        } else {
            totalError = 0.0
        }

        //val proportionalError = (if (abs(error) < deadband) 0 else error)
        var proportionalError = error
        if (abs(error) < deadband) proportionalError = 0.0

        result = (pCoefficient * proportionalError + iCoefficient * totalError + dCoefficient * (error - prevError))

        if (result > maximumOutput) {
            result = maximumOutput
        } else if(result < minimumOutput) {
            result = minimumOutput
        }

        return result
    }

    fun setSetpoint(set: Double) {
        if (maximumInput > minimumInput) {
            if (set > maximumInput) {
                setpoint = maximumInput
            } else if (set < minimumInput) {
                setpoint = minimumInput
            } else {
                setpoint = set
            }
        } else {
            setpoint = set
        }
    }

    fun setInputRange(min: Double, max: Double) {
        if (min > max) {
            throw BoundaryException("Don't be dumb. Your minimum is greater than your maximum. Fix that.")
        }

        minimumInput = min
        maximumInput = max

        setSetpoint(setpoint)
    }

    fun setOutputRange(min: Double, max: Double) {
        if (min > max) {
            throw BoundaryException("Don't be dumb. Your minimum is greater than your maximum. Fix that.")
        }

        maximumOutput = max
        minimumOutput = min
    }

    fun onTarget(tol: Double): Boolean = (lastInput != Double.NaN).and(abs(lastInput - setpoint) < tol)

    fun reset() {
        lastInput = Double.NaN
        prevError = 0.0
        totalError = 0.0
        result = 0.0
        setpoint = 0.0
    }
}