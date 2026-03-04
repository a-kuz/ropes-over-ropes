package com.uzls.four.game

import android.view.MotionEvent
import com.uzls.four.renderer.GameRenderer
import kotlin.math.atan2
import kotlin.math.sqrt

class InputHandler(
    private val renderer: GameRenderer,
    private val surfaceView: GameGLSurfaceView
) {
    private var activeDragPointerId = -1
    private var twoFingerActive = false
    private var singleTouchCancelled = false

    // Pinch/rotation state
    private var lastPinchDist = 0f
    private var lastPinchAngle = 0f

    // Triple-tap detection
    private var tapCount = 0
    private var lastTapTime = 0L

    fun onTouchEvent(event: MotionEvent): Boolean {
        val pointerCount = event.pointerCount

        when (event.actionMasked) {
            MotionEvent.ACTION_DOWN -> {
                twoFingerActive = false
                singleTouchCancelled = false
                activeDragPointerId = event.getPointerId(0)
                renderer.postTouchEvent(TouchPhase.BEGAN, event.x, event.y)
            }

            MotionEvent.ACTION_POINTER_DOWN -> {
                if (pointerCount >= 2) {
                    if (!twoFingerActive) {
                        twoFingerActive = true
                        if (!singleTouchCancelled) {
                            singleTouchCancelled = true
                            renderer.postTouchEvent(TouchPhase.CANCELLED, 0f, 0f)
                        }
                    }
                    lastPinchDist = pinchDistance(event)
                    lastPinchAngle = pinchAngle(event)
                }
            }

            MotionEvent.ACTION_MOVE -> {
                if (twoFingerActive && pointerCount >= 2) {
                    val dist = pinchDistance(event)
                    if (lastPinchDist > 0f) {
                        val scale = dist / lastPinchDist
                        renderer.postCameraZoom(1f / scale)
                    }
                    lastPinchDist = dist

                    val angle = pinchAngle(event)
                    val delta = angle - lastPinchAngle
                    renderer.postCameraSpin(delta)
                    lastPinchAngle = angle
                } else if (!singleTouchCancelled) {
                    val idx = event.findPointerIndex(activeDragPointerId)
                    if (idx >= 0) {
                        renderer.postTouchEvent(TouchPhase.MOVED, event.getX(idx), event.getY(idx))
                    }
                }
            }

            MotionEvent.ACTION_UP -> {
                if (!singleTouchCancelled) {
                    renderer.postTouchEvent(TouchPhase.ENDED, event.x, event.y)

                    // Triple-tap detection
                    val now = System.currentTimeMillis()
                    if (now - lastTapTime < 500L) {
                        tapCount++
                        if (tapCount >= 3) {
                            renderer.postCameraDebugToggle()
                            tapCount = 0
                        }
                    } else {
                        tapCount = 1
                    }
                    lastTapTime = now
                }
                resetState()
            }

            MotionEvent.ACTION_CANCEL -> {
                if (!singleTouchCancelled) {
                    renderer.postTouchEvent(TouchPhase.CANCELLED, 0f, 0f)
                }
                resetState()
            }

            MotionEvent.ACTION_POINTER_UP -> {
                if (pointerCount <= 2) {
                    twoFingerActive = false
                }
            }
        }
        return true
    }

    private fun resetState() {
        activeDragPointerId = -1
        twoFingerActive = false
        singleTouchCancelled = false
        lastPinchDist = 0f
    }

    private fun pinchDistance(event: MotionEvent): Float {
        if (event.pointerCount < 2) return 0f
        val dx = event.getX(0) - event.getX(1)
        val dy = event.getY(0) - event.getY(1)
        return sqrt(dx * dx + dy * dy)
    }

    private fun pinchAngle(event: MotionEvent): Float {
        if (event.pointerCount < 2) return 0f
        val dx = event.getX(1) - event.getX(0)
        val dy = event.getY(1) - event.getY(0)
        return atan2(dy, dx)
    }
}

enum class TouchPhase {
    BEGAN, MOVED, ENDED, CANCELLED
}
