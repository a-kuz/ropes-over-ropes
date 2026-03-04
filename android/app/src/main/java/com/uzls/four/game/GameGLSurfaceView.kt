package com.uzls.four.game

import android.annotation.SuppressLint
import android.content.Context
import android.opengl.GLSurfaceView
import android.view.MotionEvent
import com.uzls.four.renderer.GameRenderer

class GameGLSurfaceView(context: Context) : GLSurfaceView(context) {
    val gameRenderer: GameRenderer
    private val inputHandler: InputHandler

    init {
        setEGLContextClientVersion(3)
        setEGLConfigChooser(8, 8, 8, 8, 24, 8)

        gameRenderer = GameRenderer(context)
        setRenderer(gameRenderer)
        renderMode = RENDERMODE_CONTINUOUSLY

        inputHandler = InputHandler(gameRenderer, this)
    }

    @SuppressLint("ClickableViewAccessibility")
    override fun onTouchEvent(event: MotionEvent): Boolean {
        return inputHandler.onTouchEvent(event)
    }
}
