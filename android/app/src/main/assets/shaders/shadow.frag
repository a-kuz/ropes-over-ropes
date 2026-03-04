#version 300 es
precision mediump float;
// Depth-only pass. GLES requires a fragment shader even when only writing depth.
// gl_FragDepth is written automatically by the hardware from gl_Position.z/w.
out vec4 fragColor;
void main() {
    fragColor = vec4(0.0, 0.0, 0.0, 0.0);
}
