#version 300 es
// Port of Metal bloomBlurH / bloomBlurV as one shader with uniform direction.
// 9-tap Gaussian with same weights as Metal version.
// Use uDirection = vec2(1.0/width, 0.0) for horizontal,
//     uDirection = vec2(0.0, 1.0/height) for vertical.
precision highp float;

uniform sampler2D uSrcTex;
uniform vec2 uDirection; // texel step direction

in vec2 vUV; // from fullscreen.vert

out vec4 fragColor;

// Exact weights from Metal version
const float weights[9] = float[9](
    0.1574, 0.1476, 0.1216, 0.0879, 0.0559,
    0.0311, 0.0152, 0.0065, 0.0024
);

void main() {
    vec2 uv = vUV;

    // Center sample
    vec4 acc = texture(uSrcTex, uv) * weights[0];

    // Offset samples (clamped by texture sampler clamp_to_edge mode)
    for (int i = 1; i <= 8; i++) {
        vec2 offset = uDirection * float(i);
        acc += texture(uSrcTex, uv + offset) * weights[i];
        acc += texture(uSrcTex, uv - offset) * weights[i];
    }

    fragColor = acc;
}
