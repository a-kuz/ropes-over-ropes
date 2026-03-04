#version 300 es
// Port of Metal bloomThreshold compute kernel as fullscreen fragment shader.
// Input: HDR texture. Output: bright pixels above threshold.
precision highp float;

uniform sampler2D uHdrTex;

in vec2 vUV; // from fullscreen.vert

out vec4 fragColor;

void main() {
    // Sample from HDR source
    vec3 c = texture(uHdrTex, vUV).xyz;

    // Luminance
    float lum = dot(c, vec3(0.2126, 0.7152, 0.0722));

    // Soft threshold with smoothstep (matches Metal version exactly)
    float t = smoothstep(0.35, 0.75, lum);
    vec3 bright = c * t;

    // Add excess above 0.8
    bright += max(c - 0.8, vec3(0.0));

    fragColor = vec4(bright, 1.0);
}
