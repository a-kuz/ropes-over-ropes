#version 300 es
// Port of Metal postFragment: tone mapping, bloom composite, cartoon edge detection.
// Exact 1:1 match with Metal PostParams struct and postFragment function.
precision highp float;
precision highp sampler2D;

// PostParams equivalent (passed as individual uniforms)
uniform float uExposure;
uniform float uBloomStrength;
uniform float uCartoonEdgeStrength;
uniform float uCartoonMode;
uniform float uCartoonEdgeSmooth;

uniform sampler2D uHdrTex;     // HDR color
uniform sampler2D uBloomTex;   // Bloom
uniform sampler2D uDepthTex;   // Depth buffer (for edge detection)

in vec2 vUV;

out vec4 fragColor;

// ---------- Depth-based Sobel edge detection (exact port from Metal edgeDetect) ----------
float edgeDetect(vec2 uv, float edgeSmooth) {
    // Get depth texture dimensions
    vec2 dims = vec2(textureSize(uDepthTex, 0));
    float spread = mix(1.0, 3.0, edgeSmooth);
    vec2 st = spread / dims;
    vec2 c = uv;

    float d00 = texture(uDepthTex, c + vec2(-1, -1) * st).r;
    float d10 = texture(uDepthTex, c + vec2( 0, -1) * st).r;
    float d20 = texture(uDepthTex, c + vec2( 1, -1) * st).r;
    float d01 = texture(uDepthTex, c + vec2(-1,  0) * st).r;
    float d21 = texture(uDepthTex, c + vec2( 1,  0) * st).r;
    float d02 = texture(uDepthTex, c + vec2(-1,  1) * st).r;
    float d12 = texture(uDepthTex, c + vec2( 0,  1) * st).r;
    float d22 = texture(uDepthTex, c + vec2( 1,  1) * st).r;

    float gx = -d00 - 2.0 * d01 - d02 + d20 + 2.0 * d21 + d22;
    float gy = -d00 - 2.0 * d10 - d20 + d02 + 2.0 * d12 + d22;
    float grad = sqrt(gx * gx + gy * gy);
    float threshold = mix(0.002, 0.0005, edgeSmooth);
    return smoothstep(threshold * 0.5, threshold * 1.5, grad);
}

void main() {
    vec2 uv = vUV;
    vec3 c = texture(uHdrTex, uv).xyz;
    vec3 b = texture(uBloomTex, uv).xyz;

    // Add bloom
    c += b * uBloomStrength;

    // Tone mapping: 1 - exp(-c * exposure), same as Metal
    vec3 mapped = 1.0 - exp(-c * uExposure);

    // Gamma correction (linear -> sRGB)
    mapped = pow(clamp(mapped, 0.0, 1.0), vec3(1.0 / 2.2));

    // Cartoon mode
    if (uCartoonMode > 0.5) {
        // Posterize
        mapped = floor(mapped * 32.0 + 0.5) / 32.0;

        // Depth-based edge detection
        if (uCartoonEdgeStrength > 0.01) {
            float edge = edgeDetect(uv, uCartoonEdgeSmooth);
            float darken = 1.0 - uCartoonEdgeStrength;
            mapped = mix(mapped, mapped * darken, edge);
        }
    }

    fragColor = vec4(mapped, 1.0);
}
