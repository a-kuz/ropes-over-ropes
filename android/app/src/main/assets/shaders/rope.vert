#version 300 es
precision highp float;

layout(std140) uniform FrameBlock {
    mat4 uViewProj;
    mat4 uInvViewProj;
    mat4 uLightViewProj;
    vec4 uLightDirIntensity;
    vec4 uAmbientColor;
    vec4 uCameraPos;
    vec4 uOrthoHalfSizeShadowBias;
    vec4 uShadowInvSize;
    vec4 uTimeDrag;
    vec4 uWoodBoundsMin;
    vec4 uWoodBoundsMax;
    vec4 uHoleTint;
    vec4 uVisualParams;
    vec4 uLightingParams;
    vec4 uTableParams;
    vec4 uTableParams2;
    vec4 uRopeMatParams;
    vec4 uRopeMatParams2;
    vec4 uRopeMatParams3;
    vec4 uCartoonParams;
    vec4 uWormParams1;
    vec4 uWormParams2;
    vec4 uWormParams3;
    vec4 uWormParams4;
    vec4 uRopeMatParams4;
};

layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec3 aColor;
layout(location = 3) in vec2 aTexCoord;
layout(location = 4) in vec4 aParams;

out vec3 vNormal;
out vec3 vColor;
out vec3 vWorldPos;
out vec2 vUV;
out vec4 vParams;

void main() {
    float time = uTimeDrag.x;
    float energy = uTimeDrag.y;
    float dragActive = uTimeDrag.w;
    float u = aTexCoord.x;
    float pinch = aParams.y;
    float isWorm = aParams.w;

    float w = sin(u * 3.14159265);
    w = w * w;

    vec3 displaced;
    if (isWorm > 0.5) {
        float crawlSpeed = uWormParams4.x;
        float crawlAmp = uWormParams4.y;
        float sideAmpP = uWormParams4.z;
        float crawlWave = sin(u * 12.0 - time * crawlSpeed) * crawlAmp * w;
        float sideWave = sin(u * 8.0 - time * crawlSpeed * 0.57) * sideAmpP * w;
        vec3 nDir = normalize(aNormal);
        vec3 sideDir = cross(nDir, vec3(0, 0, 1));
        if (length(sideDir) < 1e-3) sideDir = cross(nDir, vec3(0, 1, 0));
        sideDir = normalize(sideDir);
        displaced = aPosition + nDir * crawlWave + sideDir * sideWave;
    } else {
        float amp = (0.002 + 0.010 * pinch) * energy * (0.25 + 0.75 * dragActive) * w;
        float wave = sin(u * 24.0 + time * 16.0) * 0.65 + sin(u * 11.0 - time * 9.0) * 0.35;
        displaced = aPosition + normalize(aNormal) * (wave * amp);
    }

    vWorldPos = displaced;
    gl_Position = uViewProj * vec4(displaced, 1.0);
    vNormal = aNormal;
    vColor = aColor;
    vUV = aTexCoord;
    vParams = aParams;
}
