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
};

// Per-vertex attributes
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;

// Per-instance attribute: (x, y, elevation, radius)
layout(location = 2) in vec4 aInstanceData;

out vec3 vNormal;
out vec3 vWorldPos;
out float vHighlight;
out float vHoleId;

void main() {
    float radius    = aInstanceData.w;
    float elevation = aInstanceData.z;
    vec3 lp = aPosition * radius;
    vec3 wp = vec3(aInstanceData.x + lp.x, aInstanceData.y + lp.y, elevation + lp.z);

    float hlIdx = uAmbientColor.w;
    float iid = float(gl_InstanceID);
    float isHighlight = (hlIdx >= 0.0 && abs(iid - hlIdx) < 0.5) ? 1.0 : 0.0;

    vWorldPos  = wp;
    gl_Position = uViewProj * vec4(wp, 1.0);
    vNormal    = normalize(aNormal);
    vHighlight = isHighlight;
    vHoleId    = iid;
}
