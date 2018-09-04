attribute vec4 vPosition;
attribute vec2 vOffset;
uniform mat4 uarMVP;
uniform float uPointSize;
void main(){
    gl_PointSize = uPointSize;
    // uProjMat * uViewMat *
    gl_Position = uarMVP * vec4(vPosition.xyz, 1.0) + vec4(vOffset.x, vOffset.y, .0, .0);
}