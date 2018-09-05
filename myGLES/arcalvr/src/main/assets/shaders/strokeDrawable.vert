attribute vec4 vPosition;
attribute vec2 vOffset;
uniform mat4 uarMVP;
uniform float uPointSize;
varying vec4 vColor;
void main(){
    gl_PointSize = uPointSize;
    vColor = vec4(vOffset.x, vOffset.y, .0, 1.0);
    gl_Position = uarMVP * vec4(vPosition.xyz, 1.0);// + vec4(vOffset.x, vOffset.y, .0, .0);
}