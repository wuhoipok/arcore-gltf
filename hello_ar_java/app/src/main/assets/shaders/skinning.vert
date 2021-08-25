#version 300 es
/*
 * Copyright 2017 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

attribute vec4 a_joint;
attribute vec4 a_weight;

uniform mat4 u_modelViewMatrix;
uniform mat4 u_projectionMatrix;
uniform mat4 u_jointMatrix[];
uniform float u_PointSize;

layout(location = 0) in vec4 a_Position;

void main() {
    mat4 skinMatrix = a_weight * u_jointMatrix[a_joint];

    vec4 pos = u_modelViewMatrix * skinMatrix * vec4(a_Position, 1.0);
    gl_Position = u_projectionMatrix * pos;
    gl_PointSize = u_PointSize;
}