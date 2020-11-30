#version 150

uniform mat4 worldviewproj_matrixa;
uniform mat4 inverse_worldview_matrixa;
uniform vec4 camera_position;
uniform float time_0_1;

// These variables are automatically bound by Ogre
in vec4 vertex;
in vec4 normal;
in vec4 colour;

// We pass these variables to the geometry shader
out vec4 v_position;
out vec4 v_color;

void main() {
    v_position = vertex;
    v_color = vec4(1, 0, 0, 1);
    if(colour.a == 0.5)
      v_color = vec4(0, 1, 0, 1);
    //v_color.x = time_0_1;
    //v_color.x = worldviewproj_matrixa[0][0];

    vec3 up = normalize(cross(vertex.xyz - normal.xyz, vertex.xyz - camera_position.xyz));
    float mult = 1;
    if(colour.a == 0)
      mult = -1;
    gl_Position = worldviewproj_matrixa*(vertex + vec4(mult*up, 0.0));

    /*
    float angle = acos(dot(vec3(normal.x, normal.y, normal.z), vec3(vertex.x, vertex.y, vertex.z)));
    vec3 increment = vec3(colour.b, colour.a, 0);
    if(increment.x == 0)
      increment.x = -1;
    if(increment.y == 0)
      increment.y = -1;
    increment.x *= sin(angle);
    increment.y *= cos(angle);
    float temp = increment.x;
    increment.x = increment.y;
    increment.y = temp;
    gl_Position = worldviewproj_matrixa*(vertex + vec4(mat3(inverse_worldview_matrixa)*increment, 0.));
    */
    //gl_LineWidth = 20.f;
}
