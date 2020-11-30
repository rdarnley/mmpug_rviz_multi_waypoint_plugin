#version 150
#extension GL_ARB_conservative_depth : enable

uniform vec4 shading;

in vec4 v_position;
in vec4 v_color;

out vec4 pixel_color;

// Help the renderer to avoid shader invocations for fragments which will be
// discarded anyway
layout(depth_greater) out float gl_FragDepth;

void main(){
  pixel_color = v_color;
  gl_FragDepth = 0.1;//gl_FragCoord.z;// * (1 + 0.001 * rsquared);

}

/*
// rasterizes a little camera-facing cone that looks like a shaded circle
// and merges nicely with surrounding points in dense point clouds

uniform vec4 shading;

// These attributes are passed from the geometry shader
in vec4 frag_color;
in vec4 tex_coord;

// The output variable will automatically become the fragment's color
out vec4 pixel_color;

// Help the renderer to avoid shader invocations for fragments which will be
// discarded anyway
layout(depth_greater) out float gl_FragDepth;

void main()
{
  float ax = tex_coord.x;
  float ay = tex_coord.y;

  float rsquared = ax*ax+ay*ay;
  if (rsquared >= 1)
  {
    discard;
  }
  pixel_color = vec4(frag_color.xyz * (1 - 0.2 * rsquared), 1);
  gl_FragDepth = gl_FragCoord.z * (1 + 0.001 * rsquared);
}
*/
