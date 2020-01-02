#version 330

in vec3 values;

// layout (location = 0) 
out vec4 result;

void main()
{
  // blending ensures that the sum is computed.
  result = vec4(values, 0.0);
}