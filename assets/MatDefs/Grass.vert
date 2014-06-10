#define ATTENUATION
//#define HQ_ATTENUATION

#import "Common/ShaderLib/Skinning.glsllib"

uniform mat4 g_WorldViewProjectionMatrix;
uniform mat4 g_ViewProjectionMatrix;
uniform mat4 g_WorldViewMatrix;
uniform mat4 g_WorldMatrix;
uniform mat3 g_NormalMatrix;
uniform mat4 g_ViewMatrix;
uniform vec3 g_CameraPosition;

uniform vec4 m_Ambient;
uniform vec4 m_Diffuse;
uniform vec4 m_Specular;
uniform float m_Shininess;

uniform vec4 g_LightColor;
uniform vec4 g_LightPosition;
uniform vec4 g_AmbientLightColor;

varying vec2 texCoord;
#ifdef SEPARATE_TEXCOORD
  varying vec2 texCoord2;
  attribute vec2 inTexCoord2;
#endif

varying vec3 AmbientSum;
varying vec4 DiffuseSum;
varying vec3 SpecularSum;

varying float vDistance;


attribute vec3 inPosition;
attribute vec2 inTexCoord;
attribute vec3 inNormal;

varying vec3 lightVec;
//varying vec4 spotVec;

#ifdef VERTEX_COLOR
  attribute vec4 inColor;
#endif

#ifndef VERTEX_LIGHTING
  attribute vec4 inTangent;

  #ifndef NORMALMAP
    varying vec3 vNormal;
  #endif
  //varying vec3 vPosition;
  varying vec3 vViewDir;
  varying vec4 vLightDir;
#else
  varying vec2 vertexLightValues;
  uniform vec4 g_LightDirection;
#endif

#ifdef USE_REFLECTION
    uniform vec3 g_CameraPosition;
    uniform mat4 g_WorldMatrix;

    uniform vec3 m_FresnelParams;
    varying vec4 refVec;


    /**
     * Input:
     * attribute inPosition
     * attribute inNormal
     * uniform g_WorldMatrix
     * uniform g_CameraPosition
     *
     * Output:
     * varying refVec
     */
    void computeRef(in vec4 modelSpacePos){
        vec3 worldPos = (g_WorldMatrix * modelSpacePos).xyz;

        vec3 I = normalize( g_CameraPosition - worldPos  ).xyz;
        vec3 N = normalize( (g_WorldMatrix * vec4(inNormal, 0.0)).xyz );

        refVec.xyz = reflect(I, N);
        refVec.w   = m_FresnelParams.x + m_FresnelParams.y * pow(1.0 + dot(I, N), m_FresnelParams.z);
    }
#endif

// JME3 lights in world space
void lightComputeDir(in vec3 worldPos, in vec4 color, in vec4 position, out vec4 lightDir){
    float posLight = step(0.5, color.w);
    vec3 tempVec = position.xyz * sign(posLight - 0.5) - (worldPos * posLight);
    lightVec = tempVec;  
    #ifdef ATTENUATION
     float dist = length(tempVec);
     lightDir.w = clamp(1.0 - position.w * dist * posLight, 0.0, 1.0);
     lightDir.xyz = tempVec / vec3(dist);
    #else
     lightDir = vec4(normalize(tempVec), 1.0);
    #endif
}

#ifdef VERTEX_LIGHTING
  float lightComputeDiffuse(in vec3 norm, in vec3 lightdir){
      return max(0.0, dot(norm, lightdir));
  }

  float lightComputeSpecular(in vec3 norm, in vec3 viewdir, in vec3 lightdir, in float shiny){
      if (shiny <= 1.0){
          return 0.0;
      }
      #ifndef LOW_QUALITY
        vec3 H = (viewdir + lightdir) * vec3(0.5);
        return pow(max(dot(H, norm), 0.0), shiny);
      #else
        return 0.0;
      #endif
  }

vec2 computeLighting(in vec3 wvPos, in vec3 wvNorm, in vec3 wvViewDir, in vec4 wvLightPos){
     vec4 lightDir;
     lightComputeDir(wvPos, g_LightColor, wvLightPos, lightDir);
     float spotFallOff = 1.0;
     if(g_LightDirection.w != 0.0){
          vec3 L=normalize(lightVec.xyz);
          vec3 spotdir = normalize(g_LightDirection.xyz);
          float curAngleCos = dot(-L, spotdir);    
          float innerAngleCos = floor(g_LightDirection.w) * 0.001;
          float outerAngleCos = fract(g_LightDirection.w);
          float innerMinusOuter = innerAngleCos - outerAngleCos;
          spotFallOff = clamp((curAngleCos - outerAngleCos) / innerMinusOuter, 0.0, 1.0);
     }
     float diffuseFactor = lightComputeDiffuse(wvNorm, lightDir.xyz);
     float specularFactor = lightComputeSpecular(wvNorm, wvViewDir, lightDir.xyz, m_Shininess);
     //specularFactor *= step(0.01, diffuseFactor);
     return vec2(diffuseFactor, specularFactor) * vec2(lightDir.w)*spotFallOff;
  }
#endif

void main(){
   vec4 modelSpacePos = vec4(inPosition, 1.0);
   vec3 modelSpaceNorm = inNormal;
   
   #ifndef VERTEX_LIGHTING
        vec3 modelSpaceTan  = inTangent.xyz;
   #endif

   #ifdef NUM_BONES
        #ifndef VERTEX_LIGHTING
        Skinning_Compute(modelSpacePos, modelSpaceNorm, modelSpaceTan);
        #else
        Skinning_Compute(modelSpacePos, modelSpaceNorm);
        #endif
   #endif

   // We plot the position of the grass vertexes based on the
   // texture coordinates
   //gl_Position = g_WorldViewProjectionMatrix * modelSpacePos;
   texCoord = inTexCoord;
   #ifdef SEPARATE_TEXCOORD
      texCoord2 = inTexCoord2;
   #endif

   // Find the world location of the vertex.  All three corners
   // will have the same vertex.
   vec3 wPos = (g_WorldMatrix * modelSpacePos).xyz; 

   // We face the billboarded grass towards the camera's location
   // instead of parallel to the screen.  This keeps the blades from
   // sliding around if we turn the camera.
   vec3 cameraOffset = wPos - g_CameraPosition;
   vDistance = length(cameraOffset);
   vec3 cameraDir = cameraOffset / vDistance;   
   vec3 posOffset = normalize(vec3(-cameraDir.z, 0.0, cameraDir.x));

   // The whole part of the x coordinate is the atlas cell.
   // The fractional part says which corner this is.   
   // X fract() will be 0.25, 0.5, or 0.0
   // Y will be 1 at x=0 and x=0.5 but 0 at x=0.25.
   // I kept the decimal part small so that it could be safely
   // extracted from the texture coordinate.  
   float texFract = fract(texCoord.x);
   float offsetLength = (texFract * 2.0) - 0.5; 
   float texY = abs(offsetLength) * 2.0; 
   float normalProjectionLength = texY - 0.25; 
   float size = texCoord.y;
 
   modelSpacePos.xyz += modelSpaceNorm * normalProjectionLength * size;
   wPos = (g_WorldMatrix * modelSpacePos).xyz; 
    
   // Move the upper parts of the triangle along the camera-perpendicular
   // vector (posOffset)    
   wPos += posOffset * offsetLength * size;
    
   gl_Position = g_ViewProjectionMatrix * vec4(wPos, 1.0);

   // Figure out the texture coordinate from the index
   float index = texCoord.x - texFract;
   float u = mod(index, 4.0);
   float v = mod((index - u) * 0.25, 4.0);
   texCoord.x = u * 0.25 + texFract * 0.5;
   texCoord.y = v * 0.25 + texY * 0.25;  
   

   vec3 wvPosition = (g_WorldViewMatrix * modelSpacePos).xyz;
   vec3 wvNormal  = normalize(g_NormalMatrix * modelSpaceNorm);
   vec3 viewDir = normalize(-wvPosition);
  
       //vec4 lightColor = g_LightColor[gl_InstanceID];
       //vec4 lightPos   = g_LightPosition[gl_InstanceID];
       //vec4 wvLightPos = (g_ViewMatrix * vec4(lightPos.xyz, lightColor.w));
       //wvLightPos.w = lightPos.w;

   vec4 wvLightPos = (g_ViewMatrix * vec4(g_LightPosition.xyz,clamp(g_LightColor.w,0.0,1.0)));
   wvLightPos.w = g_LightPosition.w;
   vec4 lightColor = g_LightColor;

   #if defined(NORMALMAP) && !defined(VERTEX_LIGHTING)
     vec3 wvTangent = normalize(g_NormalMatrix * modelSpaceTan);
     vec3 wvBinormal = cross(wvNormal, wvTangent);

     mat3 tbnMat = mat3(wvTangent, wvBinormal * -inTangent.w,wvNormal);
     
     //vPosition = wvPosition * tbnMat;
     //vViewDir  = viewDir * tbnMat;
     vViewDir  = -wvPosition * tbnMat;
     lightComputeDir(wvPosition, lightColor, wvLightPos, vLightDir);
     vLightDir.xyz = (vLightDir.xyz * tbnMat).xyz;
   #elif !defined(VERTEX_LIGHTING)
     vNormal = wvNormal;

     //vPosition = wvPosition;
     vViewDir = viewDir;

     lightComputeDir(wvPosition, lightColor, wvLightPos, vLightDir);

     #ifdef V_TANGENT
        vNormal = normalize(g_NormalMatrix * inTangent.xyz);
        vNormal = -cross(cross(vLightDir.xyz, vNormal), vNormal);
     #endif
   #endif

   //computing spot direction in view space and unpacking spotlight cos
//   spotVec = (g_ViewMatrix * vec4(g_LightDirection.xyz, 0.0) );
//   spotVec.w  = floor(g_LightDirection.w) * 0.001;
//   lightVec.w = fract(g_LightDirection.w);

   lightColor.w = 1.0;
   #ifdef MATERIAL_COLORS
      AmbientSum  = (m_Ambient  * g_AmbientLightColor).rgb;
      DiffuseSum  =  m_Diffuse  * lightColor;
      SpecularSum = (m_Specular * lightColor).rgb;
    #else
      AmbientSum  = vec3(0.2, 0.2, 0.2) * g_AmbientLightColor.rgb; // Default: ambient color is dark gray
      DiffuseSum  = lightColor;
      SpecularSum = vec3(0.0);
    #endif

    #ifdef VERTEX_COLOR
      AmbientSum *= inColor.rgb;
      DiffuseSum *= inColor;
    #endif

    #ifdef VERTEX_LIGHTING
       vertexLightValues = computeLighting(wvPosition, wvNormal, viewDir, wvLightPos);
    #endif

    #ifdef USE_REFLECTION
        computeRef(modelSpacePos);
    #endif 
}
