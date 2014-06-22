#import "Common/ShaderLib/Parallax.glsllib"
#import "Common/ShaderLib/Optics.glsllib"
#define ATTENUATION
//#define HQ_ATTENUATION

#import "MatDefs/FragScattering.glsllib"
 
// Trinlinear mapping related stuff
uniform mat3 g_NormalMatrix;
varying vec3 worldNormal;

varying vec3 worldTangent;

varying float z;
uniform sampler2D m_Noise;

uniform float m_LowResDistance;
uniform sampler2D m_DiffuseMapLow;
uniform sampler2D m_DiffuseMapX;
uniform sampler2D m_NormalMapX;
uniform sampler2D m_DiffuseMapY;
uniform sampler2D m_NormalMapY;
uniform sampler2D m_DiffuseMapZ;
uniform sampler2D m_NormalMapZ;


varying vec3 texCoord;
#ifdef SEPARATE_TEXCOORD
  varying vec2 texCoord2;
#endif

varying vec3 AmbientSum;
varying vec4 DiffuseSum;
varying vec3 SpecularSum;

#ifndef VERTEX_LIGHTING
  uniform vec4 g_LightDirection;
  //varying vec3 vPosition;
  varying vec3 vViewDir;
  varying vec4 vLightDir;
  varying vec3 lightVec;
#else
  varying vec2 vertexLightValues;
#endif

#ifdef DIFFUSEMAP
  uniform sampler2D m_DiffuseMap;
#endif

#ifdef SPECULARMAP
  uniform sampler2D m_SpecularMap;
#endif

#ifdef PARALLAXMAP
  uniform sampler2D m_ParallaxMap;  
#endif
#if (defined(PARALLAXMAP) || (defined(NORMALMAP_PARALLAX) && defined(NORMALMAP))) && !defined(VERTEX_LIGHTING) 
    uniform float m_ParallaxHeight;
#endif

#ifdef LIGHTMAP
  uniform sampler2D m_LightMap;
#endif
  
#ifdef NORMALMAP
  uniform sampler2D m_NormalMap;
  
  // For debugging we want it either way
  varying vec3 vNormal;     
#else
  varying vec3 vNormal;
#endif

#ifdef ALPHAMAP
  uniform sampler2D m_AlphaMap;
#endif

#ifdef COLORRAMP
  uniform sampler2D m_ColorRamp;
#endif

uniform float m_AlphaDiscardThreshold;

#ifndef VERTEX_LIGHTING
uniform float m_Shininess;

#ifdef HQ_ATTENUATION
uniform vec4 g_LightPosition;
#endif

#ifdef USE_REFLECTION 
    uniform float m_ReflectionPower;
    uniform float m_ReflectionIntensity;
    varying vec4 refVec;

    uniform ENVMAP m_EnvMap;
#endif

float tangDot(in vec3 v1, in vec3 v2){
    float d = dot(v1,v2);
    #ifdef V_TANGENT
        d = 1.0 - d*d;
        return step(0.0, d) * sqrt(d);
    #else
        return d;
    #endif
}

float lightComputeDiffuse(in vec3 norm, in vec3 lightdir, in vec3 viewdir){
    #ifdef MINNAERT
        float NdotL = max(0.0, dot(norm, lightdir));
        float NdotV = max(0.0, dot(norm, viewdir));
        return NdotL * pow(max(NdotL * NdotV, 0.1), -1.0) * 0.5;
    #else
        return max(0.0, dot(norm, lightdir));
    #endif
}

float lightComputeSpecular(in vec3 norm, in vec3 viewdir, in vec3 lightdir, in float shiny){
    // NOTE: check for shiny <= 1 removed since shininess is now 
    // 1.0 by default (uses matdefs default vals)
    #ifdef LOW_QUALITY
       // Blinn-Phong
       // Note: preferably, H should be computed in the vertex shader
       vec3 H = (viewdir + lightdir) * vec3(0.5);
       return pow(max(tangDot(H, norm), 0.0), shiny);
    #elif defined(WARDISO)
        // Isotropic Ward
        vec3 halfVec = normalize(viewdir + lightdir);
        float NdotH  = max(0.001, tangDot(norm, halfVec));
        float NdotV  = max(0.001, tangDot(norm, viewdir));
        float NdotL  = max(0.001, tangDot(norm, lightdir));
        float a      = tan(acos(NdotH));
        float p      = max(shiny/128.0, 0.001);
        return NdotL * (1.0 / (4.0*3.14159265*p*p)) * (exp(-(a*a)/(p*p)) / (sqrt(NdotV * NdotL)));
    #else
       // Standard Phong
       vec3 R = reflect(-lightdir, norm);
       return pow(max(tangDot(R, viewdir), 0.0), shiny);
    #endif
}

vec2 computeLighting(in vec3 wvNorm, in vec3 wvViewDir, in vec3 wvLightDir){
   float diffuseFactor = lightComputeDiffuse(wvNorm, wvLightDir, wvViewDir);
   float specularFactor = lightComputeSpecular(wvNorm, wvViewDir, wvLightDir, m_Shininess);

   #ifdef HQ_ATTENUATION
    float att = clamp(1.0 - g_LightPosition.w * length(lightVec), 0.0, 1.0);
   #else
    float att = vLightDir.w;
   #endif

   if (m_Shininess <= 1.0) {
       specularFactor = 0.0; // should be one instruction on most cards ..
   }

   specularFactor *= diffuseFactor;

   return vec2(diffuseFactor, specularFactor) * vec2(att);
}
#endif

vec4 getColor( in sampler2D diffuseMap, in sampler2D diffuseMapLow, in sampler2D normalMap, in vec2 tc, in float distMix, out vec3 normal ) {

    vec2 tcOffset;
    tcOffset = texture2D(m_Noise, tc * 0.01).xy * 6.0 - 3.0;
    vec4 diffuseColor = texture2D(diffuseMap, (tc + tcOffset) * 0.75);
    
    tcOffset.x = (texture2D(m_Noise, tc * 0.01).x * 0.2) - 0.1;
    tcOffset.y = (texture2D(m_Noise, tc * 0.03).x * 0.2) - 0.1;
    vec4 subColor = texture2D(diffuseMapLow, tc * 0.1 + tcOffset * 0.1);
    diffuseColor = mix(diffuseColor, subColor, distMix); 
 
    vec4 normalHeight = texture2D(normalMap, tc);
    normal = normalize((normalHeight.xyz * vec3(2.0) - vec3(1.0)));
 
    return diffuseColor;     
}


void main(){
    vec2 newTexCoord;

    /**** 
      This is taken care of by the trilinear mapping now     
    #if (defined(PARALLAXMAP) || (defined(NORMALMAP_PARALLAX) && defined(NORMALMAP))) && !defined(VERTEX_LIGHTING) 
     
       #ifdef STEEP_PARALLAX
           #ifdef NORMALMAP_PARALLAX
               //parallax map is stored in the alpha channel of the normal map         
               newTexCoord = steepParallaxOffset(m_NormalMap, vViewDir, texCoord.xz, m_ParallaxHeight);
           #else
               //parallax map is a texture
               newTexCoord = steepParallaxOffset(m_ParallaxMap, vViewDir, texCoord.xz, m_ParallaxHeight);         
           #endif
       #else
           #ifdef NORMALMAP_PARALLAX
               //parallax map is stored in the alpha channel of the normal map         
               newTexCoord = classicParallaxOffset(m_NormalMap, vViewDir, texCoord.xz, m_ParallaxHeight);
           #else
               //parallax map is a texture
               newTexCoord = classicParallaxOffset(m_ParallaxMap, vViewDir, texCoord.xz, m_ParallaxHeight);
           #endif
       #endif
    #else
       newTexCoord = texCoord.xz;    
    #endif
    
   #ifdef DIFFUSEMAP
      vec4 diffuseColor = texture2D(m_DiffuseMap, newTexCoord);
    #else
      vec4 diffuseColor = vec4(1.0);
    #endif

    float alpha = DiffuseSum.a * diffuseColor.a;
    #ifdef ALPHAMAP
       alpha = alpha * texture2D(m_AlphaMap, newTexCoord).r;
    #endif
    if(alpha < m_AlphaDiscardThreshold){
        discard;
    }
    */

    //diffuseColor.xyz = normalize(worldNormal);
    float alpha = 1.0;   

    /*****
     This is taken care of by the trilinear mapping now
    // ***********************
    // Read from textures
    // ***********************
    #if defined(NORMALMAP) && !defined(VERTEX_LIGHTING)
      vec4 normalHeight = texture2D(m_NormalMap, newTexCoord);
      vec3 normal = normalize((normalHeight.xyz * vec3(2.0) - vec3(1.0)));
      #ifdef LATC
        normal.z = sqrt(1.0 - (normal.x * normal.x) - (normal.y * normal.y));
      #endif
      //normal.y = -normal.y;
    #elif !defined(VERTEX_LIGHTING)
      vec3 normal = vNormal;
      #if !defined(LOW_QUALITY) && !defined(V_TANGENT)
         normal = normalize(normal);
      #endif
    #endif
    */
 
 
    // *************************************
    // Trinlinear Mapping
    // *************************************
     
    // We change the low res to hi res based on distance
    float lowMix = z / m_LowResDistance;
    lowMix = clamp(lowMix, 0.5, 1.0);

    // Collect the basic axis textures for x, y, and z 
    vec3 normalX;
    vec3 normalY;
    vec3 normalZ;
    vec3 normalTop;
    
    vec4 xColor = getColor(m_DiffuseMapX, m_DiffuseMapX, m_NormalMapX, texCoord.zy, lowMix, normalX);    
    vec4 yColor = getColor(m_DiffuseMapY, m_DiffuseMapY, m_NormalMapY, texCoord.xz, lowMix, normalY);    
    vec4 zColor = getColor(m_DiffuseMapZ, m_DiffuseMapZ, m_NormalMapZ, texCoord.xy, lowMix, normalZ);    

    // Turn the normal into the blending ratios.    
    float up = worldNormal.y;
    float offset = texture2D(m_Noise, texCoord.xz).x;
    
    // We bias the threshold a bit based on a noise value.
    float threshold = up + (offset * 0.1 - 0.05);
    
    // Now bias the y blend factor by the threshold if it exceeds
    // 45 degrees, basically.  The noise above will give us some
    // nice rough edges.
    vec3 blend = abs(normalize(worldNormal));
    if( threshold > 0.707 ) {
        blend.y += 10.0 * offset;         
    }
    
    // "normalize" it once with the initial bias in place.
    // blend should always add to 1.0
    blend /= (blend.x + blend.y + blend.z); 
 
    // The top will use a different texture based on the threshold so far    
    vec4 topColor = getColor(m_DiffuseMap, m_DiffuseMapLow, m_NormalMapY, texCoord.xz, lowMix, normalTop);

    // Mix a border into the topColor based on an area around the threshold
    float edge1 = smoothstep(0.5, 0.72, threshold);
    float edge2 = 1.0 - smoothstep(0.72, 0.75, threshold);
    topColor = mix(topColor, topColor * vec4(0.6, 0.5, 0.5, 1.0), edge1 * edge2);
  
    // Top will be 1.0 if normal is up, 0 otherwise
    float top = step(worldNormal.y, 0.0);
    
    // Select the top or the bottom texture based on sign of y
    // ...and darken the bottom texture while we are at it   
    yColor = topColor * (1.0 - top) + yColor * top * 0.5;   
 
    // Bias the y blend value based on the up-ness and
    // the edges  
    blend.y *= max(top, edge1 * edge1);
        
    // It is important that x, y, z add up to just 1.0 and only 1.0 
    blend /= (blend.x + blend.y + blend.z); 

    // If we are very close to the grass edge then also darken the
    // other two axes just a bit. 
    float darken = min(1.0, 0.9 + (1.0 - edge1) * 0.1);
       
    vec4 diffuseColor = xColor * blend.x * darken 
                        + yColor * blend.y
                        + zColor * blend.z * darken;

    // Move the normal map normals into their respective axis world space
    // a bit arbitrarily.                        
    normalX = vec3(0.0, -normalX.y, normalX.x);
    normalY = vec3(normalY.x, 0.0, normalY.y);
    normalZ = vec3(normalZ.x, -normalZ.y, 0.0);  
 
    // Mix the normal map normals together based on blend                        
    vec3 bumpNormal = normalX * blend.x 
                        + normalY * blend.y
                        + normalZ * blend.z;
    vec3 normal = normalize(bumpNormal);                                                
 
    normal = normalize(vNormal + g_NormalMatrix * bumpNormal); 

    // Moved this to after trilinear mapping is performed so that the color
    // will be accurate
    #ifndef VERTEX_LIGHTING
        float spotFallOff = 1.0;

        #if __VERSION__ >= 110
          // allow use of control flow
          if(g_LightDirection.w != 0.0){
        #endif

          vec3 L       = normalize(lightVec.xyz);
          vec3 spotdir = normalize(g_LightDirection.xyz);
          float curAngleCos = dot(-L, spotdir);             
          float innerAngleCos = floor(g_LightDirection.w) * 0.001;
          float outerAngleCos = fract(g_LightDirection.w);
          float innerMinusOuter = innerAngleCos - outerAngleCos;
          spotFallOff = (curAngleCos - outerAngleCos) / innerMinusOuter;

          #if __VERSION__ >= 110
              if(spotFallOff <= 0.0){
                  gl_FragColor.rgb = AmbientSum * diffuseColor.rgb;
                  gl_FragColor.a   = alpha;
                  return;
              }else{
                  spotFallOff = clamp(spotFallOff, 0.0, 1.0);
              }
             }
          #else
             spotFallOff = clamp(spotFallOff, step(g_LightDirection.w, 0.001), 1.0);
          #endif
    #endif
 
    #ifdef SPECULARMAP
      vec4 specularColor = texture2D(m_SpecularMap, newTexCoord);
    #else
      vec4 specularColor = vec4(1.0);
    #endif

    #ifdef LIGHTMAP
       vec3 lightMapColor;
       #ifdef SEPARATE_TEXCOORD
          lightMapColor = texture2D(m_LightMap, texCoord2).rgb;
       #else
          lightMapColor = texture2D(m_LightMap, texCoord.xz).rgb;
       #endif
       specularColor.rgb *= lightMapColor;
       diffuseColor.rgb  *= lightMapColor;
    #endif

    #ifdef VERTEX_LIGHTING
       vec2 light = vertexLightValues.xy;
       #ifdef COLORRAMP
           light.x = texture2D(m_ColorRamp, vec2(light.x, 0.0)).r;
           light.y = texture2D(m_ColorRamp, vec2(light.y, 0.0)).r;
       #endif

        #ifndef USE_SCATTERING
            gl_FragColor.rgb =  AmbientSum     * diffuseColor.rgb + 
                                DiffuseSum.rgb * diffuseColor.rgb  * vec3(light.x) +
                                SpecularSum    * specularColor.rgb * vec3(light.y);
        #else
            vec3 color = AmbientSum     * diffuseColor.rgb + 
                         DiffuseSum.rgb * diffuseColor.rgb  * vec3(light.x) +
                         SpecularSum    * specularColor.rgb * vec3(light.y);
            gl_FragColor.rgb =  calculateGroundColor(vec4(color, 1.0)).rgb;
        #endif            
    #else
       vec4 lightDir = vLightDir;
       lightDir.xyz = normalize(lightDir.xyz);
       vec3 viewDir = normalize(vViewDir);

       vec2   light = computeLighting(normal, viewDir, lightDir.xyz) * spotFallOff;
       #ifdef COLORRAMP
           diffuseColor.rgb  *= texture2D(m_ColorRamp, vec2(light.x, 0.0)).rgb;
           specularColor.rgb *= texture2D(m_ColorRamp, vec2(light.y, 0.0)).rgb;
       #endif

       // Workaround, since it is not possible to modify varying variables
       vec4 SpecularSum2 = vec4(SpecularSum, 1.0);
       #ifdef USE_REFLECTION
            vec4 refColor = Optics_GetEnvColor(m_EnvMap, refVec.xyz);

            // Interpolate light specularity toward reflection color
            // Multiply result by specular map
            specularColor = mix(SpecularSum2 * light.y, refColor, refVec.w) * specularColor;

            SpecularSum2 = vec4(1.0);
            light.y = 1.0;
       #endif

        #ifndef USE_SCATTERING
            gl_FragColor.rgb =  AmbientSum     * diffuseColor.rgb + 
                                DiffuseSum.rgb * diffuseColor.rgb  * vec3(light.x) +
                                SpecularSum    * specularColor.rgb * vec3(light.y);
        #else
            vec3 color = AmbientSum     * diffuseColor.rgb + 
                         DiffuseSum.rgb * diffuseColor.rgb  * vec3(light.x) +
                         SpecularSum    * specularColor.rgb * vec3(light.y);
            gl_FragColor.rgb =  calculateGroundColor(vec4(color, 1.0)).rgb;
        #endif            
    #endif
    gl_FragColor.a = alpha;
}
