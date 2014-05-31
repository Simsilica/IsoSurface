/*
 * $Id$
 * 
 * Copyright (c) 2014, Simsilica, LLC
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions 
 * are met:
 * 
 * 1. Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in 
 *    the documentation and/or other materials provided with the 
 *    distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its 
 *    contributors may be used to endorse or promote products derived 
 *    from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED 
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package com.simsilica.iso.fractal;

import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.simsilica.iso.DensityVolume;


/**
 *  DensityVolume implementation returning densities based on the
 *  fractal defined in the GPU Gems book.  Some adaptations as necessary
 *  to convert from a shader-based algorithm to a Java-based one.
 *
 *  @author    Paul Speed
 */
public class GemsFractalDensityVolume implements DensityVolume {

    private PerlinNoise noise = new PerlinNoise(0);
    private PerlinNoise noise2 = new PerlinNoise(1);
    private PerlinNoise noise3 = new PerlinNoise(2);

    private Quaternion octaveMat0;
    private Quaternion octaveMat1;
    private Quaternion octaveMat2;
    private Quaternion octaveMat3;
    private Quaternion octaveMat4;
    private Quaternion octaveMat5;
    private Quaternion octaveMat6;
    private Quaternion octaveMat7;


    public GemsFractalDensityVolume() {
        octaveMat0 = new Quaternion();
        octaveMat1 = new Quaternion().fromAngles(0.1f, 0.01f, -0.1f);
        octaveMat2 = new Quaternion().fromAngles(0.2f, 0.02f, -0.2f);
        octaveMat3 = new Quaternion().fromAngles(0.3f, 0.03f, -0.3f);
        octaveMat4 = new Quaternion().fromAngles(0.4f, 0.04f, -0.4f);
        octaveMat5 = new Quaternion().fromAngles(0.5f, 0.05f, -0.5f);
        octaveMat6 = new Quaternion().fromAngles(0.6f, 0.06f, -0.6f);
        octaveMat7 = new Quaternion().fromAngles(0.7f, 0.07f, -0.7f);
    }

    private void warp( Vector3f loc, double frequency, double scale )
    {
        double x = noise.getNoise(loc.x * frequency, loc.y * frequency, loc.z * frequency);
        double y = noise2.getNoise(loc.x * frequency, loc.y * frequency, loc.z * frequency);
        double z = noise3.getNoise(loc.x * frequency, loc.y * frequency, loc.z * frequency);
 
        loc.x = (float)(loc.x + x * scale);
        loc.y = (float)(loc.y + y * scale);
        loc.z = (float)(loc.z + z * scale);
    }

    private double getNoise( Vector3f loc, double scale ) 
    {
        //scale *= 0.5f;
        //scale *= 4; // 16;
        //scale *= 10; //16;
        scale *= 16;
        return noise.getNoise(loc.x * scale, loc.y * scale, loc.z * scale); // * 2 - 1;
    }

    protected float density( Vector3f loc ) {
    
        double density = -loc.y;
    
        // Warp the location
        warp(loc, 0.004, 8);
        //warp(loc, 0.001, 40);
    
        Vector3f c0 = octaveMat0.mult(loc);
        Vector3f c1 = octaveMat1.mult(loc);
        Vector3f c2 = octaveMat2.mult(loc);
        Vector3f c3 = octaveMat3.mult(loc);
        Vector3f c4 = octaveMat4.mult(loc);
        Vector3f c5 = octaveMat5.mult(loc);
        Vector3f c6 = octaveMat6.mult(loc);
        Vector3f c7 = octaveMat7.mult(loc);
    
        density += getNoise(c0, 0.1600*1.021) * 0.32*1.16;        
        density += getNoise(c1, 0.0800*0.985) * 0.64*1.12;        
        density += getNoise(c2, 0.0400*1.051) * 1.28*1.08;        
        density += getNoise(c3, 0.0200*1.020) * 2.56*1.04;        
        density += getNoise(c4, 0.0100*0.968) * 5;        
        density += getNoise(c5, 0.0050*0.994) * 10;        
        density += getNoise(c6, 0.0025*1.045) * 20*0.9;        
        density += getNoise(c7, 0.0012*0.972) * 40*0.8;        
        
        return (float)density;
    }

    public float getDensity( int x, int y, int z ) {
        return density(new Vector3f(x, y, z));
    }

    public float getDensity( float x, float y, float z ) {
        return density(new Vector3f(x, y, z));
    }

    public Vector3f getFieldDirection(float x, float y, float z, Vector3f target) {
    
        float d = 1f; 
    
        double nx = getDensity(x + d, y, z)
                    - getDensity(x - d, y, z);
        double ny = getDensity(x, y + d, z)
                    - getDensity(x, y - d, z);
        double nz = getDensity(x, y, z + d)
                    - getDensity(x, y, z - d);
 
        if( target == null ) {
            target = new Vector3f((float)-nx, (float)-ny, (float)-nz).normalizeLocal();
        } else {
            target.set((float)-nx, (float)-ny, (float)-nz);
            target.normalizeLocal();
        }
    
        return target;
    }
}

