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

package com.simsilica.iso.volume;

import com.jme3.math.Vector3f;
import com.simsilica.iso.DensityVolume;


/**
 *  Presents a resampled view of a delegate volume where the
 *  x, y, and z space may be rescaled.
 *
 *  @author    Paul Speed
 */
public class ResamplingVolume implements DensityVolume {
 
    private Vector3f offset;
    private Vector3f scale;
    private DensityVolume delegate;
    
    public ResamplingVolume( Vector3f scale, DensityVolume delegate ) {
        this(scale, Vector3f.ZERO, delegate);
    }

    public ResamplingVolume( Vector3f scale, Vector3f preScaleOffset, DensityVolume delegate ) {
        this.scale = scale.clone();
        this.offset = preScaleOffset.clone();
        this.delegate = delegate;
    }
 
    public float getDensity( int x, int y, int z ) {
        return getDensity((float)x, (float)y, (float)z);   
    }
       
    public float getDensity( float x, float y, float z ) {
        x *= scale.x;
        y *= scale.y;
        z *= scale.z;
        x += offset.x;
        y += offset.y;
        z += offset.z;        
        return delegate.getDensity(x, y, z);
    }
    
    public Vector3f getFieldDirection( float x, float y, float z, Vector3f target ) {
        float xt = x * scale.x;
        float yt = y * scale.y;
        float zt = z * scale.z;
        xt += offset.x;
        yt += offset.y;
        zt += offset.z;
        Vector3f dir = delegate.getFieldDirection(xt, yt, zt, target);
        return dir;
    }
 
    @Override   
    public String toString() {
        return "ResamplingVolume[" + scale + ", " + offset + ", " + delegate + "]";
    }
}
