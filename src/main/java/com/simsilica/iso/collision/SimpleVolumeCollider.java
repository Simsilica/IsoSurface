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

package com.simsilica.iso.collision;

import com.jme3.math.Vector3f;
import com.simsilica.iso.DensityVolume;


/**
 *
 *
 *  @author    Paul Speed
 */
public class SimpleVolumeCollider implements Collider {
    private DensityVolume volume;
     
    public SimpleVolumeCollider( DensityVolume volume ) {
        this.volume = volume;
    }

    @Override
    public Contact getContact( Vector3f loc, float radius ) {
    
        // Check the density in the direction of the surface
        // to the best of our ability to estimate
        Vector3f dir = volume.getFieldDirection(1 + loc.x, 1 + loc.y, 1 + loc.z, null);
        float density = volume.getDensity(1 + loc.x - dir.x * radius, 
                                          1 + loc.y - dir.y * radius,
                                          1 + loc.z - dir.z * radius);
    
        if( density < 0 ) {
            // We are not in contact
            return null;
        }
        
        // Else try to calculate the intersection
        float center = volume.getDensity(1 + loc.x, 1 + loc.y, 1 + loc.z);
        
        float pen;
        Vector3f cp;
        
        if( center > 0 ) {
            // we are deep in it... try to see if we can guess a way 
            //  out
            float outside = volume.getDensity(1 + loc.x + dir.x, 
                                              1 + loc.y + dir.y,
                                              1 + loc.z + dir.z);
            float part = Math.abs(center) / Math.abs(center - outside);
            pen = part;
            //System.out.println( "outside:" + outside + "  center:" + center + "  part:" + part + "  pen:" + pen );
             
            // Calculate a contact point
            cp = loc.add(dir.mult(radius - pen));
            //System.out.println( "loc:" + loc + " cp:" + cp );
        } else {        
            float part = Math.abs(density) / Math.abs(density - center);
            pen = radius * part;
            //System.out.println( "density:" + density + "  center:" + center + "  part:" + part + "  pen:" + pen ); 
 
            // Calculate a contact point
            cp = loc.add(dir.mult(radius - pen));
            //System.out.println( "loc:" + loc + " cp:" + cp );
        }
 
        Contact result = new Contact();
        result.contactPoint = cp;
        result.contactNormal = volume.getFieldDirection(1 + cp.x, 1 + cp.y, 1 + cp.z, null);
        result.penetration = pen;              
    
        return result;
    }    
     
}
