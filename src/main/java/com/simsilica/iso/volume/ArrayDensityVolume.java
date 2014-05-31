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
import java.util.Arrays;


/**
 *  A DensityVolume implementation backed by a fixed size
 *  array of values.  These may be set directly or extracted
 *  from an existing DensityField as required.  Intercell
 *  sampling is done using trilinear interpolation.
 *
 *  @author    Paul Speed
 */
public class ArrayDensityVolume implements DensityVolume {
 
    private int cx, cy, cz, cLayer;
    private float[] array;
 
    public ArrayDensityVolume( int width, int height, int depth ) {
        this.array = new float[width * height * depth];
        this.cx = width;
        this.cy = height;
        this.cz = depth;
        this.cLayer = cx * cy;
    }
 
    public static ArrayDensityVolume create( float[][][] source, 
                                             int width, int height, int depth ) {
        ArrayDensityVolume result = new ArrayDensityVolume(width, height, depth);
        int index = 0;
        float[] target = result.array;
        for( int z = 0; z < depth; z++ ) {
            for( int y = 0; y < height; y++ ) {
                for( int x = 0; x < width; x++ ) {
                    target[index++] = source[x][y][z];
                } 
            }
        }
        return result;                                               
    }
 
    public static ArrayDensityVolume extractVolume( DensityVolume source, 
                                                    int xBase, int yBase, int zBase, 
                                                    int width, int height, int depth ) {
        ArrayDensityVolume result = new ArrayDensityVolume(width, height, depth);
        int index = 0;
        float[] target = result.array;
        for( int z = 0; z < depth; z++ ) {
            for( int y = 0; y < height; y++ ) {
                for( int x = 0; x < width; x++ ) {
                    target[index++] = source.getDensity(xBase+x, yBase+y, zBase+z);
                } 
            }
        }
        return result;                                               
    }                                               
 
    public void extract( DensityVolume source, int xBase, int yBase, int zBase ) {
        int index = 0;
        for( int z = 0; z < cz; z++ ) {
            for( int y = 0; y < cy; y++ ) {
                for( int x = 0; x < cx; x++ ) {
                    array[index++] = source.getDensity(xBase+x, yBase+y, zBase+z);
                } 
            }
        }
    }
 
    public void clear() {
        Arrays.fill(array, -1);
    }
 
    private int index( int x, int y, int z ) {
        /*if( x < 0 || y < 0 || z < 0 || x >= cx || y >= cy || z >= cz ) {
            throw new IndexOutOfBoundsException( "(" + x + ", " + y + ", " + z + ") in size:" + cx + ", " + cy + ", " + cz );
        }*/
        return z * cLayer + cx * y + x; 
    }
 
    public void setDensity( int x, int y, int z, float d ) {
        array[index(x, y, z)] = d;
    }
 
    public float getDensity( int x, int y, int z ) {
        return array[index(x, y, z)];
    }

    private double trilinear( float x, float y, float z ) {
        int xBase = (int)Math.floor(x);
        int yBase = (int)Math.floor(y);
        int zBase = (int)Math.floor(z);
        int xTop = (int)Math.ceil(x);
        int yTop = (int)Math.ceil(y);
        int zTop = (int)Math.ceil(z);

        double c000 = array[index(xBase, yBase, zBase)];
        double c100 = array[index(xTop, yBase, zBase)];
        double c101 = array[index(xTop, yBase, zTop)];
        double c001 = array[index(xBase, yBase, zTop)]; 
        double c010 = array[index(xBase, yTop, zBase)];
        double c110 = array[index(xTop, yTop, zBase)];
        double c111 = array[index(xTop, yTop, zTop)];
        double c011 = array[index(xBase, yTop, zTop)]; 
        
        double xPart = x - xBase;   
        double c00 = c000 + (c100 - c000) * xPart;
        double c01 = c001 + (c101 - c001) * xPart;
        
        double c10 = c010 + (c110 - c010) * xPart;
        double c11 = c011 + (c111 - c011) * xPart;
 
        double yPart = y - yBase;
        double c0 = c00 + (c10 - c00) * yPart;              
        double c1 = c11 + (c11 - c01) * yPart;              
 
        double zPart = z - zBase;
        double c = c0 + (c1 - c0) * zPart;
 
        return c;              
    } 
 
       
    public float getDensity( float x, float y, float z ) {
        return (float)trilinear(x, y, z);
    }
    
    public Vector3f getFieldDirection( float x, float y, float z, Vector3f target ) {
    
        float d = 1f; 

        double nx = trilinear(x + d, y, z)
                    - trilinear(x - d, y, z);
        double ny = trilinear(x, y + d, z)
                    - trilinear(x, y - d, z);
        double nz = trilinear(x, y, z + d)
                    - trilinear(x, y, z - d);
 
        if( target == null ) {
            target = new Vector3f((float)-nx, (float)-ny, (float)-nz).normalizeLocal();
        } else {
            target.set((float)-nx, (float)-ny, (float)-nz);
            target.normalizeLocal();
        }
    
        return target;
    }
}
