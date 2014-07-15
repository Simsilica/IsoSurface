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

package com.simsilica.iso.util;

import com.jme3.math.Matrix3f;
import com.jme3.math.Matrix4f;
import com.jme3.math.Quaternion;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;


/**
 *
 *
 *  @author    Paul Speed
 */
public class MatrixUtils {

    public static FloatBuffer createMatrixBuffer( int count ) {
        return BufferUtils.createFloatBuffer(count * 16);       
    }
    
    public static FloatBuffer createMatrixBuffer( Matrix4f[] mats ) {
        FloatBuffer result = createMatrixBuffer(mats.length);
 
        Matrix3f rotMat = new Matrix3f();
        Quaternion rot = new Quaternion();
               
        for( Matrix4f mat : mats ) {
            mat.toRotationMatrix(rotMat);
            rot.fromRotationMatrix(rotMat);
            
            result.put(mat.m00);
            result.put(mat.m10);
            result.put(mat.m20);
            result.put(rot.getX());
            result.put(mat.m01);
            result.put(mat.m11);
            result.put(mat.m21);
            result.put(rot.getY());
            result.put(mat.m02);
            result.put(mat.m12);
            result.put(mat.m22);
            result.put(rot.getZ());
            result.put(mat.m03);
            result.put(mat.m13);
            result.put(mat.m23);
            result.put(rot.getW());
        }
        
        result.flip();       
        return result;
    }
}

