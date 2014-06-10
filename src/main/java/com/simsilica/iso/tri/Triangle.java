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

package com.simsilica.iso.tri;

import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;


/**
 *
 *
 *  @author    Paul Speed
 */
public class Triangle  implements Cloneable {
    
    public Vector3f[] verts = { new Vector3f(), new Vector3f(), new Vector3f() };
    public Vector3f[] norms = { new Vector3f(), new Vector3f(), new Vector3f() };
    public Vector2f[] texes = { new Vector2f(), new Vector2f(), new Vector2f() };
    public int[] indexes = new int[3];
 
    public Triangle() {
    }
    
    @Override
    public Triangle clone() {
        Triangle result = new Triangle();
        for( int v = 0; v < 3; v++ ) {
            result.verts[v].set(verts[v]);
            result.norms[v].set(norms[v]);
            result.texes[v].set(texes[v]);
            result.indexes[v] = indexes[v];
        }
        return result;
    }
    
    @Override
    public String toString() {
        return "Triangle[" + verts[0] + ", " + verts[1] + ", " + verts[2] + "]";
    }
}
