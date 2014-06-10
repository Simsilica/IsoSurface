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
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.VertexBuffer.Type;
import com.jme3.scene.mesh.IndexBuffer;
import java.nio.FloatBuffer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


/**
 *
 *
 *  @author    Paul Speed
 */
public class TriangleUtils {

    static Logger log = LoggerFactory.getLogger(TriangleUtils.class);

    public static int processTriangles( Mesh mesh, TriangleProcessor proc ) {
        if( log.isTraceEnabled() ) {
            log.trace("processTriangles(" + mesh + ", " + proc + ")");
            log.trace("  verts:" + mesh.getVertexCount() + "  mode:" + mesh.getMode());
        }                
        if( mesh.getVertexCount() == 0 ) {
            return 0;
        }
        switch( mesh.getMode() ) {
            case LineLoop:
            case Lines:
            case LineStrip:
            case Points:
                return 0; // no triangles in those
            case Hybrid:
            case TriangleFan:
            case TriangleStrip:
                throw new UnsupportedOperationException("Mesh type not yet supported:" + mesh.getMode());
            case Triangles:
                if( mesh.getIndexBuffer() != null ) {
                    return doIndexedTriangles(mesh, proc);
                } else {
                    return doTriangles(mesh, proc);
                }
            default:
                return 0;                
        }       
    }  
    
    public static int processTriangles( Spatial s, TriangleProcessor proc ) {
        if( log.isTraceEnabled() ) {
            log.trace("processTriangles(Spatial:" + s + ", proc:" + proc + ")");
        }        
        int count = 0;    
        if( s instanceof Node ) {
            for( Spatial child : ((Node)s).getChildren() ) {
                count += processTriangles(child, proc);
            }
        } else if( s instanceof Geometry ) {
            count += processTriangles(((Geometry)s).getMesh(), proc);
        } else {
            throw new UnsupportedOperationException("Unsupported spatial type:" + s);
        }
        return count;
    }
    
    protected static int doTriangles( Mesh mesh, TriangleProcessor proc ) {
        throw new UnsupportedOperationException("Non-indexed triangles not yet supported.");
    }
     
    protected static int doIndexedTriangles( Mesh mesh, TriangleProcessor proc ) {
        Triangle tri = new Triangle();
 
        VertexBuffer vbPos = mesh.getBuffer(Type.Position);
        int posSize = vbPos.getNumComponents();        
        VertexBuffer vbNorms = mesh.getBuffer(Type.Normal);
        int normSize = 0;        
        VertexBuffer vbTexes = mesh.getBuffer(Type.TexCoord);
        int texesSize = 0;        
                
        FloatBuffer pos = ((FloatBuffer)vbPos.getData()).duplicate();
        FloatBuffer norms = null;
        if( vbNorms != null ) {
            norms = ((FloatBuffer)vbNorms.getData()).duplicate();
            normSize = vbNorms.getNumComponents();        
        }
        FloatBuffer texes = null;
        if( vbTexes != null ) {
            texes = ((FloatBuffer)vbTexes.getData()).duplicate();
            texesSize = vbTexes.getNumComponents();
        }
          
        IndexBuffer ib = mesh.getIndexBuffer();
        int size = ib.size();
        int triangleIndex = 0;
        for( int i = 0; i < size; ) {
            for( int v = 0; v < 3; v++ ) {
                int index = ib.get(i++);
                tri.indexes[v] = index;
                
                pos.position(index * posSize);
                Vector3f vert = tri.verts[v];
                vert.x = pos.get();   
                vert.y = pos.get();   
                vert.z = pos.get();
                if( norms != null ) {
                    norms.position(index * normSize);
                    Vector3f norm = tri.norms[v];
                    norm.x = norms.get();    
                    norm.y = norms.get();    
                    norm.z = norms.get();    
                }   
                if( texes != null ) {
                    texes.position(index * texesSize);
                    Vector2f tex = tri.texes[v];
                    tex.x = texes.get();    
                    tex.y = texes.get();    
                }   
            }
            proc.processTriangle(mesh, triangleIndex++, tri); 
        }
        return triangleIndex;    
    }    
}
