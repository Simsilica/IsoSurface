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

package com.simsilica.iso.plot;

import com.jme3.bounding.BoundingBox;
import com.jme3.math.Matrix4f;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.VertexBuffer.Format;
import com.jme3.scene.VertexBuffer.Type;
import com.jme3.scene.VertexBuffer.Usage;
import com.simsilica.iso.util.MatrixUtils;
import java.nio.FloatBuffer;
import java.util.List;


/**
 *  Wraps a Geometry to provide a template for
 *  instance-batching.  This could have been combined with
 *  BatchTemplate but I'm trying to isolated the JME 3.x specific
 *  dependencies from the JME 3.0 dependencies.
 *
 *  @author    Paul Speed
 */
public class InstanceTemplate {
    private Geometry sourceGeom;
    private Mesh source;
    private VertexBuffer[] templates;
    private boolean intIndex;
    
    public InstanceTemplate( Geometry sourceGeom, boolean intIndex ) {
        this.sourceGeom = sourceGeom;
        this.intIndex = intIndex;
        this.source = sourceGeom.getMesh();
    }
    
    public Geometry createInstances( List<BatchInstance> instances ) {
        if( instances.isEmpty() ) {
            return null;
        }
 
        Vector3f offset = sourceGeom.getWorldTranslation();
       
        // For instances, we just clone the original mesh and add
        // in the transform instance data
        Mesh mesh = source.clone();
 
        Vector3f min = new Vector3f(Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY);
        Vector3f max = new Vector3f(Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY);
        Matrix4f[] transforms = new Matrix4f[instances.size()];
        for( int i = 0; i < instances.size(); i++ ) {
            BatchInstance instance = instances.get(i);
                                    
            Vector3f p1 = instance.position;
            Quaternion rot = instance.rotation;
            float scale = instance.scale;
 
            // The offset needs to be projected
            Vector3f localOffset = rot.mult(offset);
            localOffset.multLocal(scale);
 
            p1 = p1.add(localOffset);
            
            min.minLocal(p1);
            max.maxLocal(p1);
            
 
            Matrix4f transform = new Matrix4f();
            transform.setRotationQuaternion(rot);
            transform.setTranslation(p1);
            transform.setScale(scale, scale, scale);
            
            transforms[i] = transform;
        }

        // Note: this doesn't calculate it correctly right now
        //       so we'll do it manually and fudge it a little.                    
        //mesh.updateBound();
        min.subtractLocal(3, 1, 3);
        max.addLocal(3, 8, 3);
        BoundingBox bounds = new BoundingBox(min, max);
        
        // Create the transform buffer
        FloatBuffer xb = MatrixUtils.createMatrixBuffer(transforms);

        VertexBuffer vb = new VertexBuffer(Type.InstanceData);
        vb.setInstanceSpan(1);
        vb.setupData(Usage.Stream, 16, Format.Float, xb);
        
        return createInstances(vb, bounds); 
    }
 
    public Geometry createInstances( VertexBuffer transforms, BoundingBox bounds ) {
    
        // For instances, we just clone the original mesh and add
        // in the transform instance data
        Mesh mesh = source.clone();    
        mesh.setBuffer(transforms);
        mesh.setBound(bounds);
        
        Geometry result = new Geometry("batch:" + sourceGeom.getName(), mesh);
        result.setMaterial(sourceGeom.getMaterial());
        result.setQueueBucket(sourceGeom.getQueueBucket());        
        return result;                                          
    }


    
}
