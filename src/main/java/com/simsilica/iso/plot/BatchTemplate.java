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

import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.VertexBuffer.Format;
import com.jme3.scene.VertexBuffer.Type;
import com.jme3.util.BufferUtils;
import java.nio.Buffer;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.nio.ShortBuffer;
import java.util.List;
import java.util.Random;


/**
 *  Wraps a Geometry to provide a template for
 *  batching.  This is different than standard JME
 *  batching because it will randomize rotation, etc.
 *  based on how it is setup.  Also it will (eventually)
 *  support instancing.
 *
 *  @author    Paul Speed
 */
public class BatchTemplate {
    private Geometry sourceGeom;
    private Mesh source;
    private VertexBuffer[] templates;
    private boolean intIndex;
    
    public BatchTemplate( Geometry sourceGeom, boolean intIndex ) {
        this.sourceGeom = sourceGeom;
        this.intIndex = intIndex;
        this.source = sourceGeom.getMesh();
        
        // No reason to do this since we won't be using those
        // because they are slower
        //setupSource();
    }
    
    protected final void setupSource() {
        templates = source.getBufferList().getArray();
        
        if( intIndex ) {
            // Make sure any ShortBuffer index is converted
            VertexBuffer ib = source.getBuffer(Type.Index);
            if( ib.getFormat() == Format.Short || ib.getFormat() == Format.UnsignedShort ) {
                ShortBuffer sb = (ShortBuffer)ib.getDataReadOnly();
                IntBuffer target = (IntBuffer)VertexBuffer.createBuffer(Format.UnsignedInt, ib.getNumComponents(), 
                                                                        ib.getNumElements());
                int size = ib.getNumComponents() * ib.getNumElements();
                for( int i = 0; i < size; i++ ) {
                    int v = sb.get();
                    target.put(v);   
                }
                
                for( int i = 0; i < templates.length; i++ ) {
                    if( templates[i] == ib ) {
                        templates[i] = new VertexBuffer(Type.Index);
                        templates[i].setupData(ib.getUsage(), ib.getNumComponents(), Format.UnsignedInt, target);
                    }
                }                                                               
            }
        }
    }
 
    protected BufferTarget createTarget( VertexBuffer vb, int instanceCount, ElementFunction f ) {
    
        switch( vb.getFormat() ) {
            case Float:
                return new FloatTarget(vb, instanceCount, f);
            case Int: 
            case UnsignedInt: 
                return new IntTarget(vb, instanceCount, f);
            case Short:
            case UnsignedShort:
                return new ShortTarget(vb, instanceCount, f);
            default:
                throw new UnsupportedOperationException("Type not yet supported:" + vb.getFormat());
        }
    }
    
    protected BufferTarget createTarget( VertexBuffer vb, int instanceCount ) {        
 
        switch( vb.getBufferType() ) {
            case Index:
                return createTarget(vb, instanceCount, new IndexFunction(source.getVertexCount()));
            case Normal:
            case Tangent:
            case Binormal:
                return createTarget(vb, instanceCount, new RotationFunction());
            case Position:
                return createTarget(vb, instanceCount, new TransformFunction());
            default:
                return createTarget(vb, instanceCount, null);
        }
    }

    public Geometry createBatch( List<BatchInstance> instances ) {
        
        Vector3f offset = sourceGeom.getWorldTranslation();
        
        int templateVertexCount = source.getVertexCount();
        
        FloatBuffer tPos = source.getFloatBuffer(Type.Position).duplicate();
        FloatBuffer tNorm = source.getFloatBuffer(Type.Normal);
        if( tNorm != null ) {
            tNorm = tNorm.duplicate();
        }
        FloatBuffer tTex = source.getFloatBuffer(Type.TexCoord).duplicate();
        FloatBuffer tTan = source.getFloatBuffer(Type.Tangent);
        if( tTan != null ) {
            tTan = tTan.duplicate();
        }
        FloatBuffer tSize = source.getFloatBuffer(Type.Size);
        if( tSize != null ) {
            tSize = tSize.duplicate();
        }
        ShortBuffer tIndex = source.getShortBuffer(Type.Index).duplicate();
         
        int texComponents = source.getBuffer(Type.TexCoord).getNumComponents();
                  
        int triCount = instances.size() * source.getTriangleCount();
        FloatBuffer pb = BufferUtils.createVector3Buffer(triCount * 3);        
        FloatBuffer nb = null;
        if( tNorm != null ) {
            nb = BufferUtils.createVector3Buffer(triCount * 3);
        }
        FloatBuffer tb = BufferUtils.createVector2Buffer(triCount * texComponents);
        FloatBuffer tanb = null;
        if( tTan != null ) {
            tanb = BufferUtils.createVector2Buffer(triCount * 3);
        }
        FloatBuffer sizeb = null;
        if( tSize != null ) {
            sizeb = BufferUtils.createFloatBuffer(triCount * 3);
        }
        ShortBuffer ibShort = null;
        IntBuffer ibInt = null;
        if( triCount * 3 <= 0xffff ) {
            ibShort = (ShortBuffer)VertexBuffer.createBuffer(Format.UnsignedShort, 3, triCount);
        } else {       
            ibInt = (IntBuffer)VertexBuffer.createBuffer(Format.UnsignedInt, 3, triCount);
        }       
 
        Vector3f point = new Vector3f();
 
        // We're making a line mesh... so just render 
        // the point and a point + normal
        int lastIndex = 0;
        for( BatchInstance instance : instances ) { 
            Vector3f p1 = instance.position;
            Quaternion rot = instance.rotation;
            float scale = instance.scale;
            
            tPos.rewind();
            if( tNorm != null ) {
                tNorm.rewind();
            }
            tTex.rewind();
            if( tTan != null ) {
                tTan.rewind();
            }
            if( tSize != null ) {
                tSize.rewind();
            }
  
            for( int v = 0; v < templateVertexCount; v++ ) {
                point.set(tPos.get(), tPos.get(), tPos.get());
                rot.mult(point, point);
 
                pb.put((point.x * scale) + p1.x + offset.x); 
                pb.put((point.y * scale) + p1.y + offset.y); 
                pb.put((point.z * scale) + p1.z + offset.z);
             
                if( nb != null ) {
                    point.set(tNorm.get(), tNorm.get(), tNorm.get());
                    rot.mult(point, point);
                
                    nb.put(point.x); 
                    nb.put(point.y); 
                    nb.put(point.z);
                }
 
                for( int t = 0; t < texComponents; t++ ) {               
                    tb.put(tTex.get()); 
                } 
 
                if( tanb != null ) {               
                    point.set(tTan.get(), tTan.get(), tTan.get());
                    rot.mult(point, point);
                    tanb.put(point.x); 
                    tanb.put(point.y);
                    tanb.put(point.z);
                }
                
                if( sizeb != null ) {
                    sizeb.put(tSize.get());
                }                                
            }
            
            tIndex.rewind();
         
            if( ibShort != null ) {               
                for( int t = 0; t < source.getTriangleCount(); t++ ) {
                    ibShort.put((short)(tIndex.get() + lastIndex));
                    ibShort.put((short)(tIndex.get() + lastIndex));
                    ibShort.put((short)(tIndex.get() + lastIndex));
                }
            } else if( ibInt != null ) {
                for( int t = 0; t < source.getTriangleCount(); t++ ) {
                    ibInt.put(tIndex.get() + lastIndex);
                    ibInt.put(tIndex.get() + lastIndex);
                    ibInt.put(tIndex.get() + lastIndex);
                }
            }
            lastIndex += templateVertexCount;  
        }
 
        Mesh batch = new Mesh();
        batch.setBuffer(Type.Position, 3, pb);
        if( nb != null ) {
            batch.setBuffer(Type.Normal, 3, nb);
        }
        batch.setBuffer(Type.TexCoord, texComponents, tb);
        if( tanb != null ) {
            batch.setBuffer(Type.Tangent, 3, tanb);
        }
        if( sizeb != null ) {
            batch.setBuffer(Type.Size, 1, sizeb);
        }
        if( ibShort != null ) {
            batch.setBuffer(Type.Index, 3, ibShort);
        } else if( ibInt != null ) {
            batch.setBuffer(Type.Index, 3, ibInt);
        }
        batch.updateBound();
 
        Geometry result = new Geometry("batch:" + sourceGeom.getName(), batch);
        result.setMaterial(sourceGeom.getMaterial());        
        return result;                                          
    }
 
    // This method is more elegant and will handle any mesh...
    // but it's also many many times slower.   
    public Geometry createBatch2( List<Vector3f> positions, List<Vector3f> upVectors,
                                  Random random ) {
 
        int instanceCount = positions.size();        
        BufferTarget[] targets = new BufferTarget[templates.length];
        
        for( int i = 0; i < targets.length; i++ ) {
            targets[i] = createTarget(templates[i], instanceCount);
        }
 
        Vector3f offset = sourceGeom.getWorldTranslation();
        
        Quaternion rot = new Quaternion();
        Quaternion upRot = new Quaternion();
        
        for( int i = 0; i < instanceCount; i++ ) {
            Vector3f pos = positions.get(i).add(offset);
            Vector3f up = Vector3f.UNIT_Y;
            if( upVectors != null ) {
                up = upVectors.get(i);
            }
            
            rot.fromAngles(0, FastMath.TWO_PI * random.nextFloat(), 0); 
 
            // Make the quaternion's "up" be the normal provided
            float angle = Vector3f.UNIT_Y.angleBetween(up);
            if( Math.abs(angle) > 0 ) {
                Vector3f axis = Vector3f.UNIT_Y.cross(up).normalizeLocal();
                upRot.fromAngleNormalAxis(angle, axis);
                upRot.mult(rot, rot);
            }
            
            for( BufferTarget t : targets ) {
                t.addInstance(i, pos, rot, 1);
            }       
        }               
 
        // Create the mesh
        Mesh mesh = new Mesh();
        for( BufferTarget t : targets ) {
            mesh.setBuffer(t.createBuffer());
        }
        mesh.updateBound();
 
        Geometry result = new Geometry("batch:" + sourceGeom.getName(), mesh);
        result.setMaterial(sourceGeom.getMaterial());
                                          
        return result;
    }                                 
 
    private interface ElementFunction<T> {
        public void apply( T element, int id, Vector3f pos, Quaternion rot, float scale );   
    }
    
    private class RotationFunction implements ElementFunction<float[]> {
        Vector3f v = new Vector3f();
        
        public void apply( float[] element, int id, Vector3f pos, Quaternion rot, float scale ) {
            v.set(element[0], element[1], element[2]);
            rot.mult(v, v);
            element[0] = v.x;
            element[1] = v.y;
            element[2] = v.z;   
        }
    }

    private class TransformFunction implements ElementFunction<float[]> {
        Vector3f v = new Vector3f();
        
        public void apply( float[] element, int id, Vector3f pos, Quaternion rot, float scale ) {
            v.set(element[0], element[1], element[2]);
            rot.mult(v, v);
            element[0] = pos.x + v.x * scale;
            element[1] = pos.y + v.y * scale;
            element[2] = pos.z + v.z * scale;   
        }
    }
    
    private class IndexFunction implements ElementFunction<int[]> {
        
        private int vertexCount;
        
        public IndexFunction( int vertexCount ) {
            this.vertexCount = vertexCount;
        }
 
        public void apply( int[] element, int id, Vector3f pos, Quaternion rot, float scale ) {
            for( int i = 0; i < element.length; i++ ) {
                element[i] += id * vertexCount;
            }           
        }
    }
    
    private abstract class BufferTarget<T> {
        VertexBuffer source;
        int instanceCount;
        int elementCount;
        ElementFunction<T> function;
        
        public BufferTarget( VertexBuffer source, int instanceCount, ElementFunction<T> f ) {
            this.source = source;
            this.instanceCount = instanceCount;
            this.elementCount = source.getNumElements();
            this.function = f;
        }
        
        public abstract void addInstance( int id, Vector3f pos, Quaternion rot, float scale );
        
        protected abstract Buffer getTarget() ;
                
        public VertexBuffer createBuffer() {
            VertexBuffer result = new VertexBuffer(source.getBufferType());
            result.setupData(source.getUsage(), source.getNumComponents(), source.getFormat(), getTarget());
            return result;
        }               
    }

    private class FloatTarget extends BufferTarget<float[]> {
        FloatBuffer target;
        FloatBuffer sourceData;
        float[] buffer; 

        public FloatTarget( VertexBuffer source, int instanceCount, ElementFunction f ) {
            super(source, instanceCount, (ElementFunction<float[]>)f);                     
            this.target = (FloatBuffer)VertexBuffer.createBuffer(source.getFormat(), source.getNumComponents(), 
                                                                 instanceCount * source.getNumElements());
            this.sourceData = (FloatBuffer)source.getDataReadOnly();
            this.buffer = new float[source.getNumComponents()];
        }
        
        public final void addInstance( int id, Vector3f pos, Quaternion rot, float scale ) {
            sourceData.rewind();
            for( int i = 0; i < elementCount; i++ ) {
                sourceData.get(buffer);
                if( function != null ) {
                    function.apply(buffer, id, pos, rot, scale);
                }
                target.put(buffer);
            }   
        }

        protected Buffer getTarget() {
            return target;
        }
    }

    private class ShortTarget extends BufferTarget<short[]> {
        ShortBuffer target;
        ShortBuffer sourceData;
        short[] buffer; 

        public ShortTarget( VertexBuffer source, int instanceCount, ElementFunction f ) {
            super(source, instanceCount, (ElementFunction<short[]>)f);                     
            this.target = (ShortBuffer)VertexBuffer.createBuffer(source.getFormat(), source.getNumComponents(), 
                                                                 instanceCount * source.getNumElements());
            this.sourceData = (ShortBuffer)source.getDataReadOnly();
            this.buffer = new short[source.getNumComponents()];
        }
                                                                                                
        public final void addInstance( int id, Vector3f pos, Quaternion rot, float scale ) {
            sourceData.rewind();
            for( int i = 0; i < elementCount; i++ ) {
                sourceData.get(buffer);
                if( function != null ) {
                    function.apply(buffer, id, pos, rot, scale);
                }
                target.put(buffer);
            }   
        }
                
        protected Buffer getTarget() {
            return target;
        }
    }

    private class IntTarget extends BufferTarget<int[]> {
        IntBuffer target;
        IntBuffer sourceData;
        int[] buffer; 

        public IntTarget( VertexBuffer source, int instanceCount, ElementFunction f ) {
            super(source, instanceCount, (ElementFunction<int[]>)f);                     
            this.target = (IntBuffer)VertexBuffer.createBuffer(source.getFormat(), source.getNumComponents(), 
                                                                 instanceCount * source.getNumElements());
            this.sourceData = (IntBuffer)source.getDataReadOnly();
            this.buffer = new int[source.getNumComponents()];
        }
                                                                                                
        public final void addInstance( int id, Vector3f pos, Quaternion rot, float scale ) {
            sourceData.rewind();
            for( int i = 0; i < elementCount; i++ ) {
                sourceData.get(buffer);
                if( function != null ) {
                    function.apply(buffer, id, pos, rot, scale);
                }
                target.put(buffer);
            }   
        }
                
        protected Buffer getTarget() {
            return target;
        }
    }

    
}
