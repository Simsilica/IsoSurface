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

import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.VertexBuffer.Type;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.nio.ShortBuffer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


/**
 *  Utility methods for comparing meshes and mesh parts.
 *
 *  @author    Paul Speed
 */
public class MeshCompareUtil {

    static Logger log = LoggerFactory.getLogger(MeshCompareUtil.class);

    public static boolean compare( Mesh mesh1, Mesh mesh2 ) {
        VertexBuffer[] source = mesh1.getBufferList().getArray();
        for( int i = 0; i < source.length; i++ ) {
            VertexBuffer comp = mesh2.getBuffer(source[i].getBufferType());
            if( comp == null ) {
                log.info("Second mesh has no type:" + source[i].getBufferType());
                return false; 
            }
            if( !compare(source[i], comp) ) {
                return false;
            }            
        }
        return true;
    }
    
    public static boolean compare( VertexBuffer vb1, VertexBuffer vb2 ) {
        if( vb1.getBufferType() != vb2.getBufferType() ) {
            log.info("Buffer types differ 1:" + vb1.getBufferType() + "  2:" + vb2.getBufferType());
            return false;
        }
        if( vb1.getFormat() != vb2.getFormat() ) {
            log.info("Buffer formats differ for type:" + vb1.getBufferType() 
                      + " 1:" + vb1.getFormat() + "  2:" + vb2.getFormat());
            return false;
        }
        if( vb1.getNumComponents() != vb2.getNumComponents() ) {
            log.info("Buffer components differ for type:" + vb1.getBufferType() 
                      + " 1:" + vb1.getNumComponents() + "  2:" + vb2.getNumComponents());
            return false;
        }
        switch( vb1.getFormat() ) {
            case Float:
                if( !compare(vb1.getBufferType(), (FloatBuffer)vb1.getData(), (FloatBuffer)vb2.getData()) ) {
                    return false;
                }
                break;
            case UnsignedShort:
            case Short:
                if( !compare(vb1.getBufferType(), (ShortBuffer)vb1.getData(), (ShortBuffer)vb2.getData()) ) {
                    return false;
                }
                break;
            case UnsignedInt:
            case Int:
                if( !compare(vb1.getBufferType(), (IntBuffer)vb1.getData(), (IntBuffer)vb2.getData()) ) {
                    return false;
                }
                break;
            default:
                throw new UnsupportedOperationException("Unhandled format" + vb1.getFormat());
        }
        return true;
    }
    
    public static boolean compare( Type type, FloatBuffer b1, FloatBuffer b2 ) {
        b1.rewind();
        b2.rewind();
        for( int i = 0; i < b1.limit(); i++ ) {
            float v1 = b1.get();
            float v2 = b2.get();
            if( v1 != v2 ) {
                log.info("Type:" + type + " differs at pos:" + i + "  1:" + v1 + "  2:" + v2);
                return false;
            }
        }
        return true;
    }
    
    public static boolean compare( Type type, ShortBuffer b1, ShortBuffer b2 ) {
        b1.rewind();
        b2.rewind();
        for( int i = 0; i < b1.limit(); i++ ) {
            short v1 = b1.get();
            short v2 = b2.get();
            if( v1 != v2 ) {
                log.info("Type:" + type + " differs at pos:" + i + "  1:" + v1 + "  2:" + v2);
                return false;
            }
        }
        return true;
    }
    
    public static boolean compare( Type type, IntBuffer b1, IntBuffer b2 ) {
        b1.rewind();
        b2.rewind();
        for( int i = 0; i < b1.limit(); i++ ) {
            int v1 = b1.get();
            int v2 = b2.get();
            if( v1 != v2 ) {
                log.info("Type:" + type + " differs at pos:" + i + "  1:" + v1 + "  2:" + v2);
                return false;
            }
        }
        return true;
    }

}
