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


import com.jme3.texture.Image;
import com.jme3.texture.Texture;
import java.nio.ByteBuffer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


/**
 *  Array data that can be sampled with bilinear interpolation
 *  similar to the way textures are sampled.  This is an 'int'
 *  array but the 4 bytes in each int are interpolated separately
 *  as if they represent a four component color.
 *
 *  @author    Paul Speed
 */
public class BilinearArray {

    static Logger log = LoggerFactory.getLogger(BilinearArray.class);
    
    private int width;
    private int height;
    private int[] array;
    
    public BilinearArray( int width, int height ) {
        this.width = width;
        this.height = height;
        this.array = new int[width * height];
    } 

    public static BilinearArray fromTexture( Texture texture ) {
        return fromImage(texture.getImage());
    }
    
    public static BilinearArray fromImage( Image image ) {
        BilinearArray result = new BilinearArray(image.getWidth(), image.getHeight());
 
        ByteBuffer data = image.getData(0);
        data.rewind();
    
        System.out.println( "Format:" + image.getFormat() + " bpp:" + image.getFormat().getBitsPerPixel() );
        int span = image.getFormat().getBitsPerPixel() / 8;
        int size = result.array.length;
               
        for( int i = 0; i < size; i++ ) {
            byte b1 = data.get();
            byte b2 = 0;
            byte b3 = 0;
            byte b4 = 0;
            if( span > 1 ) {
                b2 = data.get();
            }
            if( span > 2 ) {
                b3 = data.get();
            }
            if( span > 3 ) {
                b4 = data.get();
            }
            if( span > 4 ) {
                for( int skip = 4; skip < span; skip++ ) {
                    data.get();
                }
            }
            int value = (b1 & 0xff) << 24;
            value |= (b2 & 0xff) << 16; 
            value |= (b3 & 0xff) << 8; 
            value |= (b4 & 0xff);
            result.array[i] = value; 
        }
               
        return result;
    }

    private int index( int x, int y ) {
        return y * width + x;
    }

    private int wrappedIndex( int x, int y ) {
        x = x % width;
        y = y % height;
        
        if( x < 0 ) {
            x = width + x;
        } 
        if( y < 0 ) {
            y = height + y;
        } 
        int result = y * width + x;
        if( result >= array.length ) {
            throw new RuntimeException( "Bad clipping:" + x + ", " + y + "  width:" + width + " height:" + height );
        }
        return result;
    }
    
    public int get( int x, int y ) {
        return array[wrappedIndex(x,y)];
    }
    
    public void set( int x, int y, int value ) {
        array[wrappedIndex(x,y)] = value;
    }
 
    public byte[] getHomogenous( double u, double v, byte[] target ) {
        return get(u * width, v * height, target);
    }
    
    public byte[] get( double x, double y, byte[] target ) {
        if( target == null ) {
            target = new byte[4];
        }
        
        int ll = get((int)Math.floor(x), (int)Math.floor(y));
        int lr = get((int)Math.ceil(x), (int)Math.floor(y)); 
        int ul = get((int)Math.floor(x), (int)Math.ceil(y));
        int ur = get((int)Math.ceil(x), (int)Math.ceil(y));
 
        if( log.isTraceEnabled() ) {
            log.trace("ll:%08X  lr:%08X  ul:%08X  ur:%08X\n", ll, lr, ul, ur); 
        }
        double xPart = x - Math.floor(x);
        
        int l1 = interpolate(ll & 0xff, lr & 0xff, xPart);
        int l2 = interpolate((ll >> 8 ) & 0xff, (lr >> 8 ) & 0xff, xPart);
        int l3 = interpolate((ll >> 16) & 0xff, (lr >> 16) & 0xff, xPart);
        int l4 = interpolate((ll >> 24) & 0xff, (lr >> 24) & 0xff, xPart);

        int u1 = interpolate(ul & 0xff, ur & 0xff, xPart);
        int u2 = interpolate((ul >> 8 ) & 0xff, (ur >> 8 ) & 0xff, xPart);
        int u3 = interpolate((ul >> 16) & 0xff, (ur >> 16) & 0xff, xPart);
        int u4 = interpolate((ul >> 24) & 0xff, (ur >> 24) & 0xff, xPart);
               
        double yPart = y - Math.floor(y);
  
        if( log.isTraceEnabled() ) {
            log.trace("lower: %02X%02X%02X%02X  upper: %02X%02X%02X%02X\n", l4, l3, l2, l1, u4, u3, u2, u1 );
        } 
        
        target[3] = (byte)interpolate(l1, u1, yPart);
        target[2] = (byte)interpolate(l2, u2, yPart);
        target[1] = (byte)interpolate(l3, u3, yPart);
        target[0] = (byte)interpolate(l4, u4, yPart);
         
        return target;
    }

    private int interpolate( int b1, int b2, double f ) {
        int result = b1 + (int)Math.round((b2 - b1) * f);
        return result;
    }

    public static String toString( byte[] array ) {
        return String.format("0x%02X%02X%02X%02X", array[0], array[1], array[2], array[3]);
    }

    public static void main( String... args ) {
        
        BilinearArray test = new BilinearArray(10, 10);
        test.set(0, 0, 0x01010404);
        test.set(1, 0, 0x08080404);
        test.set(0, 1, 0x02020202);
        test.set(1, 1, 0x08080808);
        
        System.out.println( "0,0=" + test.get(0,0) );
        System.out.println( "1,1=" + test.get(1,1) );
        System.out.println( "0.5,0.5=" + toString(test.get(0.5,0.5, null)) );
        System.out.println( "0.25,0.5=" + toString(test.get(0.25,0.5, null)) );
        System.out.println( "0.25,0.25=" + toString(test.get(0.25,0.25, null)) );
        System.out.println( "10,10=" + toString(test.get(10,10, null)) );
        System.out.println( "9.5,9.5=" + toString(test.get(9.5,9.5, null)) );
        System.out.println( "9.25,9.25=" + toString(test.get(9.25,9.25, null)) );
    }
    
    @Override
    public String toString() {
        return "BilinearArray[" + width + ", " + height + "]";
    }
}
