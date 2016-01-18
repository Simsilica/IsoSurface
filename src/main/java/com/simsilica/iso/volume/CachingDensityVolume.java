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

import com.google.common.cache.CacheBuilder;
import com.google.common.cache.CacheLoader;
import com.google.common.cache.LoadingCache;
import com.jme3.math.Vector3f;
import com.simsilica.iso.DensityVolume;
import com.simsilica.pager.Grid;
import java.util.concurrent.ExecutionException;


/**
 *  Presents a continuous array-based view of 'chunks' that
 *  are extracted from a continuous source field.
 *
 *  @author    Paul Speed
 */
public class CachingDensityVolume implements DensityVolume {

    private DensityVolume source;
    private int xzSize;
    private int ySize;
    private int superSample;
    private Grid grid;
    private LoadingCache<ChunkId, Chunk> cache;

    public CachingDensityVolume( int cacheSize, DensityVolume source, int xzSize, int ySize, int xzSuperSample ) {    
        this.source = source;
        this.superSample = xzSuperSample;
        this.xzSize = xzSize / xzSuperSample;
        this.ySize = ySize;
        this.grid = new Grid(xzSize, ySize, xzSize);
 
        this.cache = CacheBuilder.newBuilder().maximumSize(cacheSize).build(new ChunkLoader());       
    }

    public CachingDensityVolume( int cacheSize, DensityVolume source, int xzSize, int ySize ) {
        this(cacheSize, source, xzSize, ySize, 1);
    }

    private Chunk getChunk( int x, int y, int z ) {
        int xCell = grid.toCellX(x);
        int yCell = grid.toCellY(y);
        int zCell = grid.toCellZ(z);
        try {
            return cache.get(new ChunkId(xCell, yCell, zCell));
        } catch( ExecutionException ex ) {
            throw new RuntimeException("Error creating chunk", ex);
        }
    }

    private Chunk getChunk( float x, float y, float z ) {
        int xCell = grid.toCellX(x);
        int yCell = grid.toCellY(y);
        int zCell = grid.toCellZ(z);
        try {
            return cache.get(new ChunkId(xCell, yCell, zCell));
        } catch( ExecutionException ex ) {
            throw new RuntimeException("Error creating chunk", ex);
        }
    }

    @Override
    public final float getDensity( int x, int y, int z ) {
        Chunk chunk = getChunk(x, y, z);
        return chunk.getDensity(x, y, z);
    }

    @Override
    public final float getDensity( float x, float y, float z ) { 
        Chunk chunk = getChunk(x, y, z);
        return chunk.getDensity(x, y, z);
    }

    @Override
    public final Vector3f getFieldDirection( float x, float y, float z, Vector3f target ) { 
        Chunk chunk = getChunk(x, y, z);
        return chunk.getFieldDirection(x, y, z, target);
    }
 
    private class ChunkId {
        int xChunk;
        int yChunk;
        int zChunk;
 
        public ChunkId( int x, int y, int z ) {
            this.xChunk = x;
            this.yChunk = y;
            this.zChunk = z;
        }
    
        @Override    
        public final int hashCode() {
            int hash = 37;
            hash += 37 * hash + xChunk;
            hash += 37 * hash + yChunk;
            hash += 37 * hash + zChunk;
            return hash;
        }
        
        @Override
        public final boolean equals( Object o ) { 
            if( o == null || o.getClass() != getClass() ) 
                return false;
            if( o == this ) 
                return true;
            ChunkId other = (ChunkId)o;
            if( other.xChunk != xChunk ) 
                return false;
            if( other.yChunk != yChunk ) 
                return false;
            if( other.zChunk != zChunk ) 
                return false;
            return true;
        }
        
        @Override
        public String toString() {
            return "ChunkId[" + xChunk + ", " + yChunk + ", " + zChunk + "]";
        }
    }
 
    private class Chunk implements DensityVolume {
        ChunkId id; 
        DensityVolume volume;
        int xBase;
        int yBase;
        int zBase;
        
        public Chunk( ChunkId id, int xBase, int yBase, int zBase, DensityVolume volume ) {
            this.id = id;
            this.volume = volume;
            this.xBase = xBase;
            this.yBase = yBase;
            this.zBase = zBase;
        }

        @Override
        public final float getDensity( int x, int y, int z ) {
            return volume.getDensity(x - xBase, y - yBase, z - zBase);
        }

        @Override
        public final float getDensity( float x, float y, float z ) {
            return volume.getDensity(x - xBase, y - yBase, z - zBase);
        }

        @Override
        public final Vector3f getFieldDirection( float x, float y, float z, Vector3f target ) {
            return volume.getFieldDirection(x - xBase, y - yBase, z - zBase, target);
        }
    }
 
    public static void main( String... args ) {
        // Testing something
        
        DensityVolume source = new DensityVolume() {
            @Override
            public float getDensity( int x, int y, int z ) {
                return x;
            }
            
            @Override
            public float getDensity( float x, float y, float z ) {
                return x;
            }
            
            @Override
            public Vector3f getFieldDirection( float x, float y, float z, Vector3f target ) {
                return null;
            }
        };
 
        int xBase = 100;
        int superSample = 2;
        int xzSize = 16;
        int ySize = 32;
        
        ResamplingVolume resampled = new ResamplingVolume(new Vector3f(superSample, 1, superSample),
                                                          new Vector3f(xBase, 0, 0),
                                                          source);
                
        // Since we've moved prescaled origin then our xBase, yBase, etc. are already
        // moved for us
        DensityVolume result = ArrayDensityVolume.extractVolume(resampled, -2, -2, -2,
                                                                xzSize + 4, ySize + 4, xzSize + 4);

        for( int x = 0; x < 16; x++ ) {
            System.out.println( "intermediate:" + result.getDensity(x, 0, 0) );
        }

        // Now unscale it... we don't have to unmove it because
        // the chunk access is already translating it as if it were an array
        // based at 0, 0... but we'll offset for the border we created.
        result = new ResamplingVolume(new Vector3f(1f/superSample, 1, 1f/superSample),
                                      new Vector3f(2, 2, 2),
                                      result);
 
        for( int x = 100; x < 132; x++ ) {
            System.out.println( "Original:" + source.getDensity(x, 0, 0) 
                                + "  Resampled:" + result.getDensity(x-xBase, 0, 0) ); 
        }       
    }
    
    private class ChunkLoader extends CacheLoader<ChunkId, Chunk> {

        @Override
        public Chunk load( ChunkId id ) throws Exception {
 
System.out.println( "Loading chunk:" + id ); 
            int xBase = (int)grid.toWorldX(id.xChunk); 
            int yBase = (int)grid.toWorldY(id.yChunk); 
            int zBase = (int)grid.toWorldZ(id.zChunk);
 
            DensityVolume result;
            if( superSample != 1 ) {
                // If we are super-sampling then we extract less data but
                // treat it like more data.  It also means we are extracting
                // from a different place, though.
                // For example, if we have a cell size of 16 x 16 but we
                // are super-sampling *2 then we are really building an array of
                // 8 x 8 by skipping every other value.  But that means when
                // x = 3 in the 'regular world' that it's really 1.5 in the
                // super-sampled world.
                // Now, we can unscale it when returning and now we can skip
                // resolution.
                // xzSize has already had supersampling factored in.
                ResamplingVolume resampled = new ResamplingVolume(new Vector3f(superSample, 1, superSample),
                                                                  new Vector3f(xBase, yBase, zBase),
                                                                  source);
                // So we've setup sampling with xBase, yBase, zBase as 0,0,0 in our
                // source, and scaled by superSample otherwise                                                              
                                                                      
                
                // Since we've moved prescaled origin then our xBase, yBase, etc. are already
                // moved for us
                result = ArrayDensityVolume.extractVolume(resampled, -2, -2, -2,
                                                          xzSize + 4, ySize + 4, xzSize + 4);
                // The array now represents:
                //  xBase - 2 * superSample to that plus (xzSize + 4) * superSample

                // Now unscale it... we don't have to unmove it because
                // the chunk access is already translating it as if it were an array
                // based at 0, 0... but we'll offset for the border we created.
                result = new ResamplingVolume(new Vector3f(1f/superSample, 1, 1f/superSample),
                                              new Vector3f(2, 2, 2),
                                              result);
                // So, the above indexes 0 as 2, 2, 2 in the array... which represented
                // a skipped version of our regular world so we un-sample to index into
                // it properly.                                             
            } else {                        
                // Make the chunk 2 elements bigger all the way 
                // around so that the field direction is likely to work
                // ArrayDensityVolume samples coordinate + 1... which means
                // that 8.5 + 1 = 9.5 which means that we'd need element 10
                // also... not just element 9
                xBase-=2;
                yBase-=2;
                zBase-=2;
    
                result = ArrayDensityVolume.extractVolume(source, xBase, yBase, zBase,
                                                          xzSize + 4, ySize + 4, xzSize + 4);
            }                                                                                        
            
            return new Chunk(id, xBase, yBase, zBase, result);           
        }
    }    
}


