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
    private Grid grid;
    private LoadingCache<ChunkId, Chunk> cache;

    public CachingDensityVolume( int cacheSize, DensityVolume source, int xzSize, int ySize ) {
        this.source = source;
        this.xzSize = xzSize;
        this.ySize = ySize;
        this.grid = new Grid(xzSize, ySize, xzSize);
 
        this.cache = CacheBuilder.newBuilder().maximumSize(cacheSize).build(new ChunkLoader());       
    }

    private Chunk getChunk( int x, int y, int z ) {
        int xCell = grid.toCellX(x);
        int yCell = grid.toCellX(y);
        int zCell = grid.toCellX(z);
        try {
            return cache.get(new ChunkId(xCell, yCell, zCell));
        } catch( ExecutionException ex ) {
            throw new RuntimeException("Error creating chunk", ex);
        }
    }

    private Chunk getChunk( float x, float y, float z ) {
        int xCell = grid.toCellX(x);
        int yCell = grid.toCellX(y);
        int zCell = grid.toCellX(z);
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
        ArrayDensityVolume volume;
        int xBase;
        int yBase;
        int zBase;
        
        public Chunk( ChunkId id, int xBase, int yBase, int zBase, ArrayDensityVolume volume ) {
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
    
    private class ChunkLoader extends CacheLoader<ChunkId, Chunk> {

        @Override
        public Chunk load( ChunkId id ) throws Exception {
 
            int xBase = (int)grid.toWorldX(id.xChunk); 
            int yBase = (int)grid.toWorldX(id.yChunk); 
            int zBase = (int)grid.toWorldX(id.zChunk);
 
            // Make the chunk 2 elements bigger all the way 
            // around so that the field direction is likely to work
            // ArrayDensityVolume samples coordinate + 1... which means
            // that 8.5 + 1 = 9.5 which means that we'd need element 10
            // also... not just element 9
            xBase-=2;
            yBase-=2;
            zBase-=2;
                        
            ArrayDensityVolume chunk = ArrayDensityVolume.extractVolume(source, xBase, yBase, zBase,
                                                                        xzSize + 4, ySize + 4, xzSize + 4);            
            
            return new Chunk(id, xBase, yBase, zBase, chunk);           
        }
    }    
}


