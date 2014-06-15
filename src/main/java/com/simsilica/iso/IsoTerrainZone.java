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

package com.simsilica.iso;

import com.google.common.base.Supplier;
import com.jme3.material.Material;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.VertexBuffer;
import com.jme3.util.BufferUtils;
import com.simsilica.iso.volume.ArrayDensityVolume;
import com.simsilica.pager.Grid;
import com.simsilica.pager.Zone;
import java.util.concurrent.locks.ReentrantLock;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


/**
 *  A Zone implementation that uses a MeshGenerator and a 
 *  density field to generator a terrain section.
 *
 *  @author    Paul Speed
 */
public class IsoTerrainZone implements Zone {

    static Logger log = LoggerFactory.getLogger(IsoTerrainZone.class);

    private int xCell;
    private int yCell;
    private int zCell;
    private Grid grid;
    private Vector3f volumeSize;
    private Vector3f volumeOffset;
    private DensityVolume source;
    private int priority;
    private Node node;
    private Spatial land;
    private Spatial wire;
    private Supplier<? extends MeshGenerator> generator;
    private Material terrainMaterial;
    private boolean generateCollisionData;

    // For testing to make sure the builder and the pager are not
    // competing.  Leaving it in is just a bit of paranoia
    private final ReentrantLock accessLock = new ReentrantLock();

    private static ThreadLocal<ArrayDensityVolume> cachedVolume = new ThreadLocal<ArrayDensityVolume>();

    public IsoTerrainZone( int xCell, int yCell, int zCell, Grid grid,  
                           Vector3f volumeSize, Vector3f volumeOffset, 
                           DensityVolume source,
                           Supplier<? extends MeshGenerator> generator,
                           Material terrainMaterial,
                           boolean generateCollisionData )
    {
        this.xCell = xCell;
        this.yCell = yCell;
        this.zCell = zCell;
        this.grid = grid;
        this.volumeSize = volumeSize;
        this.volumeOffset = volumeOffset;
        this.source = source;
        this.generator = generator;
        this.terrainMaterial = terrainMaterial;
        this.generateCollisionData = generateCollisionData;
        this.node = new Node("Terrain[" + xCell + ", " + yCell + ", " + zCell + "]");        
    }
 
    public Grid getGrid() {
        return grid;
    }
 
    public int getXCell() {
        return xCell;
    }

    public int getYCell() {
        return yCell;
    }

    public int getZCell() {
        return zCell;
    }
 
    public float getXWorld() {
        return grid.toWorldX(xCell);
    }

    public float getYWorld() {
        return grid.toWorldY(yCell);
    }

    public float getZWorld() {
        return grid.toWorldZ(zCell);
    }

    public Vector3f getWorldLocation( Vector3f target ) {
        return grid.toWorld(xCell, yCell, zCell, target);
    }
    
    public void setMeshGenerator( Supplier<? extends MeshGenerator> generator ) {
        this.generator = generator;
    }
 
    public void resetPriority( int xCenter, int yCenter, int zCenter, int bias )
    {
        int dx = xCell - xCenter;
        int dz = zCell - zCenter;
        this.priority = bias * (int)Math.sqrt(dx * dx + dz * dz);
    }
    
    public int getPriority() {
        return priority;
    }

    public Node getNode() {
        return node;
    }

    public Node getZoneRoot() {
        return node;
    }

    public void setParentZone( Zone parentZone ) {
    }

    public void build() {
        if( log.isInfoEnabled() ) {
            log.info("Building:" + xCell + ", " + yCell + ", " + zCell + "  priority:" + priority);
        }
 
        accessLock.lock();
        try {                       
            long start = System.nanoTime();
 
            int x = (int)(volumeOffset.x + xCell * volumeSize.x); 
            int y = (int)(volumeOffset.y + yCell * volumeSize.y); 
            int z = (int)(volumeOffset.z + zCell * volumeSize.z);

            MeshGenerator mg = generator.get();
            Vector3f size = mg.getRequiredVolumeSize(); 
            int cx = (int)size.x; 
            int cy = (int)size.y; 
            int cz = (int)size.z;
            
            // Note: we presume in this caching strategy that the 
            //       required volume size never changes.  But that's a reasonable
            //       assumption since if it did then a new terrain pager or factory
            //       should have been created for about a dozen other reasons. 
            ArrayDensityVolume volume = cachedVolume.get();
            if( volume == null ) { 
                volume = ArrayDensityVolume.extractVolume(source, x, y, z, cx, cy, cz);
                cachedVolume.set(volume);
            } else {
                volume.extract(source, x, y, z);
            }
 
            long time1 = System.nanoTime();
            long time2 = time1;
    
            Mesh landMesh = generator.get().buildMesh(volume);
            if( landMesh != null ) {
                land = createLand(landMesh, false);
                if( log.isDebugEnabled() ) {
                    log.debug("volume size:" + volumeSize);                
                    log.debug("bounding shape:" + landMesh.getBound());
                }                
                land.setShadowMode(RenderQueue.ShadowMode.CastAndReceive);
                time2 = System.nanoTime();
                if( generateCollisionData ) {
                    landMesh.createCollisionData();
                } 
            } else {
                land = null;
                log.debug("Empty mesh.");
            }
            long end = System.nanoTime();
            
            if( log.isInfoEnabled() ) {
                log.info("Total generation time:" + ((end-start)/1000000.0) + " ms");
                log.info("  Density field:" + ((time1-start)/1000000.0) + " ms");
                log.info("  Mesh generation:" + ((time2-time1)/1000000.0) + " ms");
                log.info("  Collision data generation:" + ((end-time2)/1000000.0) + " ms");
            }
        } finally {
            accessLock.unlock();
        }                
    }

    public void apply() {
        if( !accessLock.tryLock() ) {
            throw new IllegalStateException("Thread is still building.");
        }
        try {        
            if( land != null ) {
                node.attachChild(land);
            }
        } finally {
            accessLock.unlock();
        }
    }

    public void release() {
        if( log.isTraceEnabled() ) {
            log.trace("IsoLandReference.release():" + this );
        }
            
        if( !accessLock.tryLock() ) {
            throw new IllegalStateException("Thread is still building.");
        }
        try {        
            // Should queue the buffers to be freed
            // or something.
            // ...for now we will free them directly.
            if( land != null ) {
                Mesh mesh = ((Geometry)land).getMesh();
                for( VertexBuffer vb : mesh.getBufferList() ) {
                    if( log.isTraceEnabled() ) {
                        log.trace("--destroying buffer:" + vb );
                    }
                    BufferUtils.destroyDirectBuffer( vb.getData() );
                }
            }
        } finally {
            accessLock.unlock();
        }        
    }

    protected Spatial createLand( Mesh mesh, boolean glass )
    {
        Geometry geom = new Geometry("landGeometry[" + xCell + ", " + yCell + ", " + zCell + "]", mesh);
        geom.setMaterial(terrainMaterial);         
        return geom;        
    }

    /*protected Spatial createWireLand( Mesh mesh )
    {
        Geometry geom = new Geometry("wireLandGeometry[" + xCell + ", " + yCell + ", " + zCell + "]", mesh);
        geom.setMaterial(wireMaterial);
        geom.setQueueBucket(Bucket.Transparent);         
        return geom;        
    }*/

    @Override
    public String toString() {
        return "IsoLandReference[" + xCell + ", " + yCell + ", " + zCell + "]";
    }
}


