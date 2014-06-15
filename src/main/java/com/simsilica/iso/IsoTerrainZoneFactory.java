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
import com.simsilica.pager.PagedGrid;
import com.simsilica.pager.Zone;
import com.simsilica.pager.ZoneFactory;


/**
 *  Creates IsoTerrainZones that carve out specific sections of
 *  a supplied world DensityVolume and use a supplied MeshGenerator
 *  to build meshes.
 *
 *  @author    Paul Speed
 */
public class IsoTerrainZoneFactory implements ZoneFactory {

    private DensityVolume worldVolume;
    private Supplier<? extends MeshGenerator> generator;
    private Vector3f volumeSize;
    private Vector3f volumeOffset;
    private Material terrainMaterial;
    private boolean generateCollisionData;

    public IsoTerrainZoneFactory( DensityVolume worldVolume,
                                  Vector3f volumeSize, Vector3f volumeOffset,
                                  Supplier<? extends MeshGenerator> generator,
                                  Material terrainMaterial,
                                  boolean generateCollisionData ) {
        this.worldVolume = worldVolume;
        this.generator = generator;
        this.volumeSize = volumeSize;
        this.volumeOffset = volumeOffset;
        this.terrainMaterial = terrainMaterial;
        this.generateCollisionData = generateCollisionData;                               
    }
    
    public Zone createZone( PagedGrid pg, int xCell, int yCell, int zCell ) {

        IsoTerrainZone result = new IsoTerrainZone(xCell, yCell, zCell, pg.getGrid(), 
                                                   volumeSize, volumeOffset, worldVolume,
                                                   generator, terrainMaterial,
                                                   generateCollisionData);
        
        return result; 
    }
}
