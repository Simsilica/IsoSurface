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

package com.simsilica.iso.mc;

import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer.Type;
import com.jme3.util.BufferUtils;
import com.simsilica.iso.DensityVolume;
import com.simsilica.iso.MeshGenerator;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


/**
 *  Takes a density field and generates meshes for it
 *  using the Marching Cubes algorithm.
 *
 *  @author    Paul Speed
 */
public class MarchingCubesMeshGenerator implements MeshGenerator {

    private int cx;
    private int cy;
    private int cz;
    private int[] masks;
    private int[] cells;
    private int cellIndex = 0;
    private int triangleCount = 0;
    private int vertCount = 0;
    private int maskIndex = 0;
    private boolean[] edgeHit = new boolean[12]; 
    private int[][][][] edgeVerts;
    private float xzScale = 1;

    /**
     *  Creates a Marching Cubes based mesh generator that will
     *  generate chunks of the specified size.
     */
    public MarchingCubesMeshGenerator( int cx, int cy, int cz ) {
        this(cx, cy, cz, 1);
    }
    
    /**
     *  Creates a Marching Cubes based mesh generator that will
     *  generate chunks of the specified size with an extra x, z
     *  scale applied to the resulting mesh.  In other words,
     *  even though the source data may be sampled in 64x64 cell
     *  chunks, with an xzScale of 2 the generated geometry will be
     *  128 x 128. 
     */
    public MarchingCubesMeshGenerator( int cx, int cy, int cz, float xzScale ) {
        cx++;
        cy++;
        cz++;
        this.cx = cx;
        this.cy = cy;
        this.cz = cz;
        this.masks = new int[cx * cy * cz];
        this.cells = new int[cy * cy * cz];
        this.edgeVerts = new int[cx+1][cy+1][cz+1][3];
        this.xzScale = xzScale;               
    }
 
    public Vector3f getRequiredVolumeSize() {
        // When the creator passed in a size of "cells", we actually
        // sample corners and so neded +1
        // However, we also sample an extra border on all sides which
        // is another +2... +3 in all.  But the +1 is already incorporated
        // into our size fields. 
        return new Vector3f(cx + 2, cy + 2, cz + 2);
    }
 
    public Vector3f getGenerationSize() {
        int x = cx - 1;
        int y = cy - 1;
        int z = cz - 1;
        return new Vector3f(x * xzScale, y, z * xzScale);
    }
 
    public void setXzScale( float s ) {
        this.xzScale = s;
    }
 
    public float getXzScale() {
        return xzScale;
    }
 
    private int solid( float value ) {
        return value > 0 ? 1 : 0;
    }

    private Vector3f getEdgePoint( int x, int y, int z, int edge, DensityVolume volume ) {
        
        int x1 = x + MarchingCubesConstants.edgeStarts[edge][0] + 1; 
        int y1 = y + MarchingCubesConstants.edgeStarts[edge][1] + 1; 
        int z1 = z + MarchingCubesConstants.edgeStarts[edge][2] + 1; 
        float d1 = volume.getDensity(x1, y1, z1);
                            
        int x2 = x + MarchingCubesConstants.edgeEnds[edge][0] + 1; 
        int y2 = y + MarchingCubesConstants.edgeEnds[edge][1] + 1; 
        int z2 = z + MarchingCubesConstants.edgeEnds[edge][2] + 1; 
        float d2 = volume.getDensity(x2, y2, z2);
 
        // If d1 is -0.2 and d2 is 0.6 then the 
        // point should be 0.25 from edge start.
        float part = Math.abs(d1) / Math.abs(d2 - d1);
        float vx = x1 + (x2-x1) * part;
        float vy = y1 + (y2-y1) * part;
        float vz = z1 + (z2-z1) * part; 
        Vector3f vert = new Vector3f(vx, vy, vz);
        return vert;
    }   

    /**
     *  Builds a mesh from the specified volume.  The resulting mesh
     *  will be extracted from 0 to size in all directions but requires
     *  the volume to support queries -1 to size + 2 because it will
     *  will internally build a border of cells. 
     */    
    public Mesh buildMesh( DensityVolume volume ) {
    
        List<Vector3f> verts = new ArrayList<Vector3f>();
        List<Vector3f> normals = new ArrayList<Vector3f>();
        cellIndex = 0;
        triangleCount = 0;
        vertCount = 0;
        maskIndex = 0;
 
        // Build up the edge indexes so we can share edges
        for( int x = 0; x < cx; x++ ) {
            for( int y = 0; y < cy; y++ ) {
                for( int z = 0; z < cz; z++ ) {
                    int bits = 0;
                    int sx = x + 1;
                    int sy = y + 1;
                    int sz = z + 1;
                    
                    bits |= solid(volume.getDensity(sx  , sy  , sz  ));
                    bits |= solid(volume.getDensity(sx  , sy+1, sz  )) << 1;
                    bits |= solid(volume.getDensity(sx+1, sy+1, sz  )) << 2;
                    bits |= solid(volume.getDensity(sx+1, sy  , sz  )) << 3;
                    bits |= solid(volume.getDensity(sx  , sy  , sz+1)) << 4;
                    bits |= solid(volume.getDensity(sx  , sy+1, sz+1)) << 5;
                    bits |= solid(volume.getDensity(sx+1, sy+1, sz+1)) << 6;
                    bits |= solid(volume.getDensity(sx+1, sy  , sz+1)) << 7;
                    
                    if( MarchingCubesConstants.triEdges[bits].length > 0 ) {
                        // We _do_ want to process some of the edges but
                        // we _don't_ want to process the actual cell if
                        // it is the outside border
                        if( x < cx - 1 && y < cy - 1 && z < cz - 1 ) { 
                            cells[cellIndex++] = maskIndex;
                        }
                        int[][] triangles = MarchingCubesConstants.triEdges[bits];
                        triangleCount += triangles.length;
                        Arrays.fill(edgeHit, false);
                        
                        for( int t = 0; t < triangles.length; t++ ) {
                            int[] triEdges = triangles[t];
                            
                            for( int i = 0; i < 3; i++ ) {
                                edgeHit[triEdges[i]] = true;
                            }
                        }
                            
                        // The density field is theoretically cx * cy * cz which
                        // means that we only need to generate cells for cx - 1, cy -1, cz -1
                        // We generate extra cells to make sure we have the edges we need...
                        // but these cells are technically outside of the field and
                        // we will need to skip them later.  Also, I guess we can avoid
                        // generating some extra vertexes that would stick out.
                        if( edgeHit[0] && y < cy - 1 ) {
                            Vector3f vert = getEdgePoint(x, y, z, 0, volume); 
                            edgeVerts[x][y][z][0] = verts.size();
                            verts.add(vert);
                            normals.add(volume.getFieldDirection(vert.x, vert.y, vert.z, null));
                            vert.x--;
                            vert.y--;
                            vert.z--;                            
                        }
                        if( edgeHit[3] && x < cx - 1 ) {
                            Vector3f vert = getEdgePoint(x, y, z, 3, volume); 
                            edgeVerts[x][y][z][1] = verts.size();
                            verts.add(vert);
                            normals.add(volume.getFieldDirection(vert.x, vert.y, vert.z, null));                            
                            vert.x--;
                            vert.y--;
                            vert.z--;                            
                        }
                        if( edgeHit[8] && z < cz - 1 ) {
                            Vector3f vert = getEdgePoint(x, y, z, 8, volume); 
                            edgeVerts[x][y][z][2] = verts.size();
                            verts.add(vert);
                            normals.add(volume.getFieldDirection(vert.x, vert.y, vert.z, null));                            
                            vert.x--;
                            vert.y--;
                            vert.z--;                            
                        }
                    }   
                    masks[maskIndex++] = bits;                    
                }
            }
        }

        int cellCount = cellIndex;
        if( cellCount == 0 )
            return null;

        if( xzScale != 1 ) {
            for( int i = 0; i < verts.size(); i++ ) {
                Vector3f v = verts.get(i);
                Vector3f n = normals.get(i);
                v.x *= xzScale;
                v.z *= xzScale;
                n.y *= xzScale;
                n.normalizeLocal();
            }
        }
            
        int[] triIndexes = new int[triangleCount * 3];
        int triIndexIndex = 0;
        
        // Now let's just visit the non-empty cells and spin out the
        // shared triangles' indexes.
        int cycz = cy * cz;
        for( int c = 0; c < cellCount; c++ ) {
        
            int index = cells[c]; 
            int mask = masks[index];
            int x = index / (cycz);
            int y = (index % (cycz)) / cz;
            int z = index % cz;
             
            int[][] triangles = MarchingCubesConstants.triEdges[mask];
            if( triangles.length == 0 ) {
                throw new RuntimeException("Algorithm inconsistency detected.");
                //System.out.println( "****** How did this happen? ******" );
                //continue;
            }                    
                    
            for( int t = 0; t < triangles.length; t++ ) {
            
                int[] triEdges = triangles[t];
                        
                for( int i = 0; i < 3; i++ ) {
                    int vertIndex;
                    switch(triEdges[i]) {
                        case 0:
                            vertIndex = edgeVerts[x][y][z][0];
                            break;
                        case 1:
                            vertIndex = edgeVerts[x][y+1][z][1];
                            break;
                        case 2:
                            vertIndex = edgeVerts[x+1][y][z][0];
                            break;
                        case 3:
                            vertIndex = edgeVerts[x][y][z][1];
                            break;
                        case 4:
                            vertIndex = edgeVerts[x][y][z+1][0];
                            break;
                        case 5:
                            vertIndex = edgeVerts[x][y+1][z+1][1];
                            break;
                        case 6:
                            vertIndex = edgeVerts[x+1][y][z+1][0];
                            break;
                        case 7:
                            vertIndex = edgeVerts[x][y][z+1][1];
                            break;
                        case 8:
                            vertIndex = edgeVerts[x][y][z][2];
                            break;
                        case 9:
                            vertIndex = edgeVerts[x][y+1][z][2];
                            break;
                        case 10:
                            vertIndex = edgeVerts[x+1][y+1][z][2];
                            break;
                        case 11:
                            vertIndex = edgeVerts[x+1][y][z][2];
                            break;
                        default:    
                            throw new RuntimeException("Unknown edge:" + triEdges[i]);
                    }
 
                    triIndexes[triIndexIndex++] = vertIndex;
                }                            
            }
        }

        Vector3f[] vertArray = new Vector3f[verts.size()];
        vertArray = verts.toArray(vertArray);
        
        Vector3f[] normArray = new Vector3f[normals.size()];
        normArray = normals.toArray(normArray);

        Mesh mesh = new Mesh();
        FloatBuffer pb = BufferUtils.createFloatBuffer(vertArray);
        mesh.setBuffer(Type.Position, 3, pb);
        FloatBuffer nb = BufferUtils.createFloatBuffer(normArray);
        mesh.setBuffer(Type.Normal, 3, nb);        
        IntBuffer ib = BufferUtils.createIntBuffer(triIndexes);
        mesh.setBuffer(Type.Index, 3, ib);
        
        mesh.updateBound();
     
        return mesh;       
    } 
}


