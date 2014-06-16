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

import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue.ShadowMode;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.VertexBuffer.Type;
import com.jme3.util.BufferUtils;
import com.simsilica.iso.tri.TriangleUtils;
import com.simsilica.iso.tri.Triangle;
import com.simsilica.iso.tri.TriangleProcessor;
import com.simsilica.iso.util.BilinearArray;
import com.simsilica.pager.AbstractZone;
import com.simsilica.pager.Grid;
import com.simsilica.pager.PagedGrid;
import com.simsilica.pager.Zone;
import com.simsilica.pager.ZoneFactory;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.nio.ShortBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


/**
 *  A zone implementation that tests plotting various 
 *  frequencies and strengths.
 *
 *  @author    Paul Speed
 */
public class TreeZone extends AbstractZone {

    static Logger log = LoggerFactory.getLogger(PlotFrequencyZone.class);

    private Material material;
    private BilinearArray noise;
    private Geometry[] geomArray;
    private Geometry[] builtGeomArray;
    
    private TreeType[] treeTemplates;
        
    // For informational purposes, I just want to know what
    // the largest blade count generated for a zone is.
    private static int maxTreeCount = 0;

    public TreeZone( Grid grid, Material material, BilinearArray noise, int xCell, int yCell, int zCell,
                     Node... treeTemplates ) {
        super(grid, xCell, yCell, zCell);
        this.material = material;
        this.noise = noise;
        this.treeTemplates = new TreeType[treeTemplates.length];
        for( int i = 0; i < treeTemplates.length; i++ ) {
            this.treeTemplates[i] = new TreeType(treeTemplates[i]);
        }        
    }
    
    public void build() {

        Grid grid = getGrid();
        Vector3f size = grid.getCellSize();
        
        long seed = (long)getXCell() << 32 | getYCell();
        Random random = new Random(seed);
        
        FrequencyPlotter plotter = new FrequencyPlotter(treeTemplates.length, random);
        if( getParentZone() != null ) {       
            // Find the parent relative corner.  This zone could be one of
            // several splitting up a larger zone.  To interact with the plotter
            // we need to know what area of the parent we should be scanning.
            Vector3f worldLoc = grid.toWorld(getXCell(), getYCell(), getZCell(), null);
            Vector3f parentLoc = getParentZone().getWorldLocation(null);
            plotter.world = worldLoc;
            plotter.min = worldLoc.subtract(parentLoc);
            log.trace("Cell min:" + plotter.min);
                        
            plotter.max = plotter.min.add(grid.getCellSize());
            
            // Scan the triangles for valid grass plots using the plotter.
            long start = System.nanoTime();
            int count = TriangleUtils.processTriangles(getParentZone().getZoneRoot(), plotter);
            long end = System.nanoTime();
            if( count > 0 && log.isInfoEnabled() ) {
                log.info("Plotted points for " + plotter.processedTriangleCount + " / " + count 
                            + " triangles in:" + ((end-start)/1000000.0) + " ms");
            }
        }
 
        int totalTreeCount = 0;
 
        // This loop will produce any number of geometry depending on the
        // number of tree types used and the LOD.
        List<Geometry> results = new ArrayList<Geometry>();
 
        long start = System.nanoTime();              
        for( int i = 0; i < treeTemplates.length; i++ ) {
            TreeType type = treeTemplates[i];
            TreeBin bin = plotter.bins[i];
            
            if( bin.instances.isEmpty() ) {
                continue;
            }
            
            if( log.isInfoEnabled() ) {
                log.info("bin[" + i + "] Number of points plotted:" + bin.instances.size());
            }
            totalTreeCount += bin.instances.size();
 
            type.addBatch(results, 0, bin.instances);
        }
        
        long end = System.nanoTime();
        if( log.isInfoEnabled() ) {
            log.info("Built " + totalTreeCount + " trees in:" + ((end-start)/1000000.0) + " ms");
        }
        
        builtGeomArray = new Geometry[results.size()];
        builtGeomArray = results.toArray(builtGeomArray);
        
        // For statistics, let's keep track of the most number of trees
        // that we generate
        synchronized(getClass()) {
            int count = totalTreeCount;
            if( count > maxTreeCount ) {
                maxTreeCount = count;
                log.info("New max blade count:" + (count / 2));
            }
        }   
    }

    /*
    private Geometry createBatch( List<Vector3f> points, List<Vector3f> normals,
                                  Geometry sourceGeom, Random random ) {        
        BatchTemplate bt = new BatchTemplate(sourceGeom, true);        
        Geometry result = bt.createBatch(points, normals, null, random);
        return result;                                          
    }

    private void createTreeMesh( List<Vector3f> points, List<Vector3f> normals, 
                                 List<ColorRGBA> colors, List<Float> sizes ) {
 
        long seed = (long)getXCell() << 32 | getYCell();
        Random random = new Random(seed);
 
        Node lod1 = (Node)treeTemplates[1].getChild(0);
        Geometry gTrunkLod1 = (Geometry)lod1.getChild(0);
        builtTrunkGeom = createBatch(points, normals, gTrunkLod1, random);
        builtTrunkGeom.setShadowMode(ShadowMode.CastAndReceive);
         
        Geometry gLeafLod1 = null;
        if( lod1.getQuantity() > 1 ) {
            gLeafLod1 = (Geometry)lod1.getChild(1);
            builtLeafGeom = createBatch(points, normals, gLeafLod1, random);
            builtLeafGeom.setShadowMode(ShadowMode.CastAndReceive);
        } else {
            builtLeafGeom = null;
        }
        
    } */

    public void apply() {
        release(geomArray);
        this.geomArray = builtGeomArray;
        for( Geometry g : geomArray ) {
            getZoneRoot().attachChild(g);
        }
/*         
        release(trunkGeom);
        release(leafGeom);
        this.trunkGeom = builtTrunkGeom;
        this.leafGeom = builtLeafGeom; 
        if( trunkGeom != null ) {
            getZoneRoot().attachChild(trunkGeom);
        }
        if( leafGeom != null ) {
            getZoneRoot().attachChild(leafGeom);
        }*/
    }

    public void release() {
        release(geomArray);
    }
 
    protected void release( Geometry[] array ) {
        if( array == null ) {
            return;
        }
        for( Geometry g : array ) {
            release(g);
        }
    }
 
    protected void release( Geometry geom ) {
        if( geom == null ) {
            return;
        }
        geom.removeFromParent();
        release(geom.getMesh());
    }
    
    protected void release( Mesh mesh ) {
        if( mesh == null ) {
            return;
        }
        for( VertexBuffer vb : mesh.getBufferList() ) {
            if( log.isTraceEnabled() ) {
                log.trace("--destroying buffer:" + vb);
            }
            BufferUtils.destroyDirectBuffer( vb.getData() );
        }
    }
    
    private class TreeType {
        Node treeTemplate;
        
        BatchTemplate[][] lodTemplates = new BatchTemplate[3][];
        
        public TreeType( Node treeTemplate ) {
            // Setup the different batch templates for each
            // LOD
            setLod(0, (Node)treeTemplate.getChild(0)); 
            setLod(1, (Node)treeTemplate.getChild(1)); 
            setLod(2, (Node)treeTemplate.getChild(2)); 
        }
        
        protected final void setLod( int lod, Node tree ) {
            lodTemplates[lod] = new BatchTemplate[tree.getQuantity()];
            for( int i = 0; i < tree.getQuantity(); i++ ) {
                Geometry geom = (Geometry)tree.getChild(i);
                lodTemplates[lod][i] = new BatchTemplate(geom, true);
            }
        } 

        public void addBatch( List<Geometry> results, int lod, List<BatchInstance> instances ) { 
                              
            BatchTemplate[] templates = lodTemplates[lod];
            for( BatchTemplate bt : templates ) {
                Geometry geom = bt.createBatch(instances);
                if( geom != null ) {
                    results.add(geom);
                }
            }
        }                
    }
    
    private class TreeBin {
        List<BatchInstance> instances;
        
        public TreeBin() {
            instances = new ArrayList<BatchInstance>();
        }
    }
 
    protected class FrequencyPlotter implements TriangleProcessor {
 
        Random random;   
        TreeBin[] bins;
        
        float threshold = FastMath.sin(FastMath.QUARTER_PI);
        Vector3f min;
        Vector3f max;
        Vector3f world;
        int processedTriangleCount;
        int binCount;

        public FrequencyPlotter( int binCount, Random random ) {
            this.binCount = binCount;
            this.bins = new TreeBin[binCount];
            this.random = random;
            
            for( int i = 0; i < binCount; i++ ) {
                bins[i] = new TreeBin();
            }
        }

        private boolean inZone( Vector3f v ) {
            if( v.x < min.x || v.y < min.y || v.z < min.z ) {
                return false;
            }
            if( v.x > max.x || v.y > max.y || v.z > max.z ) {
                return false;
            }
            return true;
        }

        public void processTriangle( Mesh mesh, int index, Triangle tri ) {
            if( tri.norms[0].y < threshold && tri.norms[1].y < threshold && tri.norms[2].y < threshold ) {
                return;
            }
            Vector3f[] verts = tri.verts;
 
            // Is the triangle even in this zone?           
            if( !inZone(verts[0]) || !inZone(verts[1]) || !inZone(verts[2]) ) {
                return;
            }
 
            // Yes, so we'll rasterize it
            processedTriangleCount++;
            rasterize(index, tri);           
        }

        private float orient2D( Vector3f a, Vector3f b, Vector3f c ) {
            return (b.x - a.x) * (c.z - a.z) - (b.z - a.z) * (c.x - a.x);
        }
        
        private float min( float a, float b, float c ) {
            if( a < b ) {
                if( a < c ) {
                    return a;
                } else {
                    return c;
                }
            } else {
                if( b < c ) {
                    return b;
                } else {
                    return c; 
                }
            }
        }

        private float max( float a, float b, float c ) {
            if( a > b ) {
                if( a > c ) {
                    return a;
                } else {
                    return c;
                }
            } else {
                if( b > c ) {
                    return b;
                } else {
                    return c; 
                }
            }
        }
        
        private void rasterize( int triIndex, Triangle tri ) {
 
            float testOffsetX = (triIndex % 2) * 0.01f;
            float testOffsetZ = (triIndex % 3) * 0.01f;
 
            byte[] noiseValues1 = new byte[4];
            byte[] noiseValues2 = new byte[4];
            byte[] noiseValues3 = new byte[4];
            Vector3f pOffset = new Vector3f();
 
            Vector3f[] verts = tri.verts;
            Vector3f[] norms = tri.norms;
 
            Vector3f vf0 = verts[0].clone();
            vf0.y = 0;
            Vector3f vf1 = verts[1].clone();
            vf1.y = 0;
            Vector3f vf2 = verts[2].clone();
            vf2.y = 0; 
 
            float resolution = 0.25f;
            float plotVariation = 0.1f;
            
            // Compute a bounding box... we'll iterate over all points
            // in the bounding box to see if they are in the triangle.
            // Not the most efficient way as we will check 2x as many
            // points as needed but it is simple. 
            float minX = min(verts[0].x, verts[1].x, verts[2].x);
            float minZ = min(verts[0].z, verts[1].z, verts[2].z);
            float maxX = max(verts[0].x, verts[1].x, verts[2].x);
            float maxZ = max(verts[0].z, verts[1].z, verts[2].z);
 
            // Now, quantize them to even resolutions
            minX -= minX % resolution;
            minZ -= minZ % resolution;
            maxX += maxX % resolution;
            maxZ += maxZ % resolution;
            
            // Offset them slightly to avoid many 'on edge' situations
            minX += 0.01f;
            minZ += 0.01f;
            maxX -= 0.01f;
            maxZ -= 0.01f;
 
            // Start at the Barycentric coordinates for the
            // min corner.
            Vector3f p = new Vector3f(minX, 0, minZ);
            
            Quaternion upRot = new Quaternion(); // reused in the inner loop
 
            // Rasterize
            for( p.z = minZ; p.z < maxZ; p.z += resolution ) {
                for( p.x = minX; p.x < maxX; p.x += resolution ) {
                    // Let's add some noise to the coordinates
                    noise.getHomogenous((world.x + (p.x - min.x)) * 0.5, 
                                        (world.z + (p.z - min.z)) * 0.5, 
                                        noiseValues1);
                    float xPlotOffset = (noiseValues1[2] & 0xff) / 255f - 0.5f;                                                              
                    float zPlotOffset = (noiseValues1[3] & 0xff) / 255f - 0.5f;
                    pOffset.set(p);
                    pOffset.addLocal(xPlotOffset, 0, zPlotOffset);                                                              
                         
                    // Need to find y at this location so calculate
                    // the barycentric coordinates
                    Vector3f v0 = vf2.subtract(vf0);
                    Vector3f v1 = vf1.subtract(vf0);
                    Vector3f v2 = pOffset.subtract(vf0);
            
                    float dot00 = v0.dot(v0);
                    float dot01 = v0.dot(v1);
                    float dot02 = v0.dot(v2);
                    float dot11 = v1.dot(v1);
                    float dot12 = v1.dot(v2);

                    float invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
                    float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
                    float v = (dot00 * dot12 - dot01 * dot02) * invDenom;                     
 
                    // Check to see if it is in the triangle                      
                    if( u >= 0 && v >= 0 && u + v < 1 ) {
                         
                        float y2 = verts[1].y - verts[0].y;
                        float y1 = verts[2].y - verts[0].y;
                        float y = verts[0].y + y1 * u + y2 * v;                       
 
                        // We'll be handing it to the points list which is
                        // why we need our own copy here.
                        Vector3f plot = pOffset.clone(); 
                        plot.y = y;
                        plot.subtractLocal(min);
                        plot.addLocal(testOffsetX, 0, testOffsetZ);                        
                        
                        Vector3f n2 = norms[1].subtract(norms[0]);             
                        Vector3f n1 = norms[2].subtract(norms[0]);
                        Vector3f newNorm = norms[0].add(n1.mult(u)).add(n2.mult(v));
                        newNorm.normalizeLocal();
 
                        // Is the normal facing up past 45 degrees?  This is the
                        // threshold for growing grass.   
                        if( newNorm.y >= threshold ) {
 
                            // Grab some noise that we'll use to perturb the location
                            // for the real noise lookup                       
                            noise.getHomogenous((world.x + plot.x) * 0.01, (world.z + plot.z) * 0.01, noiseValues1);
                            float xOffset = (noiseValues1[2] & 0xff) / 255f - 0.5f; 
                            float zOffset = (noiseValues1[3] & 0xff) / 255f - 0.5f;
                            
                            // And the noise to see if we plot grass here
                            noise.getHomogenous((world.x + plot.x) * 0.07 + xOffset, (world.z + plot.z) * 0.07 + zOffset, noiseValues2);
                            float offset1 = (noiseValues1[1] & 0xff) / 255f;
                            float offset2 = ((noiseValues2[1] & 0xff) / 255f) - 0.5f;
                            
                            float normalOffset = (newNorm.y - threshold) - ((1.0f - threshold) * 0.5f);
                            float offset = Math.min(1, Math.max(0, (offset1 + offset2 + normalOffset)));
                            
                            // Grab an alternate frequency
                            noise.getHomogenous((world.x + plot.x) * 0.11 + xOffset, (world.z + plot.z) * 0.11 + zOffset, noiseValues3);
                            float offset3 = ((noiseValues3[1] & 0xff) / 255f) - 0.5f;
                            float altOffset = Math.min(1, Math.max(0, (offset1 + offset3 + normalOffset)));
                            if( offset > 0.5 && altOffset > 0.5 ) {
 
                                // Trees are at a lower resolution
                                boolean tall = offset > 0.75 && altOffset > 0.5;                               
                                float treeResolution = resolution * 5;
                                if( tall ) {
                                    treeResolution *= 2;
                                }
                                if( (plot.x % treeResolution) < 0.1
                                    && (plot.z % treeResolution) < 0.1 ) {
 
                                    // Figure out which bin we should be in
                                    float binOffset = (offset - 0.5f) + (altOffset - 0.5f);
                                    int bin = (int)Math.round(binOffset * (binCount-1)); 
                                    
                                    BatchInstance instance = new BatchInstance();
                                    instance.position = plot;
                                    instance.scale = 1;
                                                       
                                    // Create the random but directed rotation
                                    Vector3f n = newNorm.addLocal(0, 1, 0).normalizeLocal();                                                
                                    Quaternion rot = new Quaternion();
                                    rot.fromAngles(0, FastMath.TWO_PI * random.nextFloat(), 0);
                                     
                                    // Make the quaternion's "up" be the normal provided
                                    float angle = Vector3f.UNIT_Y.angleBetween(n);
                                    if( Math.abs(angle) > 0 ) {
                                        Vector3f axis = Vector3f.UNIT_Y.cross(n).normalizeLocal();
                                        upRot.fromAngleNormalAxis(angle, axis);
                                        upRot.mult(rot, rot);
                                    }
                                                                                                                    
                                    instance.rotation = rot;
                                    
                                    bins[bin].instances.add(instance);
                                    /*if( tall ) {
                                        sizes.add(8.8f);
                                    } else {
                                        sizes.add(4.25f); 
                                    }
                                    points.add(plot);                                                
                                    Vector3f n = newNorm.add(0, 1, 0).normalizeLocal();                                                 
                                    normals.add(n);
                                    colors.add(ColorRGBA.White);*/
                                    //bins[bin].points.add(plot);
                                    //Vector3f n = newNorm.add(0, 1, 0).normalizeLocal();                                                 
                                    //bins[bin].upVectors.add(n);
                                    //bins[bin].sizes.add(binOffset);
                                }
                            }                           
                        } 
                    }
                }                
            }           
        }   
    }

    
    public static class Factory implements ZoneFactory {
        private Material material;
        private BilinearArray noise;
        private Node[] treeTemplates;
        
        public Factory( Material material, BilinearArray noise, Node... treeTemplates ) {
            this.noise = noise;
            this.material = material;
            this.treeTemplates = treeTemplates;
        }
        
        public Zone createZone( PagedGrid pg, int xCell, int yCell, int zCell ) {
            Zone result = new TreeZone(pg.getGrid(), material, noise, xCell, yCell, zCell, treeTemplates);
            return result;   
        }        
    }
}
