package com.google.ar.core.examples.java.common.samplerender;

import java.nio.FloatBuffer;
import java.nio.IntBuffer;

public class tiny_gltf_loader
{

    private IntBuffer indices;
    private FloatBuffer vertices;
    private FloatBuffer normals;
    private FloatBuffer texcoords;

    static
    {
        System.loadLibrary("tiny_gltf");
    }

    public IntBuffer getIndices() { return indices; }

    public FloatBuffer getVertices() { return vertices; }

    public FloatBuffer getNormals()
    {
        return normals;
    }

    public FloatBuffer getTexcoords() { return texcoords; }

    public native void loadBinaryFromFile(String filename);
}
