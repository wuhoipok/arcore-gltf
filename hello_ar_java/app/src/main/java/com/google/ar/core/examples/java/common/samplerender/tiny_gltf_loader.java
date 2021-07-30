package com.google.ar.core.examples.java.common.samplerender;

import android.content.res.AssetManager;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;

public class tiny_gltf_loader
{

    private byte[] indices;
    private byte[] vertices;
    private byte[] normals;
    private byte[] texcoords;

    static
    {
        System.loadLibrary("tiny_gltf");
    }

    public IntBuffer getIndices()
    {
        return ByteBuffer.allocateDirect(indices.length).put(indices).asIntBuffer();
    }

    public FloatBuffer getVertices()
    {
        return ByteBuffer.allocateDirect(vertices.length).put(vertices).asFloatBuffer();
    }

    public FloatBuffer getNormals()
    {
        return ByteBuffer.allocateDirect(normals.length).put(normals).asFloatBuffer();
    }

    public FloatBuffer getTexcoords()
    {
        return ByteBuffer.allocateDirect(texcoords.length).put(texcoords).asFloatBuffer();
    }

    public native void loadBinaryFromFile(String filename, AssetManager assetManager);
}
