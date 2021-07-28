package com.google.ar.core.examples.java.common.samplerender;

import android.content.res.AssetManager;

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

    public byte[] getIndices() { return indices; }

    public byte[] getVertices() { return vertices; }

    public byte[] getNormals()
    {
        return normals;
    }

    public byte[] getTexcoords() { return texcoords; }

    public native void loadBinaryFromFile(String filename, AssetManager assetManager);
}
