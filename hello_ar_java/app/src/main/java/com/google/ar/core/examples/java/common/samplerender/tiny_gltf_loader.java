package com.google.ar.core.examples.java.common.samplerender;

import android.content.res.AssetManager;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;

public class tiny_gltf_loader
{

    private int[] indices;
    private float[] vertices;
    private float[] normals;
    private float[] texcoords;

    static
    {
        System.loadLibrary("tiny_gltf");
    }

    public IntBuffer getIndices()
    {
        ByteBuffer buffer = ByteBuffer.allocateDirect(4 * indices.length);
        IntBuffer intBuffer = buffer.order(ByteOrder.nativeOrder()).asIntBuffer();
        intBuffer.put(indices).position(0);
        return intBuffer;
    }

    public FloatBuffer getVertices()
    {
        ByteBuffer buffer = ByteBuffer.allocateDirect(4 * vertices.length);
        FloatBuffer floatBuffer = buffer.order(ByteOrder.nativeOrder()).asFloatBuffer();
        floatBuffer.put(vertices).position(0);
        return floatBuffer;
    }

    public FloatBuffer getNormals()
    {
        ByteBuffer buffer = ByteBuffer.allocateDirect(4 * normals.length);
        FloatBuffer floatBuffer = buffer.order(ByteOrder.nativeOrder()).asFloatBuffer();
        floatBuffer.put(normals).position(0);
        return floatBuffer;
    }

    public FloatBuffer getTexcoords()
    {
        ByteBuffer buffer = ByteBuffer.allocateDirect(4 * texcoords.length);
        FloatBuffer floatBuffer = buffer.order(ByteOrder.nativeOrder()).asFloatBuffer();
        floatBuffer.put(texcoords).position(0);
        return floatBuffer;
    }

    public native void loadBinaryFromFile(String filename, AssetManager assetManager);
}
