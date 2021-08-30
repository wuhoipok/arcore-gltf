package com.google.ar.core.examples.java.common.samplerender;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.util.Arrays;

public class tiny_gltf_loader
{

    public native void loadBinaryFromFile(String filename);

    public IntBuffer getIndices()
    {
        IntBuffer intBuffer = createDirectIntBuffer(4 * indices.length);
        intBuffer.put(indices).position(0);
        return intBuffer;
    }

    public FloatBuffer getVertices()
    {
        FloatBuffer floatBuffer = createDirectFloatBuffer(4 * vertices.length);
        floatBuffer.put(vertices).position(0);
        return floatBuffer;
    }

    public FloatBuffer getNormals()
    {
        FloatBuffer floatBuffer = createDirectFloatBuffer(4 * normals.length);
        floatBuffer.put(normals).position(0);
        return floatBuffer;
    }

    public FloatBuffer getTexcoords()
    {
        FloatBuffer floatBuffer = createDirectFloatBuffer(4 * texcoords.length);
        floatBuffer.put(texcoords).position(0);
        return floatBuffer;
    }

    public FloatBuffer getJoints()
    {
        FloatBuffer floatBuffer = createDirectFloatBuffer(4 * joints.length);
        floatBuffer.put(joints).position(0);
        return floatBuffer;
    }
    public FloatBuffer getWeights()
    {
        FloatBuffer floatBuffer = createDirectFloatBuffer(4 * weights.length);
        floatBuffer.put(weights).position(0);
        return floatBuffer;
    }

    public float[] getScale() { return scale; }
    public float[] getTranslation() { return translation; }
    public float[] getRotation() { return rotation; }

    public float[] getJointTranslation(int index) { return Arrays.copyOfRange(jointTranslation, 3 * index, 3 * (index + 1)); }
    public float[] getJointRotation(int index) { return Arrays.copyOfRange(jointRotation, 4 * index, 4 * (index + 1)); }
    public float[] getInverseBindMatrices(int index) { return Arrays.copyOfRange(inverseBindMatrices, 16 * index, 16 * (index + 1)); }

    private int[] indices;
    private float[] vertices;
    private float[] normals;
    private float[] texcoords;
    private float[] joints;
    private float[] weights;

    private float[] scale;
    private float[] translation;
    private float[] rotation;

    private float[] jointTranslation;
    private float[] jointRotation;
    private float[] inverseBindMatrices;

    static
    {
        System.loadLibrary("tiny_gltf");
    }

    private static IntBuffer createDirectIntBuffer(int size)
    {
        return ByteBuffer.allocateDirect(size)
                .order(ByteOrder.nativeOrder())
                .asIntBuffer();
    }

    private static FloatBuffer createDirectFloatBuffer(int size)
    {
        return ByteBuffer.allocateDirect(size)
                .order(ByteOrder.nativeOrder())
                .asFloatBuffer();
    }
}
