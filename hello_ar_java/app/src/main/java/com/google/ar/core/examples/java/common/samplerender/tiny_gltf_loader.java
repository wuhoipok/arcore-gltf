package com.google.ar.core.examples.java.common.samplerender;

import java.util.List;

public class tiny_gltf_loader
{

    private List<Float> vertices;
    private List<Float> normals;
    private List<Float> texcoords;

    static
    {
        System.loadLibrary("tiny_gltf");
    }

    public native void loadBinaryFromFile();
}
