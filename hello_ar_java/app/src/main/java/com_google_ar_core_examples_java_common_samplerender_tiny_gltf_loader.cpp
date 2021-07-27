//
// Created by sean on 2021/7/26.
//
#include "com_google_ar_core_examples_java_common_samplerender_tiny_gltf_loader.h"

#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "tiny_gltf.h"

#include <iostream>

using namespace tinygltf;

JNIEXPORT void JNICALL Java_com_google_ar_core_examples_java_common_samplerender_tiny_1gltf_1loader_loadBinaryFromFile
(JNIEnv* env, jobject obj, jstring filename)
{
    // load the model
    Model model;
    TinyGLTF loader;
    std::string err;
    std::string warn;

    // bool ret = loader.LoadASCIIFromFile(&model, &err, &warn, argv[1]);
    bool ret = loader.LoadBinaryFromFile(&model, &err, &warn,env->GetStringUTFChars(filename, (jboolean*) false)); // for binary glTF(.glb)

    if (!warn.empty())
    {
        printf("Warn: %s\n", warn.c_str());
    }

    if (!err.empty())
    {
        printf("Err: %s\n", err.c_str());
    }

    if (!ret)
    {
        printf("Failed to parse glTF\n");
        return;
    }

    // parse the data
    const tinygltf::Scene& scene = model.scenes[model.defaultScene];

    std::vector<int> indices;
    std::vector<float> vertices;
    std::vector<float> normals;
    std::vector<float> texcoords;

    for (size_t nodeIndex = 0; nodeIndex < scene.nodes.size(); nodeIndex++)
    {
        const tinygltf::Node& node = model.nodes[scene.nodes[nodeIndex]];
        const tinygltf::Mesh& mesh = model.meshes[node.mesh];

        for (size_t primitiveIndex = 0; primitiveIndex < mesh.primitives.size(); primitiveIndex++)
        {
            const tinygltf::Primitive& primitive = mesh.primitives[primitiveIndex];
            const tinygltf::Accessor& indexAccessor = model.accessors[primitive.indices];
            const tinygltf::BufferView& indexBufferView = model.bufferViews[indexAccessor.bufferView];
            const tinygltf::Buffer& indexBuffer = model.buffers[indexBufferView.buffer];

            std::vector<int> indexData (indexBuffer.data.begin() + indexBufferView.byteOffset, indexBuffer.data.begin() + indexBufferView.byteOffset + indexBufferView.byteLength);

            indices.insert(std::end(indices), std::begin(indexData), std::end(indexData));

            for (const auto& attrib : primitive.attributes)
            {
                const tinygltf::Accessor& accessor = model.accessors[attrib.second];
                const tinygltf::BufferView& bufferView = model.bufferViews[accessor.bufferView];
                const tinygltf::Buffer& buffer = model.buffers[bufferView.buffer];

                std::vector<float> data (buffer.data.begin() + bufferView.byteOffset, buffer.data.begin() + bufferView.byteOffset + bufferView.byteLength);

                if (attrib.first.compare("POSITION") == 0)
                {
                    vertices.insert(std::end(vertices), std::begin(data), std::end(data));
                }
                else if (attrib.first.compare("NORMAL") == 0)
                {
                    normals.insert(std::end(normals), std::begin(data), std::end(data));
                }
                else if (attrib.first.compare("TEXCOORD_0") == 0)
                {
                    texcoords.insert(std::end(texcoords), std::begin(data), std::end(data));
                }
                else
                {
                    continue;
                }
            }
        }
    }

    // Create the object of the class UserData
    jclass loaderClass = env->GetObjectClass(obj);

    // Get the UserData fields to be set
    jfieldID inidexField = env->GetFieldID(loaderClass , "indices", "Ljava/nio/IntBuffer;");
    jfieldID vertexField = env->GetFieldID(loaderClass , "vertices", "Ljava/nio/FloatBuffer;");
    jfieldID normalField = env->GetFieldID(loaderClass , "normals", "Ljava/nio/FloatBuffer;");
    jfieldID texcoordField = env->GetFieldID(loaderClass , "texcoords", "Ljava/nio/FloatBuffer;");

    env->SetObjectField(obj, inidexField, indices.data());
    env->SetObjectField(obj, vertexField, vertices.data());
    env->SetObjectField(obj, normalField, normals.data());
    env->SetObjectField(obj, texcoordField, texcoords.data());
}
