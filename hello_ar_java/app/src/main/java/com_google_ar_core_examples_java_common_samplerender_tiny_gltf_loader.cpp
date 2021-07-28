//
// Created by sean on 2021/7/26.
//
#include "com_google_ar_core_examples_java_common_samplerender_tiny_gltf_loader.h"

#define TINYGLTF_IMPLEMENTATION
#define TINYGLTF_ANDROID_LOAD_FROM_ASSETS
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION

#include "tiny_gltf.h"

#include <iostream>
#include <android/asset_manager_jni.h>

using namespace tinygltf;

JNIEXPORT void JNICALL Java_com_google_ar_core_examples_java_common_samplerender_tiny_1gltf_1loader_loadBinaryFromFile
(JNIEnv* env, jobject obj, jstring filename, jobject assetManager)
{
    // load the model
    Model model;
    TinyGLTF loader;
    std::string err;
    std::string warn;

    tinygltf::asset_manager = AAssetManager_fromJava(env, assetManager);

    // bool ret = loader.LoadASCIIFromFile(&model, &err, &warn, argv[1]);
    std::string file = env->GetStringUTFChars(filename, (jboolean*) false);
    bool ret = loader.LoadBinaryFromFile(&model, &err, &warn, file); // for binary glTF(.glb)

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

    jbyteArray indexArray = env->NewByteArray(indices.size());
    env->SetByteArrayRegion (indexArray, 0, indices.size(), reinterpret_cast<jbyte*>(indices.data()));

    jbyteArray vertexArray = env->NewByteArray( vertices.size());
    env->SetByteArrayRegion (vertexArray, 0, vertices.size(), reinterpret_cast<jbyte*>(vertices.data()));

    jbyteArray normalArray = env->NewByteArray(normals.size());
    env->SetByteArrayRegion (normalArray, 0, normals.size(), reinterpret_cast<jbyte*>(normals.data()));

    jbyteArray texcoordArray = env->NewByteArray(texcoords.size());
    env->SetByteArrayRegion (texcoordArray, 0, texcoords.size(), reinterpret_cast<jbyte*>(texcoords.data()));

    jfieldID indexField = env->GetFieldID(loaderClass, "indices", "[B");
    jfieldID vertexField = env->GetFieldID(loaderClass, "vertices", "[B");
    jfieldID normalField = env->GetFieldID(loaderClass, "normals", "[B");
    jfieldID texcoordField = env->GetFieldID(loaderClass, "texcoords", "[B");

    jbyteArray indexBytes = static_cast<jbyteArray>(env->GetObjectField(obj, indexField));
    jbyteArray vertexBytes = static_cast<jbyteArray>(env->GetObjectField(obj, vertexField));
    jbyteArray normalBytes = static_cast<jbyteArray>(env->GetObjectField(obj, normalField));
    jbyteArray texcoordBytes = static_cast<jbyteArray>(env->GetObjectField(obj, texcoordField));

    jbyte* b = env->GetByteArrayElements(indexBytes, NULL);
    memcpy(indexArray, b, indices.size());
    env->ReleaseByteArrayElements(indexBytes, b, 0);

    b = env->GetByteArrayElements(vertexBytes, NULL);
    memcpy(vertexArray, b, vertices.size());
    env->ReleaseByteArrayElements(vertexBytes, b, 0);

    b = env->GetByteArrayElements(normalBytes, NULL);
    memcpy(normalArray, b, normals.size());
    env->ReleaseByteArrayElements(normalBytes, b, 0);

    b = env->GetByteArrayElements(texcoordBytes, NULL);
    memcpy(texcoordArray, b, texcoords.size());
    env->ReleaseByteArrayElements(texcoordBytes, b, 0);
}
