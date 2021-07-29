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

struct meshData
{
    std::vector<int> indices;
    std::vector<float> vertices;
    std::vector<float> normals;
    std::vector<float> texcoords;
};

void retrieveNode(const tinygltf::Node& node, const tinygltf::Model& model, meshData& object)
{
    const tinygltf::Mesh& mesh = model.meshes[node.mesh];

    if (mesh.primitives.size() > 0)
    {
        for (size_t primitiveIndex = 0; primitiveIndex < mesh.primitives.size(); primitiveIndex++)
        {
            const tinygltf::Primitive &primitive = mesh.primitives[primitiveIndex];
            const tinygltf::Accessor &indexAccessor = model.accessors[primitive.indices];
            const tinygltf::BufferView &indexBufferView = model.bufferViews[indexAccessor.bufferView];
            const tinygltf::Buffer &indexBuffer = model.buffers[indexBufferView.buffer];

            unsigned short* data = (unsigned short*) (indexBuffer.data.data() + indexBufferView.byteOffset);
            std::vector<unsigned short> indexData { data, data + indexBufferView.byteLength / sizeof(unsigned short) };

            object.indices.insert(std::end(object.indices), std::begin(indexData),
                                  std::end(indexData));

            for (const auto &attrib : primitive.attributes)
            {
                const tinygltf::Accessor &accessor = model.accessors[attrib.second];
                const tinygltf::BufferView &bufferView = model.bufferViews[accessor.bufferView];
                const tinygltf::Buffer &buffer = model.buffers[bufferView.buffer];

                float* data = (float*) (buffer.data.data() + bufferView.byteOffset);
                std::vector<float> extractedData { data, data + bufferView.byteLength / sizeof(float) };

                if (attrib.first.compare("POSITION") == 0)
                {
                    object.vertices.insert(std::end(object.vertices), std::begin(extractedData),
                                           std::end(extractedData));
                }
                else if (attrib.first.compare("NORMAL") == 0)
                {
                    object.normals.insert(std::end(object.normals), std::begin(extractedData),
                                          std::end(extractedData));
                }
                else if (attrib.first.compare("TEXCOORD_0") == 0)
                {
                    object.texcoords.insert(std::end(object.texcoords), std::begin(extractedData),
                                            std::end(extractedData));
                }
                else
                {
                    continue;
                }
            }
        }
    }

    for (size_t childrenIndex = 0; childrenIndex < node.children.size(); childrenIndex++)
    {
        retrieveNode(model.nodes[node.children[childrenIndex]], model, object);
    }
}

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

    meshData object;

    // parse the data
    const tinygltf::Scene& scene = model.scenes[model.defaultScene];

    for (size_t nodeIndex = 0; nodeIndex < scene.nodes.size(); nodeIndex++)
    {
        const tinygltf::Node& node = model.nodes[scene.nodes[nodeIndex]];

        retrieveNode(node, model, object);
    }

    // Create the object of the class UserData
    jclass loaderClass = env->GetObjectClass(obj);

    jbyteArray indexArray = env->NewByteArray(object.indices.size());
    env->SetByteArrayRegion (indexArray, 0, object.indices.size(), reinterpret_cast<jbyte*>(object.indices.data()));

    jbyteArray vertexArray = env->NewByteArray(object.vertices.size());
    env->SetByteArrayRegion (vertexArray, 0, object.vertices.size(), reinterpret_cast<jbyte*>(object.vertices.data()));

    jbyteArray normalArray = env->NewByteArray(object.normals.size());
    env->SetByteArrayRegion (normalArray, 0, object.normals.size(), reinterpret_cast<jbyte*>(object.normals.data()));

    jbyteArray texcoordArray = env->NewByteArray(object.texcoords.size());
    env->SetByteArrayRegion (texcoordArray, 0, object.texcoords.size(), reinterpret_cast<jbyte*>(object.texcoords.data()));

    jfieldID indexField = env->GetFieldID(loaderClass, "indices", "[B");
    jfieldID vertexField = env->GetFieldID(loaderClass, "vertices", "[B");
    jfieldID normalField = env->GetFieldID(loaderClass, "normals", "[B");
    jfieldID texcoordField = env->GetFieldID(loaderClass, "texcoords", "[B");

    jbyteArray indexBytes = static_cast<jbyteArray>(env->GetObjectField(obj, indexField));
    jbyteArray vertexBytes = static_cast<jbyteArray>(env->GetObjectField(obj, vertexField));
    jbyteArray normalBytes = static_cast<jbyteArray>(env->GetObjectField(obj, normalField));
    jbyteArray texcoordBytes = static_cast<jbyteArray>(env->GetObjectField(obj, texcoordField));

    jbyte* b = env->GetByteArrayElements(indexBytes, NULL);
    memcpy(indexArray, b, object.indices.size());
    env->ReleaseByteArrayElements(indexBytes, b, 0);

    b = env->GetByteArrayElements(vertexBytes, NULL);
    memcpy(vertexArray, b, object.vertices.size());
    env->ReleaseByteArrayElements(vertexBytes, b, 0);

    b = env->GetByteArrayElements(normalBytes, NULL);
    memcpy(normalArray, b, object.normals.size());
    env->ReleaseByteArrayElements(normalBytes, b, 0);

    b = env->GetByteArrayElements(texcoordBytes, NULL);
    memcpy(texcoordArray, b, object.texcoords.size());
    env->ReleaseByteArrayElements(texcoordBytes, b, 0);
}
