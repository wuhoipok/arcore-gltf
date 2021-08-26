//
// Created by sean on 2021/7/26.
//
#include "com_google_ar_core_examples_java_common_samplerender_tiny_gltf_loader.h"

#define TINYGLTF_IMPLEMENTATION
#define TINYGLTF_ANDROID_LOAD_FROM_ASSETS
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION

#include "tiny_gltf.h"

#if defined(__arm__)
    #include "libs/curl/armeabi-v7a/include/curl/curl.h"
#elif defined(__i386__)
    #include "libs/curl/x86/include/curl/curl.h"
#elif defined(__x86_64__)
    #include "libs/curl/x86_64/include/curl/curl.h"
#elif defined(__aarch64__)
    #include "libs/curl/arm64-v8a/include/curl/curl.h"
#endif

#include <iostream>
#include <android/asset_manager_jni.h>

using namespace tinygltf;

// single mesh struct
struct aMesh
{
    std::vector<int> indices;
    std::vector<float> vertices;
    std::vector<float> normals;
    std::vector<float> texcoords;
    std::vector<int> joints;
    std::vector<float> weights;

    std::vector<float> globalScale;
    std::vector<float> globalTranslation;
    std::vector<float> globalRotation;

    void extractGlobalTransformation(const tinygltf::Node& rootNode);
    void extractData(const tinygltf::Node& node, const tinygltf::Model& model);
};

template <typename T>
void copyToJavaClass(JNIEnv* env, jobject obj, const std::vector<T>& data, jfieldID fieldId)
{

}

template <>
void copyToJavaClass<int>(JNIEnv* env, jobject obj, const std::vector<int>& data, jfieldID fieldId)
{
    const jint* intArray = data.data();

    jintArray outIntArray = env->NewIntArray(data.size());
    env->SetIntArrayRegion(outIntArray, 0, data.size(), intArray);

    env->SetObjectField(obj, fieldId, outIntArray);
}

template <>
void copyToJavaClass<float>(JNIEnv* env, jobject obj, const std::vector<float>& data, jfieldID fieldId)
{
    const jfloat* floatArray = data.data();

    jfloatArray outFloatArray = env->NewFloatArray(data.size());
    env->SetFloatArrayRegion(outFloatArray, 0, data.size(), floatArray);

    env->SetObjectField(obj, fieldId, outFloatArray);
}

void aMesh::extractGlobalTransformation(const tinygltf::Node& rootNode)
{
    globalScale.assign(rootNode.scale.begin(), rootNode.scale.end());
    globalTranslation.assign(rootNode.translation.begin(), rootNode.translation.end());
    globalRotation.assign(rootNode.rotation.begin(), rootNode.rotation.end());
}

void aMesh::extractData(const tinygltf::Node& node, const tinygltf::Model& model)
{
    if (node.mesh >= 0)
    {
        const tinygltf::Mesh &mesh = model.meshes[node.mesh];

        for (size_t primitiveIndex = 0; primitiveIndex < mesh.primitives.size(); primitiveIndex++) {
            const tinygltf::Primitive &primitive = mesh.primitives[primitiveIndex];
            const tinygltf::Accessor &indexAccessor = model.accessors[primitive.indices];
            const tinygltf::BufferView &indexBufferView = model.bufferViews[indexAccessor.bufferView];
            const tinygltf::Buffer &indexBuffer = model.buffers[indexBufferView.buffer];

            unsigned short *data = (unsigned short *) (indexBuffer.data.data() +
                                                       indexBufferView.byteOffset);
            std::vector<unsigned short> indexData{data, data + indexBufferView.byteLength /
                                                               sizeof(unsigned short)};

            indices.insert(std::end(indices), std::begin(indexData),
                           std::end(indexData));

            for (const auto &attrib : primitive.attributes) {
                const tinygltf::Accessor &accessor = model.accessors[attrib.second];
                const tinygltf::BufferView &bufferView = model.bufferViews[accessor.bufferView];
                const tinygltf::Buffer &buffer = model.buffers[bufferView.buffer];

                float *data = (float *) (buffer.data.data() + bufferView.byteOffset);
                std::vector<float> extractedData{data,
                                                 data + bufferView.byteLength / sizeof(float)};

                int *intData = (int *) (buffer.data.data() + bufferView.byteOffset);
                std::vector<int> extractedIntData{intData,
                                                  intData + bufferView.byteLength / sizeof(int)};

                if (attrib.first.compare("POSITION") == 0) {
                    vertices.insert(std::end(vertices), std::begin(extractedData),
                                    std::end(extractedData));
                } else if (attrib.first.compare("NORMAL") == 0) {
                    normals.insert(std::end(normals), std::begin(extractedData),
                                   std::end(extractedData));
                } else if (attrib.first.compare("TEXCOORD_0") == 0) {
                    texcoords.insert(std::end(texcoords), std::begin(extractedData),
                                     std::end(extractedData));
                } else if (attrib.first.compare("JOINTS_0") == 0) {
                    joints.insert(std::end(joints), std::begin(extractedIntData),
                                  std::end(extractedIntData));
                } else if (attrib.first.compare("WEIGHTS_0") == 0) {
                    weights.insert(std::end(weights), std::begin(extractedData),
                                   std::end(extractedData));
                }
            }
        }
    }

    for (size_t childrenIndex = 0; childrenIndex < node.children.size(); childrenIndex++)
    {
        extractData(model.nodes[node.children[childrenIndex]], model);
    }
}

static size_t writeCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

JNIEXPORT void JNICALL Java_com_google_ar_core_examples_java_common_samplerender_tiny_1gltf_1loader_loadBinaryFromFile
(JNIEnv* env, jobject obj, jstring filename)
{
    const char* nativeFilename = env->GetStringUTFChars(filename, 0);

    // load the model
    Model model{};
    TinyGLTF loader{};
    std::string err;
    std::string warn;

    CURL *curl;
    CURLcode res;
    std::string readBuffer;

    curl = curl_easy_init();

    if (curl)
    {
        curl_easy_setopt(curl, CURLOPT_URL, nativeFilename);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, false);
        res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);
    }

    env->ReleaseStringUTFChars(filename, nativeFilename);

    std::vector<unsigned char> file { readBuffer.begin(), readBuffer.end() };
    bool ret = loader.LoadBinaryFromFile(&model, &err, &warn, file, "human/glb");

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

    aMesh mesh;

    // parse the data
    const tinygltf::Scene& scene = model.scenes[model.defaultScene];

    const tinygltf::Node& rootNode = model.nodes[scene.nodes[0]];
    mesh.extractGlobalTransformation(rootNode);
    mesh.extractData(rootNode, model);

//    for (size_t nodeIndex = 0; nodeIndex < scene.nodes.size(); nodeIndex++)
//    {
//        const tinygltf::Node& node = model.nodes[scene.nodes[nodeIndex]];
//        object.retrieveNode(node, model);
//    }

    // Create the object of the class UserData
    jclass loaderClass = env->GetObjectClass(obj);

    jfieldID indexField = env->GetFieldID(loaderClass, "indices", "[I");
    jfieldID vertexField = env->GetFieldID(loaderClass, "vertices", "[F");
    jfieldID normalField = env->GetFieldID(loaderClass, "normals", "[F");
    jfieldID texcoordField = env->GetFieldID(loaderClass, "texcoords", "[F");
    jfieldID jointField = env->GetFieldID(loaderClass, "joints", "[I");
    jfieldID weightField = env->GetFieldID(loaderClass, "weights", "[F");

    jfieldID scaleField = env->GetFieldID(loaderClass, "scale", "[F");
    jfieldID translationField = env->GetFieldID(loaderClass, "translation", "[F");
    jfieldID rotationField = env->GetFieldID(loaderClass, "rotation", "[F");

    copyToJavaClass<int>(env, obj, mesh.indices, indexField);
    copyToJavaClass<float>(env, obj, mesh.vertices, vertexField);
    copyToJavaClass<float>(env, obj, mesh.normals, normalField);
    copyToJavaClass<float>(env, obj, mesh.texcoords, texcoordField);
    copyToJavaClass<int>(env, obj, mesh.joints, jointField);
    copyToJavaClass<float>(env, obj, mesh.weights, weightField);

    copyToJavaClass<float>(env, obj, mesh.globalScale, scaleField);
    copyToJavaClass<float>(env, obj, mesh.globalTranslation, translationField);
    copyToJavaClass<float>(env, obj, mesh.globalRotation, rotationField);
}
