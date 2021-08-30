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
public:

    void extractInverseBindMatrices(const tinygltf::Node& node, const tinygltf::Model& model);
    void extractGlobalTransformation(const tinygltf::Node& rootNode);
    void extractJointTransformation(const tinygltf::Skin& skin, const tinygltf::Model& model);
    void extractMeshPrimitiveData(const tinygltf::Node& node, const tinygltf::Model& model);
    void copyToJava(JNIEnv* env, jobject obj);

private:

    std::vector<int> indices;
    std::vector<float> vertices;
    std::vector<float> normals;
    std::vector<float> texcoords;
    std::vector<float> joints;
    std::vector<float> weights;

    std::vector<float> scale;
    std::vector<float> translation;
    std::vector<float> rotation;

    std::vector<float> jointTranslation;
    std::vector<float> jointRotation;
    std::vector<float> inverseBindMatrices;
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

void aMesh::extractInverseBindMatrices(const tinygltf::Node& node, const tinygltf::Model& model)
{
    if (!inverseBindMatrices.empty()) { return; }

    const tinygltf::Skin& skin = model.skins[node.skin];
    const tinygltf::Accessor& accessor = model.accessors[skin.inverseBindMatrices];
    const tinygltf::BufferView &bufferView = model.bufferViews[accessor.bufferView];
    const tinygltf::Buffer &buffer = model.buffers[bufferView.buffer];

    float *data = (float *) (buffer.data.data() + bufferView.byteOffset);
    std::vector<float> extractedData{data,
                                     data + bufferView.byteLength / sizeof(float)};

    inverseBindMatrices.insert(std::end(inverseBindMatrices), std::begin(extractedData),
                    std::end(extractedData));
}

void aMesh::extractGlobalTransformation(const tinygltf::Node& rootNode)
{
    scale.assign(rootNode.scale.begin(), rootNode.scale.end());
    translation.assign(rootNode.translation.begin(), rootNode.translation.end());
    rotation.assign(rootNode.rotation.begin(), rootNode.rotation.end());
}

void aMesh::extractJointTransformation(const tinygltf::Skin &skin, const tinygltf::Model& model)
{
    for (auto jointIndex : skin.joints)
    {
        const tinygltf::Node& jointNode = model.nodes[jointIndex];

        jointTranslation.insert(jointTranslation.end(), jointNode.translation.begin(), jointNode.translation.end());
        jointRotation.insert(jointRotation.end(), jointNode.rotation.begin(), jointNode.rotation.end());
    }
}

void aMesh::extractMeshPrimitiveData(const tinygltf::Node& node, const tinygltf::Model& model)
{
    if (node.mesh >= 0)
    {
        const tinygltf::Mesh &mesh = model.meshes[node.mesh];

        if (node.skin >= 0) {
            extractInverseBindMatrices(node, model);
        }

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

                unsigned char *intData = (unsigned char *) (buffer.data.data() + bufferView.byteOffset);
                std::vector<unsigned char> extractedIntData{intData,
                                                            intData + bufferView.byteLength / sizeof(unsigned char)};

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
        extractMeshPrimitiveData(model.nodes[node.children[childrenIndex]], model);
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
    mesh.extractJointTransformation(model.skins[0], model);
    mesh.extractMeshPrimitiveData(rootNode, model);

//    for (size_t nodeIndex = 0; nodeIndex < scene.nodes.size(); nodeIndex++)
//    {
//        const tinygltf::Node& node = model.nodes[scene.nodes[nodeIndex]];
//        object.retrieveNode(node, model);
//    }

    mesh.copyToJava(env, obj);
}

void aMesh::copyToJava(JNIEnv* env, jobject obj)
{
    // Create the object of the class UserData
    jclass loaderClass = env->GetObjectClass(obj);

    jfieldID indicesField = env->GetFieldID(loaderClass, "indices", "[I");
    jfieldID verticesField = env->GetFieldID(loaderClass, "vertices", "[F");
    jfieldID normalsField = env->GetFieldID(loaderClass, "normals", "[F");
    jfieldID texcoordsField = env->GetFieldID(loaderClass, "texcoords", "[F");
    jfieldID jointsField = env->GetFieldID(loaderClass, "joints", "[F");
    jfieldID weightsField = env->GetFieldID(loaderClass, "weights", "[F");

    jfieldID scaleField = env->GetFieldID(loaderClass, "scale", "[F");
    jfieldID translationField = env->GetFieldID(loaderClass, "translation", "[F");
    jfieldID rotationField = env->GetFieldID(loaderClass, "rotation", "[F");

    jfieldID jointTranslationField = env->GetFieldID(loaderClass, "jointTranslation", "[F");
    jfieldID jointRotationField = env->GetFieldID(loaderClass, "jointRotation", "[F");
    jfieldID inverseBindMatricesField = env->GetFieldID(loaderClass, "inverseBindMatrices", "[F");

    copyToJavaClass<int>(env, obj, indices, indicesField);
    copyToJavaClass<float>(env, obj, vertices, verticesField);
    copyToJavaClass<float>(env, obj, normals, normalsField);
    copyToJavaClass<float>(env, obj, texcoords, texcoordsField);
    copyToJavaClass<float>(env, obj, joints, jointsField);
    copyToJavaClass<float>(env, obj, weights, weightsField);

    copyToJavaClass<float>(env, obj, scale, scaleField);
    copyToJavaClass<float>(env, obj, translation, translationField);
    copyToJavaClass<float>(env, obj, rotation, rotationField);

    copyToJavaClass<float>(env, obj, jointTranslation, jointTranslationField);
    copyToJavaClass<float>(env, obj, jointRotation, jointRotationField);
    copyToJavaClass<float>(env, obj, inverseBindMatrices, inverseBindMatricesField);
}
