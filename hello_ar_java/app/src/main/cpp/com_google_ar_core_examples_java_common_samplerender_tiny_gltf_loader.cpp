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

#include <android/asset_manager_jni.h>

using namespace tinygltf;

namespace Mat4
{
    typedef std::array<float, 16> mat4;
    typedef std::array<float, 3> vec3;
    typedef std::array<float, 4> vec4;

    mat4 multiplyMM(const mat4& a, const mat4 b)
    {
        mat4 newMat;

        newMat[0] = a[0] * b[0] + a[4] * b[1] + a[8] * b[2] + a[12] * b[3];
        newMat[1] = a[1] * b[0] + a[5] * b[1] + a[9] * b[2] + a[13] * b[3];
        newMat[2] = a[2] * b[0] + a[6] * b[1] + a[10] * b[2] + a[14] * b[3];
        newMat[3] = a[3] * b[0] + a[7] * b[1] + a[11] * b[2] + a[15] * b[3];
        newMat[4] = a[0] * b[4] + a[4] * b[5] + a[8] * b[6] + a[12] * b[7];
        newMat[5] = a[1] * b[4] + a[5] * b[5] + a[9] * b[6] + a[13] * b[7];
        newMat[6] = a[2] * b[4] + a[6] * b[5] + a[10] * b[6] + a[14] * b[7];
        newMat[7] = a[3] * b[4] + a[7] * b[5] + a[11] * b[6] + a[15] * b[7];
        newMat[8] = a[0] * b[8] + a[4] * b[9] + a[8] * b[10] + a[12] * b[11];
        newMat[9] = a[1] * b[8] + a[5] * b[9] + a[9] * b[10] + a[13] * b[11];
        newMat[10] = a[2] * b[8] + a[6] * b[9] + a[10] * b[10] + a[14] * b[11];
        newMat[11] = a[3] * b[8] + a[7] * b[9] + a[11] * b[10] + a[15] * b[11];
        newMat[12] = a[0] * b[12] + a[4] * b[13] + a[8] * b[14] + a[12] * b[15];
        newMat[13] = a[1] * b[12] + a[5] * b[13] + a[9] * b[14] + a[13] * b[15];
        newMat[14] = a[2] * b[12] + a[6] * b[13] + a[10] * b[14] + a[14] * b[15];
        newMat[15] = a[3] * b[12] + a[7] * b[13] + a[11] * b[14] + a[15] * b[15];

        return newMat;
    }

    mat4 quaternionToMat4(float x, float y, float z, float w)
    {
        mat4 newMat;

        newMat[0] = 1.0f - 2.0f * y * y - 2.0f * z * z;
        newMat[1] = 2.0f * x * y + 2.0f * z * w;
        newMat[2] = 2.0f * x * z - 2.0f * y * w;
        newMat[3] = 0.0f;
        newMat[4] = 2.0f * x * y - 2.0f * z * w;
        newMat[5] = 1.0f - 2.0f * x * x - 2.0f * z * z;
        newMat[6] = 2.0f * y * z + 2.0f * x * w;
        newMat[7] = 0.0f;
        newMat[8] = 2.0f * x * z + 2.0f * y * w;
        newMat[9] = 2.0f * y * z - 2.0f * x * w;
        newMat[10] = 1.0f - 2.0f * x * x - 2.0f * y * y;
        newMat[11] = 0.0f;
        newMat[12] = 0.0f;
        newMat[13] = 0.0f;
        newMat[14] = 0.0f;
        newMat[15] = 1.0f;

        return newMat;
    }

    mat4 mat4Transform(const mat4& target, const vec3& translation, const vec4& rotation, const vec3& scale)
    {
        mat4 newMat = target;

        newMat[12] += translation[0];
        newMat[13] += translation[1];
        newMat[14] += translation[2];

        float x = rotation[0];
        float y = rotation[1];
        float z = rotation[2];
        float w = rotation[3];

        mat4 rotationMatrix = quaternionToMat4(x, y, z, w);
        newMat = multiplyMM(newMat, rotationMatrix);

        mat4 scaleMatrix = { scale[0], 0.0f, 0.0f, 0.0f, 0.0f, scale[1], 0.0f, 0.0f, 0.0f, 0.0f, scale[2], 0.0f, 0.0f, 0.0f, 0.0f, 1.0f };
        newMat = multiplyMM(newMat, scaleMatrix);

        return newMat;
    }
}

using namespace Mat4;

constexpr mat4 identityMat4 = { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f };

// single mesh struct
struct aMesh
{
public:

    void extractInverseBindMatrices(const tinygltf::Node& node, const tinygltf::Model& model);
    void extractJointTransformation(const mat4& target, int jointNodeIndex, const tinygltf::Model& model);
    void extractMeshPrimitiveData(const tinygltf::Node& node, const tinygltf::Model& model);
    void copyToJava(JNIEnv* env, jobject obj);

private:

    std::vector<int> _indices;
    std::vector<float> _vertices;
    std::vector<float> _normals;
    std::vector<float> _texcoords;
    std::vector<float> _joints;
    std::vector<float> _weights;

    std::vector<float> _jointTransformations;
    std::vector<float> _inverseBindMatrices;
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
    const tinygltf::Skin& skin = model.skins[node.skin];
    const tinygltf::Accessor& accessor = model.accessors[skin.inverseBindMatrices];
    const tinygltf::BufferView &bufferView = model.bufferViews[accessor.bufferView];
    const tinygltf::Buffer &buffer = model.buffers[bufferView.buffer];

    float *data = (float *) (buffer.data.data() + bufferView.byteOffset);
    std::vector<float> inverseBindMatrices{data,data + bufferView.byteLength / sizeof(float)};

    _inverseBindMatrices.insert(std::end(_inverseBindMatrices), std::begin(inverseBindMatrices),
                                   std::end(inverseBindMatrices));
}

void aMesh::extractJointTransformation(const mat4& target, int jointNodeIndex, const tinygltf::Model& model)
{
    const tinygltf::Node &jointNode = model.nodes[jointNodeIndex];

    vec3 translation { 0.0f, 0.0f, 0.0f };
    vec4 rotation { 0.0f, 0.0f, 0.0f, 1.0f };
    vec3 scale { 1.0f, 1.0f, 1.0f };

    if (!jointNode.translation.empty())
    {
        translation[0] = jointNode.translation[0];
        translation[1] = jointNode.translation[1];
        translation[2] = jointNode.translation[2];
    }

    if (!jointNode.rotation.empty())
    {
        rotation[0] = jointNode.rotation[0];
        rotation[1] = jointNode.rotation[1];
        rotation[2] = jointNode.rotation[2];
        rotation[3] = jointNode.rotation[3];
    }

    if (!jointNode.scale.empty())
    {
        scale[0] = jointNode.scale[0];
        scale[1] = jointNode.scale[1];
        scale[2] = jointNode.scale[2];
    }

    mat4 transformedTarget = mat4Transform(target, translation, rotation, scale);

    _jointTransformations.insert(std::end(_jointTransformations), std::begin(transformedTarget),
                               std::end(transformedTarget));

    for (const auto& childIndex : jointNode.children)
    {
        extractJointTransformation(transformedTarget, childIndex, model);
    }
}

void aMesh::extractMeshPrimitiveData(const tinygltf::Node& node, const tinygltf::Model& model)
{
    const tinygltf::Mesh &mesh = model.meshes[node.mesh];

    for (size_t primitiveIndex = 0; primitiveIndex < mesh.primitives.size(); primitiveIndex++)
    {
        const tinygltf::Primitive &primitive = mesh.primitives[primitiveIndex];
        const tinygltf::Accessor &indexAccessor = model.accessors[primitive.indices];
        const tinygltf::BufferView &indexBufferView = model.bufferViews[indexAccessor.bufferView];
        const tinygltf::Buffer &indexBuffer = model.buffers[indexBufferView.buffer];

        unsigned short *data = (unsigned short *) (indexBuffer.data.data() +
                                                       indexBufferView.byteOffset);
        std::vector<unsigned short> indicesData{data, data + indexBufferView.byteLength / sizeof(unsigned short)};

        _indices.insert(std::end(_indices), std::begin(indicesData), std::end(indicesData));

        for (const auto &attrib : primitive.attributes)
        {
            const tinygltf::Accessor &accessor = model.accessors[attrib.second];
            const tinygltf::BufferView &bufferView = model.bufferViews[accessor.bufferView];
            const tinygltf::Buffer &buffer = model.buffers[bufferView.buffer];

            float *data = (float *) (buffer.data.data() + bufferView.byteOffset);
            std::vector<float> attribData{data,data + bufferView.byteLength / sizeof(float)};

            if (attrib.first.compare("POSITION") == 0)
            {
                _vertices.insert(std::end(_vertices), std::begin(attribData), std::end(attribData));
            }

            else if (attrib.first.compare("NORMAL") == 0)
            {
                _normals.insert(std::end(_normals), std::begin(attribData), std::end(attribData));
            }

            else if (attrib.first.compare("TEXCOORD_0") == 0)
            {
                _texcoords.insert(std::end(_texcoords), std::begin(attribData), std::end(attribData));
            }

            else if (attrib.first.compare("JOINTS_0") == 0)
            {
                unsigned char *data = (unsigned char *) (buffer.data.data() + bufferView.byteOffset);
                std::vector<unsigned char> jointsData{data, data + bufferView.byteLength / sizeof(unsigned char)};

                _joints.insert(std::end(_joints), std::begin(jointsData), std::end(jointsData));
            }

            else if (attrib.first.compare("WEIGHTS_0") == 0)
            {
                _weights.insert(std::end(_weights), std::begin(attribData), std::end(attribData));
            }
        }
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
    // java string to c++ const char*
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
    bool ret = loader.LoadBinaryFromFile(&model, &err, &warn, file, ".glb");

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

    const tinygltf::Scene& scene = model.scenes[model.defaultScene];
    const tinygltf::Node& rootNode = model.nodes[scene.nodes[0]];

    // mesh node must be the child of root node
    for (const auto& childrenIndex : rootNode.children)
    {
        const tinygltf::Node& children = model.nodes[childrenIndex];
        if (children.mesh >= 0)
        {
            mesh.extractMeshPrimitiveData(children, model);

            // mesh node could also be a skin node
            if (children.skin >= 0)
            {
                mesh.extractInverseBindMatrices(children, model);

                const tinygltf::Skin& skin = model.skins[children.skin];

                mat4 globalTransformationMat4 = identityMat4;
                vec3 translation { 0.0f, 0.0f, 0.0f };
                vec4 rotation { 0.0f, 0.0f, 0.0f, 1.0f };
                vec3 scale { 1.0f, 1.0f, 1.0f };

                if (!rootNode.translation.empty())
                {
                    translation[0] = rootNode.translation[0];
                    translation[1] = rootNode.translation[1];
                    translation[2] = rootNode.translation[2];
                }

                if (!rootNode.rotation.empty())
                {
                    rotation[0] = rootNode.rotation[0];
                    rotation[1] = rootNode.rotation[1];
                    rotation[2] = rootNode.rotation[2];
                    rotation[3] = rootNode.rotation[3];
                }

                if (!rootNode.scale.empty())
                {
                    scale[0] = rootNode.scale[0];
                    scale[1] = rootNode.scale[1];
                    scale[2] = rootNode.scale[2];
                }

                globalTransformationMat4 = mat4Transform(globalTransformationMat4, translation, rotation, scale);

                mesh.extractJointTransformation(globalTransformationMat4, skin.joints[0], model);
            }

            break;
        }
    }

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

    jfieldID jointTransformationsField = env->GetFieldID(loaderClass, "jointTransformations", "[F");
    jfieldID inverseBindMatricesField = env->GetFieldID(loaderClass, "inverseBindMatrices", "[F");

    copyToJavaClass<int>(env, obj, _indices, indicesField);
    copyToJavaClass<float>(env, obj, _vertices, verticesField);
    copyToJavaClass<float>(env, obj, _normals, normalsField);
    copyToJavaClass<float>(env, obj, _texcoords, texcoordsField);
    copyToJavaClass<float>(env, obj, _joints, jointsField);
    copyToJavaClass<float>(env, obj, _weights, weightsField);

    copyToJavaClass<float>(env, obj, _jointTransformations, jointTransformationsField);
    copyToJavaClass<float>(env, obj, _inverseBindMatrices, inverseBindMatricesField);
}
