#include "User.h"
#include "database.h"

#include <memory>

using namespace database;

class database::NodeImpl
{
public:
  jobject data;
  NodeImpl(jobject jobj): data(jobj){}
};

JNIEnv * env_buff = nullptr;
jobject jthis_buff;
jmethodID callback;

std::shared_ptr<NodeImpl> createNode(
  std::shared_ptr<NodeImpl> parent, std::shared_ptr<Params> param)
{
  return std::make_shared<NodeImpl>(
    env_buff->CallObjectMethod(
      jthis_buff, callback, (parent != nullptr)? parent->data: nullptr,
      env_buff->NewStringUTF(param->name.c_str())));
}

JNIEXPORT jobject JNICALL Java_User_load(JNIEnv * env, jobject jthis)
{
  env_buff = env;
  jthis_buff = jthis;

  jclass thisClass = env->GetObjectClass(jthis);
  callback = env->GetStaticMethodID(
      thisClass, "createNode", "(LNode;Ljava/lang/String;)LNode;");
  if (callback == NULL)
    return nullptr;

  return load(createNode)->data;
}

