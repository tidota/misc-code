#include "User.h"
#include "loader.h"

#include <iostream>
#include <memory>

using namespace database;


JNIEnv * env_buff = nullptr;
jobject jthis_buff;
jmethodID method;

std::shared_ptr<jobject> createNode(
  std::shared_ptr<jobject> parent, std::shared_ptr<Params> param)
{
  return std::make_shared<jobject>(
    env_buff->CallObjectMethod(
      jthis_buff, method, (parent != nullptr)? *parent: nullptr,
      env_buff->NewStringUTF(param->name.c_str())));
}

JNIEXPORT jobject JNICALL Java_User_load(JNIEnv * env, jobject jthis)
{
  env_buff = env;
  jthis_buff = jthis;

  jclass thisClass = env->GetObjectClass(jthis);
  method = env->GetStaticMethodID(
      thisClass, "createNode", "(LNode;Ljava/lang/String;)LNode;");
  if (NULL == method)
    return nullptr;

  return *(load(createNode));
}

