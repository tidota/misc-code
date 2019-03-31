#include <cstring>
#include <memory>

#include "User.h"
#include "loader.h"

#include <iostream>

using namespace database;

JNIEXPORT jint JNICALL Java_User_func(JNIEnv * env, jobject caller, jint val)
{
  return val * 10;
}

JNIEnv* env_buff = nullptr;
jobject jthis_buff;

std::shared_ptr<jobject> createNode(
  std::shared_ptr<jobject> parent, std::shared_ptr<Params> param)
{
  jclass thisClass = env_buff->GetObjectClass(jthis_buff);

  // Get the callback function
  // the third parameter is a descripter.
  // you can check by `javap -p -s <class name>`
  jmethodID createNode
    = env_buff->GetStaticMethodID(
      thisClass, "createNode", "(LUser;Ljava/lang/String;)LUser");
  if (NULL == createNode)
    return nullptr;

  jstring jstr;
  char buff[100];
  strcpy(buff, param->name.c_str());
  env_buff->ReleaseStringUTFChars(jstr, buff);
  return std::make_shared<jobject>(
    env_buff->CallObjectMethod(jthis_buff, createNode, *parent, jstr));
}

JNIEXPORT jobject JNICALL Java_User_load(JNIEnv * env, jobject jthis)
{
    jclass thisClass = env->GetObjectClass(jthis);

  // Get the callback function
  // the third parameter is a descripter.
  // you can check by `javap -p -s <class name>`
  jmethodID createNode
    = env->GetStaticMethodID(
      thisClass, "createNode", "(LUser;Ljava/lang/String;)LUser;");
  if (NULL == createNode)
    return nullptr;

  jstring jstr = env->NewStringUTF("root");
  //char buff[100] = "root";
  //env->ReleaseStringUTFChars(jstr, buff);
  //env->ReleaseStringUTFChars(jstr, "");
  jobject rootPtr = env->CallObjectMethod(jthis, createNode, NULL, jstr);

  std::cout << "end of load" << std::endl;
  std::cout << rootPtr << std::endl;
  return rootPtr;
}