#include "User.h"

JNIEXPORT jint JNICALL Java_User_func(JNIEnv * env, jobject caller, jint val)
{
  return val * 10;
}
