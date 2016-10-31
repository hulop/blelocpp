/*******************************************************************************
 * Copyright (c) 2014, 2015  IBM Corporation and others
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *******************************************************************************/

#include <jni.h>
#include "BasicLocalizer.hpp"

extern "C" {

struct UserData {
    JavaVM *jvm;
    jobject obj;
    jmethodID callbackId;
    bool showDebugInfo;
    clock_t lastShowDebugInfo;
    std::shared_ptr<loc::BasicLocalizer> localizer;
};

inline void throwRuntimeException(JNIEnv *env, std::exception *e) {
    jclass classj = env->FindClass("java/lang/RuntimeException");
    if (classj != nullptr) {
        std::string what;
        if (e == NULL) {
            what = "Unindentified exception thrown (not derived from std::exception)";
        } else {
            what = "type ";
            what += typeid(*e).name();
            what += ": ";
            what += e->what();
        }
        env->ThrowNew(classj, what.c_str());
        env->DeleteLocalRef(classj);
    }
}

void functionCalledWhenUpdated(void *userData, loc::Status *pStatus) {
    std::shared_ptr<loc::Location> meanLoc = pStatus->meanLocation();
    std::shared_ptr<loc::Pose> meanPose = pStatus->meanPose();
    std::shared_ptr<loc::BasicLocalizer> localizer = ((UserData *) userData)->localizer;
    loc::LatLngConverter::Ptr projection = localizer->latLngConverter();
    auto global = projection->localToGlobal(*(meanLoc.get()));
    jdouble x = meanLoc->x(),
            y = meanLoc->y(),
            z = meanLoc->z(),
            lat = global.lat(),
            lng = global.lng(),
            floor = meanLoc->floor(),
            orientation = meanPose->orientation(),
            velocity = meanPose->velocity();
    UserData *ud = (UserData *) userData;
    JNIEnv *env;
#ifdef ANDROID_JNI
    if (ud->jvm->AttachCurrentThread(&env, NULL) == JNI_OK) {
#else
        if (ud->jvm->AttachCurrentThread((void**)&env, NULL) == JNI_OK) {
#endif // ANDROID_JNI
        jdoubleArray jarray = NULL;
        jdoubleArray jarray2 = NULL;
        if (ud->showDebugInfo) {
            clock_t now = clock();
            if (now > ud->lastShowDebugInfo + CLOCKS_PER_SEC) {
                ud->lastShowDebugInfo = now;
                std::shared_ptr<std::vector<loc::State>> states = pStatus->states();
                int size = states->size();
                jdouble buff[size * 2];
                jdouble buff2[size * 2];
                for (int i = 0; i < size; i++) {
                    loc::State s = states->at(i);
                    auto g = projection->localToGlobal(s);
                    buff[i * 2] = s.x();
                    buff[i * 2 + 1] = s.y();
                    buff2[i * 2] = g.lat();
                    buff2[i * 2 + 1] = g.lng();
                }
                jarray = env->NewDoubleArray(size * 2);
                env->SetDoubleArrayRegion(jarray, 0, size * 2, buff);
                jarray2 = env->NewDoubleArray(size * 2);
                env->SetDoubleArrayRegion(jarray2, 0, size * 2, buff2);
            }
        }
        env->CallVoidMethod(ud->obj, ud->callbackId, x, y, z, floor, lat, lng, orientation, velocity, jarray,
                            jarray2);
//        ud->jvm->DetachCurrentThread(); // TODO DetachCurrentThread cause crash
    }
}

JNIEXPORT jlong JNICALL Java_hulop_jni_Localizer_create(JNIEnv *env,
                                                        jobject obj, jint mode, jint smooth,
                                                        jint accuracy) {
    try {
        jclass cls = env->GetObjectClass(obj);
        UserData *pData = new UserData();
        env->GetJavaVM(&pData->jvm);
        pData->obj = env->NewGlobalRef(obj);
        pData->callbackId = env->GetMethodID(cls, "updated", "(DDDDDDDD[D[D)V");
        pData->showDebugInfo = false;
        pData->lastShowDebugInfo = 0;
        pData->localizer = std::shared_ptr<loc::BasicLocalizer>(new loc::BasicLocalizer());
        pData->localizer->updateHandler(functionCalledWhenUpdated, pData);
        pData->localizer->localizeMode = mode == 0 ? loc::ONESHOT : loc::RANDOM_WALK_ACC_ATT;
        pData->localizer->nSmooth = smooth;
        pData->localizer->nStates = accuracy * 100;
        pData->localizer->nBurnIn = accuracy * 100;
        pData->localizer->burnInRadius2D = accuracy;
        return jlong(pData);
    } catch (std::exception &e) {
        throwRuntimeException(env, &e);
    } catch (...) {
        throwRuntimeException(env, NULL);
    }
    return 0;
}

JNIEXPORT void JNICALL Java_hulop_jni_Localizer_delete(JNIEnv *env,
                                                       jobject obj, jlong nativePtr) {
    try {
        delete (UserData *) nativePtr;
    } catch (std::exception &e) {
        throwRuntimeException(env, &e);
    } catch (...) {
        throwRuntimeException(env, NULL);
    }
}

JNIEXPORT jdouble JNICALL Java_hulop_jni_Localizer_set_1model(JNIEnv *env,
                                                              jobject obj, jlong nativePtr,
                                                              jstring jdir, jdouble biasValue,
                                                              jdouble biasWidth) {
    double prevBias = 0;
    try {
        const char *utf = env->GetStringUTFChars(jdir, NULL);
        std::string dir(utf);
        std::shared_ptr<loc::BasicLocalizer> localizer = ((UserData *) nativePtr)->localizer;
        localizer->setModel(dir + "/model.json", dir);
        env->ReleaseStringUTFChars(jdir, utf);
        prevBias = localizer->stateProperty.meanRssiBias();
        if (biasValue != 0) {
            localizer->minRssiBias(biasValue - biasWidth);
            localizer->meanRssiBias(biasValue);
            localizer->maxRssiBias(biasValue + biasWidth);
        }
        localizer->diffusionOrientationBias(10.0 / 180.0 * M_PI);
        localizer->angularVelocityLimit(45.0 / 180.0 * M_PI);
    } catch (std::exception &e) {
        throwRuntimeException(env, &e);
    } catch (...) {
        throwRuntimeException(env, NULL);
    }
    return prevBias;
}

JNIEXPORT void JNICALL Java_hulop_jni_Localizer_put_1beacons(JNIEnv *env,
                                                             jobject obj, jlong nativePtr,
                                                             jobjectArray jbeacons) {
    try {
        loc::Beacons beacons;
        int len = env->GetArrayLength(jbeacons);
        for (int i = 0; i < len; i++) {
            jdouble *element = env->GetDoubleArrayElements(
                    (jdoubleArray) env->GetObjectArrayElement(jbeacons, i), JNI_FALSE);
            beacons.push_back(loc::Beacon((int) element[0], (int) element[1], element[2]));
        }
        ((UserData *) nativePtr)->localizer->putBeacons(beacons);
    } catch (std::exception &e) {
        throwRuntimeException(env, &e);
    } catch (...) {
        throwRuntimeException(env, NULL);
    }
}

JNIEXPORT jdouble JNICALL Java_hulop_jni_Localizer_estimate_1bias(JNIEnv *env,
                                                                  jobject obj, jlong nativePtr,
                                                                  jdouble x, jdouble y, jdouble z,
                                                                  jdouble floor,
                                                                  jobjectArray jbeacons) {
    try {
        loc::Beacons beacons;
        int len = env->GetArrayLength(jbeacons);
        for (int i = 0; i < len; i++) {
            jdouble *element = env->GetDoubleArrayElements(
                    (jdoubleArray) env->GetObjectArrayElement(jbeacons, i), JNI_FALSE);
            beacons.push_back(loc::Beacon((int) element[0], (int) element[1], element[2]));
        }
        std::shared_ptr<loc::BasicLocalizer> localizer = ((UserData *) nativePtr)->localizer;
        loc::Location loc(x, y, z, floor);
        localizer->resetStatus(loc, beacons);
        return localizer->estimatedRssiBias();
    } catch (std::exception &e) {
        throwRuntimeException(env, &e);
    } catch (...) {
        throwRuntimeException(env, NULL);
    }
}

JNIEXPORT void JNICALL Java_hulop_jni_Localizer_put_1acceleration(JNIEnv *env, jobject obj,
                                                                  jlong nativePtr, jlong timestamp,
                                                                  jdouble ax, jdouble ay,
                                                                  jdouble az) {
    try {
        ((UserData *) nativePtr)->localizer->putAcceleration(
                loc::Acceleration(timestamp, ax, ay, az));
    } catch (std::exception &e) {
        throwRuntimeException(env, &e);
    } catch (...) {
        throwRuntimeException(env, NULL);
    }
}

JNIEXPORT void JNICALL Java_hulop_jni_Localizer_put_1attitude(JNIEnv *env,
                                                              jobject obj, jlong nativePtr,
                                                              jlong timestamp, jdouble pitch,
                                                              jdouble roll, jdouble yaw) {
    try {
        ((UserData *) nativePtr)->localizer->putAttitude(
                loc::Attitude(timestamp, pitch, roll, yaw));
    } catch (std::exception &e) {
        throwRuntimeException(env, &e);
    } catch (...) {
        throwRuntimeException(env, NULL);
    }
}

JNIEXPORT void JNICALL Java_hulop_jni_Localizer_set_1debug(JNIEnv *env,
                                                           jobject obj, jlong nativePtr,
                                                           jboolean debug) {
    try {
        ((UserData *) nativePtr)->showDebugInfo = debug;
    } catch (std::exception &e) {
        throwRuntimeException(env, &e);
    } catch (...) {
        throwRuntimeException(env, NULL);
    }
}

}
