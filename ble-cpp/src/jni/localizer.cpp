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

//#define ANDROID_LOG

#ifdef ANDROID_LOG
#include <android/log.h>
#endif // ANDROID_LOG

#include "BasicLocalizer.hpp"

#include <set>
#include <algorithm>
#include <cctype>

extern "C" {

struct UserData {
    JavaVM *jvm;
    jobject obj;
    jmethodID callbackId;
    jmethodID logStringId;
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

    // orientation accuracy
    std::vector<loc::State> states = *pStatus->states();

    auto stdLoc = loc::Location::standardDeviation(states);

    double orientationAccuracy = 999;
    if(localizer->tracksOrientation()){
        auto normParams = loc::Pose::computeWrappedNormalParameter(states);
        double stdOri = normParams.stdev(); // radian
        orientationAccuracy = stdOri/M_PI*180.0; // degree
    }

    loc::Status::LocationStatus  locStatus = pStatus->locationStatus();

    auto global = projection->localToGlobal(*(meanLoc.get()));
    jdouble x = meanLoc->x(),
            y = meanLoc->y(),
            z = meanLoc->z(),
            lat = global.lat(),
            lng = global.lng(),
            floor = meanLoc->floor(),
            orientation = meanPose->orientation(),
            velocity = meanPose->velocity();
    jdouble stdX = stdLoc.x();
    jdouble stdY = stdLoc.y();
    jdouble stdOrientation = orientationAccuracy;
    jint locationStatus = locStatus;

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
        env->CallVoidMethod(ud->obj, ud->callbackId, x, y, z, floor, lat, lng, orientation, velocity,
                            stdX, stdY, stdOrientation, locationStatus,
                            jarray, jarray2);
//        ud->jvm->DetachCurrentThread(); // TODO DetachCurrentThread cause crash
    }
}


void functionCalledToLog(void* userData, std::string str){
#ifdef ANDROID_LOG
    __android_log_print(ANDROID_LOG_DEBUG,"NativeLog","%s", str.c_str());
#endif // ANDROID_L0G

    UserData *ud = (UserData *) userData;
    JNIEnv *env;
#ifdef ANDROID_JNI
    if (ud->jvm->AttachCurrentThread(&env, NULL) == JNI_OK) {
#else
 　 if (ud->jvm->AttachCurrentThread((void**)&env, NULL) == JNI_OK) {
#endif // ANDROID_JNI
        jstring jstr = env->NewStringUTF(str.c_str());
        env->CallVoidMethod(ud->obj, ud->logStringId, jstr);
        env->DeleteLocalRef(jstr);
    }
}


const char* updatedMethodString = "updated";
//const char* updatedMethodSigniture = "(DDDDDDDD[D[D)V";
const char* updatedMethodSigniture = "(DDDDDDDDDDDI[D[D)V";

const char* logStringMethodString = "logString";
const char* logStringMethodSigniture = "(Ljava/lang/String;)V";

JNIEXPORT jlong JNICALL Java_hulop_jni_Localizer_create__III(JNIEnv *env,
                                                        jobject obj, jint mode, jint smooth,
                                                        jint accuracy) {
    try {
        jclass cls = env->GetObjectClass(obj);
        UserData *pData = new UserData();
        env->GetJavaVM(&pData->jvm);
        pData->obj = env->NewGlobalRef(obj);
        pData->callbackId = env->GetMethodID(cls, updatedMethodString, updatedMethodSigniture);
        pData->logStringId = env->GetMethodID(cls, logStringMethodString, logStringMethodSigniture);
        pData->showDebugInfo = false;
        pData->lastShowDebugInfo = 0;
        pData->localizer = std::shared_ptr<loc::BasicLocalizer>(new loc::BasicLocalizer());
        pData->localizer->updateHandler(functionCalledWhenUpdated, pData);
        pData->localizer->logHandler(functionCalledToLog, pData);
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

/*
 * Class:     hulop_jni_Localizer
 * Method:    create
 * Signature: (IIILjava/lang/String;)J
 */
 JNIEXPORT jlong JNICALL Java_hulop_jni_Localizer_create__IIILjava_lang_String_2(JNIEnv *env,
										 jobject obj, jint mode, jint smooth,jint accuracy,
										 jstring optionsJSON) {
   try {
        jclass cls = env->GetObjectClass(obj);
        UserData *pData = new UserData();
        env->GetJavaVM(&pData->jvm);
        pData->obj = env->NewGlobalRef(obj);
        pData->callbackId = env->GetMethodID(cls, updatedMethodString, updatedMethodSigniture);
        pData->logStringId = env->GetMethodID(cls, logStringMethodString, logStringMethodSigniture);
        pData->showDebugInfo = false;
        pData->lastShowDebugInfo = 0;

	const char *nativeString = env->GetStringUTFChars(optionsJSON, 0);

	std::stringbuf strBuf( nativeString);	
	std::istream istream( &strBuf );
	loc::BasicLocalizerParameters localizerParams;
	cereal::JSONInputArchive iarchive(istream);
	iarchive(localizerParams);
	
	env->ReleaseStringUTFChars(optionsJSON, nativeString);
	
        pData->localizer = std::shared_ptr<loc::BasicLocalizer>(new loc::BasicLocalizer(localizerParams));
        pData->localizer->updateHandler(functionCalledWhenUpdated, pData);
        pData->localizer->logHandler(functionCalledToLog, pData);
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


JNIEXPORT jlong JNICALL Java_hulop_jni_Localizer_create__Ljava_lang_String_2(JNIEnv *env, jobject obj,
                                                                             jstring optionsJSON) {
    try {
        jclass cls = env->GetObjectClass(obj);
        UserData *pData = new UserData();
        env->GetJavaVM(&pData->jvm);
        pData->obj = env->NewGlobalRef(obj);
        pData->callbackId = env->GetMethodID(cls, updatedMethodString, updatedMethodSigniture);
        pData->logStringId = env->GetMethodID(cls, logStringMethodString, logStringMethodSigniture);
        pData->showDebugInfo = false;
        pData->lastShowDebugInfo = 0;

        const char *nativeString = env->GetStringUTFChars(optionsJSON, 0);

        std::stringbuf strBuf( nativeString);
        std::istream istream( &strBuf );
        loc::BasicLocalizerParameters localizerParams;
        cereal::JSONInputArchive iarchive(istream);
        iarchive(localizerParams);

        env->ReleaseStringUTFChars(optionsJSON, nativeString);

        pData->localizer = std::shared_ptr<loc::BasicLocalizer>(new loc::BasicLocalizer(localizerParams));
        pData->localizer->updateHandler(functionCalledWhenUpdated, pData);
        pData->localizer->logHandler(functionCalledToLog, pData);
        return jlong(pData);
    } catch (std::exception &e) {
        throwRuntimeException(env, &e);
    } catch (...) {
        throwRuntimeException(env, NULL);
    }
    return 0;
}

JNIEXPORT void JNICALL Java_hulop_jni_Localizer_reset
   (JNIEnv *env, jobject obj, jlong nativePtr) {
  std::shared_ptr<loc::BasicLocalizer> localizer = ((UserData *) nativePtr)->localizer;
  localizer->resetStatus();
}

JNIEXPORT void JNICALL Java_hulop_jni_Localizer_overwrite_1location_1unknown
        (JNIEnv *env, jobject obj, jlong nativePtr) {
    std::shared_ptr<loc::BasicLocalizer> localizer = ((UserData *) nativePtr)->localizer;
    localizer->overwriteLocationStatus(loc::Status::UNKNOWN);
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

JNIEXPORT jdouble JNICALL Java_hulop_jni_Localizer_set_1model__JLjava_lang_String_2DD(JNIEnv *env,
                                                              jobject obj, jlong nativePtr,
                                                              jstring jdir, jdouble biasValue,
                                                              jdouble biasWidth) {
    double prevBias = 0;
    try {
        const char *utf = env->GetStringUTFChars(jdir, NULL);
        std::string dir(utf);
        std::shared_ptr<loc::BasicLocalizer> localizer = ((UserData *) nativePtr)->localizer;
        prevBias = localizer->stateProperty->meanRssiBias();
        if (biasValue != 0) {
            localizer->minRssiBias(biasValue - biasWidth);
            localizer->meanRssiBias(biasValue);
            localizer->maxRssiBias(biasValue + biasWidth);
        }
        localizer->diffusionOrientationBias = (10.0 / 180.0 * M_PI);
        localizer->angularVelocityLimit = (45.0 / 180.0 * M_PI);
        localizer->setModel(dir + "/model.json", dir);
        env->ReleaseStringUTFChars(jdir, utf);
    } catch (std::exception &e) {
        throwRuntimeException(env, &e);
    } catch (...) {
        throwRuntimeException(env, NULL);
    }
    return prevBias;
}

 /*
 * Class:     hulop_jni_Localizer
 * Method:    set_model
 * Signature: (JLjava/lang/String;Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL Java_hulop_jni_Localizer_set_1model__JLjava_lang_String_2Ljava_lang_String_2
  (JNIEnv *env, jobject obj, jlong nativePtr, jstring jmodel, jstring jwdir){  
    try {
        const char *utf1 = env->GetStringUTFChars(jmodel, NULL);
        std::string model(utf1);
        const char *utf2 = env->GetStringUTFChars(jwdir, NULL);
        std::string wdir(utf2);
	
        std::shared_ptr<loc::BasicLocalizer> localizer = ((UserData *) nativePtr)->localizer;
        localizer->setModel(model, wdir);
	
        env->ReleaseStringUTFChars(jmodel, utf1);
	env->ReleaseStringUTFChars(jwdir, utf2);
    } catch (std::exception &e) {
        throwRuntimeException(env, &e);
    } catch (...) {
        throwRuntimeException(env, NULL);
    }
}

/*
 * Class:     hulop_jni_Localizer
 * Method:    set_bias
 * Signature: (DD)V
 */
 JNIEXPORT void JNICALL Java_hulop_jni_Localizer_set_1bias
   (JNIEnv *env, jobject obj, jlong nativePtr, jdouble biasValue, jdouble biasWidth){
   
   std::shared_ptr<loc::BasicLocalizer> localizer = ((UserData *) nativePtr)->localizer;
   if (biasValue != 0) {
     localizer->minRssiBias(biasValue - biasWidth);
     localizer->meanRssiBias(biasValue);
     localizer->maxRssiBias(biasValue + biasWidth);
   }
 }


JNIEXPORT void JNICALL Java_hulop_jni_Localizer_put_1beacons(JNIEnv *env,
                                                             jobject obj, jlong nativePtr,
                                                             jlong timestamp,
                                                             jobjectArray juuids,
                                                             jobjectArray jbeacons) {
    try {
        UserData* ud = (UserData*) nativePtr;

        loc::Beacons beacons;
        beacons.timestamp(timestamp);
        int len = env->GetArrayLength(jbeacons);
        for (int i = 0; i < len; i++) {
            // get uuid of an observed beacon
            jstring juuid = (jstring) env->GetObjectArrayElement(juuids, i);
            const char *cuuid = env->GetStringUTFChars(juuid, NULL);
            std::string uuid(cuuid);
            env->ReleaseStringUTFChars(juuid, cuuid);
            // get beacon data
            jdouble *element = env->GetDoubleArrayElements(
                    (jdoubleArray) env->GetObjectArrayElement(jbeacons, i), JNI_FALSE);

            loc::Beacon beacon(uuid, (int) element[0], (int) element[1], element[2]);
            beacons.push_back(beacon);
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
  
  /*
 * Class:     hulop_jni_Localizer
 * Method:    put_heading
 * Signature: (JJDDDDDD)V
 */
JNIEXPORT void JNICALL Java_hulop_jni_Localizer_put_1heading
  (JNIEnv *env, jobject obj, jlong nativePtr, jlong timestamp, jdouble magnetic_heading, jdouble true_heading, jdouble heading_accuracy, jdouble mx, jdouble my , jdouble mz)
{
  try {
    ((UserData *) nativePtr)->localizer->putHeading(loc::Heading(timestamp, magnetic_heading, true_heading, heading_accuracy, mx, my, mz));
  } catch (std::exception &e) {
    throwRuntimeException(env, &e);
  } catch (...) {
    throwRuntimeException(env, NULL);
  }
}
  
  
/*
 * Class:     hulop_jni_Localizer
 * Method:    put_local_heading
 * Signature: (JJDD)V
 */
JNIEXPORT void JNICALL Java_hulop_jni_Localizer_put_1local_1heading
(JNIEnv *env, jobject obj, jlong nativePtr, jlong timestamp, jdouble orientation, jdouble orientation_deviation){
  try {
    ((UserData *) nativePtr)->localizer->putLocalHeading(loc::LocalHeading(timestamp, orientation, orientation_deviation));
  } catch (std::exception &e) {
    throwRuntimeException(env, &e);
  } catch (...) {
    throwRuntimeException(env, NULL);
  }
}

/*
 * Class:     hulop_jni_Localizer
 * Method:    put_altimeter
 * Signature: (JJDD)V
 */
JNIEXPORT void JNICALL Java_hulop_jni_Localizer_put_1altimeter
(JNIEnv *env, jobject obj, jlong nativePtr, jlong timestamp, jdouble relative_altitude, jdouble pressure){
  try {
    ((UserData *) nativePtr)->localizer->putAltimeter(loc::Altimeter(timestamp, relative_altitude, pressure));
  } catch (std::exception &e) {
    throwRuntimeException(env, &e);
  } catch (...) {
    throwRuntimeException(env, NULL);
  }
}

}