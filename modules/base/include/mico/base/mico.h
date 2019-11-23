//---------------------------------------------------------------------------------------------------------------------
//  mico
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------

#ifndef MICO_BASE_RGBDTOOLS_H_
#define MICO_BASE_RGBDTOOLS_H_

#include <mico/base/StereoCamera.h>

#include <mico/base/utils/Graph2d.h>
#include <mico/base/utils/Gui.h>
#include <mico/base/utils/LogManager.h>

#include <mico/base/cjson/json.h>

#include <mico/base/map3d/RansacP2P.h>
#include <mico/base/map3d/utils3d.h>
#include <mico/base/map3d/GMMEM.h>
#include <mico/base/map3d/BundleAdjuster.h>
#include <mico/base/map3d/utils3d.h>
#ifdef USE_MONGO
#   include <mico/base/map3d/MapDatabase.h>
#endif
#include <mico/base/object_detection/feature_based/FeatureModel.h>
#include <mico/base/object_detection/feature_based/FeatureObjectTracker.h>

#endif
 