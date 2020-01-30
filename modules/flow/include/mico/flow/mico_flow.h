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

// Streamers
#undef Q_FOREACH
#include <mico/flow/blocks/streamers/StreamRealSense.h>
#include <mico/flow/blocks/streamers/StreamKinect.h>
#include <mico/flow/blocks/streamers/StreamDataset.h>
#include <mico/flow/blocks/streamers/StreamPixhawk.h>
#include <mico/flow/blocks/streamers/StreamWebcam.h>


// Processors
#include <mico/flow/blocks/processors/BlockOdometryRGBD.h>
#include <mico/flow/blocks/processors/BlockOdometryPhotogrammetry.h>
#include <mico/flow/blocks/processors/BlockDatabaseMarkI.h>
#include <mico/flow/blocks/processors/BlockLoopClosure.h>
#include <mico/flow/blocks/processors/BlockOptimizerCF.h>
#include <mico/flow/blocks/processors/BlockEKFIMU.h>
// #include <mico/flow/blocks/processors/BlockParticleFilterKinematic.h>


// Visualizers
#include <mico/flow/blocks/visualizers/BlockImageVisualizer.h>
#include <mico/flow/blocks/visualizers/BlockTrayectoryVisualizer.h>
#include <mico/flow/blocks/visualizers/BlockDatabaseVisualizer.h>
#include <mico/flow/blocks/visualizers/BlockSceneVisualizer.h>
#include <mico/flow/blocks/visualizers/BlockPointCloudVisualizer.h>
#include <mico/flow/blocks/visualizers/BlockTrajectoryVisualizerPangolin.h>
#include <mico/flow/blocks/visualizers/BlockSceneVisualizerPangolin.h>
#include <mico/flow/blocks/visualizers/BlockSlamDebugger.h>
#include <mico/flow/blocks/visualizers/BlockDataframeInspector.h>

// Casters
#include <mico/flow/blocks/CastBlocks.h>

// Queuers
#include <mico/flow/blocks/BlockQueuer.h>

// Savers
#include <mico/flow/blocks/savers/SaverImage.h>
#include <mico/flow/blocks/savers/SaverTrajectory.h>
#include <mico/flow/blocks/savers/SaverEntity.h>

// DNN
#ifdef HAS_DARKNET
    #include <mico/flow/blocks/processors/BlockDarknet.h> // 666 HAS DARKNET
#endif

// Misc
#include <mico/flow/blocks/misc/BlockPython.h>
#include <mico/flow/blocks/misc/BlocksUtils3D.h>
#include <mico/flow/blocks/misc/BlocksUtils2D.h>