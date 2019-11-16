
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

#include <map>

namespace mico{
    //---------------------------------------------------------------------------------------------------------------------
    template <DebugLevels DebugLevel_, OutInterfaces OutInterface_>
    template<typename PointType_>
    inline std::vector<std::shared_ptr<Dataframe<PointType_>>> 
        LoopClosureDetector<DebugLevel_, OutInterface_>::findPath(
            std::shared_ptr<Dataframe<PointType_>> _init, 
            std::shared_ptr<Dataframe<PointType_>> _end
            ){

        std::map<int, bool> visitedMap;
        std::vector<typename std::vector<std::shared_ptr<Dataframe<PointType_>>>> tree;

        // Set init node
        tree.push_back({_init});

        std::vector<std::shared_ptr<Dataframe<PointType_>>> loopPath;
        bool reachedEnd = false;
        while(!reachedEnd){ // Iterate until reached end node
            std::vector<typename std::vector<std::shared_ptr<Dataframe<PointType_>>>> newTree;
            for(auto iter = tree.begin(); iter != tree.end() && !reachedEnd ; ++iter){    // Iterate over tree
                // Apice of branch
                auto apice = iter->back();
                for(auto &other: apice->covisibility()){ // Iterate over covisibility;
                    if(other->id() == _end->id()){
                        reachedEnd = true;  // Mark as finished
                        loopPath = *iter;   // Copy current branch
                        loopPath.push_back(other); // Add last as apice
                        break;
                    }else{
                        if(!visitedMap[other->id()]){
                            // Mark as visited
                            visitedMap[other->id()] = true;
                            
                            // create a new branch in the tree using the base branch + new apice.
                            auto newBranch = *iter;
                            newBranch.push_back(other);
                            newTree.push_back(newBranch);
                        }
                    }
                    
                }
                if(reachedEnd) break;
            }
            tree = newTree;
        }

        return loopPath;
    }

}



