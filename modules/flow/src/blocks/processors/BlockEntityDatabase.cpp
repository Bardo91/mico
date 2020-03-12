//---------------------------------------------------------------------------------------------------------------------
//  micoso
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2019 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com & Ricardo Lopez Lopez (a.k.a Ric92)
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

#include <mico/flow/blocks/processors/BlockEntityDatabase.h>
#include <flow/Policy.h>
#include <flow/Outpipe.h>
#include <flow/DataFlow.h>

#include <sstream>

namespace mico{

    BlockEntityDatabase::BlockEntityDatabase(){ 
        createPipe("Entities", "v_entity");
        
        createPolicy({{"Entities", "v_entity"}});

        registerCallback({"Entities"}, 
                                [&](flow::DataFlow _data){
                                    if(idle_){
                                        idle_ = false;
                                        #ifdef HAS_DARKNET
                                        auto entities = _data.get<std::vector<std::shared_ptr<mico::Entity<pcl::PointXYZRGBNormal>>>>("Entities"); 
                                        // store the new entities
                                        std::vector<std::shared_ptr<mico::Entity<pcl::PointXYZRGBNormal>>> newEntities;

                                        if(!entities_.empty()){
                                            // candidates
                                            for(auto queryE: entities){
                                                // check overlap
                                                auto eDfs = queryE->dfs();
                                                auto queryBoundingCube = queryE->boundingCube(eDfs[0]);

                                                bool newEntity = true;
                                                std::shared_ptr<mico::Entity<pcl::PointXYZRGBNormal>> parentEntity = nullptr;
                                                float affinity = 0;
                                                auto t1 = std::chrono::high_resolution_clock::now();
                                                // entities in database
                                                //for(auto trainE: entities_){
                                                int i = 0;
                                                for(auto trainE = entities_.rbegin(); trainE != entities_.rend() && i < comparedEntities_; ++trainE, i++){
                                                    if(queryE->id() != (*trainE).second->id()){
                                                        float overlaped = queryE->percentageOverlapped((*trainE).second);
                                                        // std::cout << "Overlapped percentage between " << queryE->id() << " and " << trainE.second->id() << " : " << 
                                                        //     overlaped << std::endl;

                                                        // if the entity overlaps with other created dont create a new one and update the first
                                                        if(overlaped > overlapScore_ && overlaped > affinity){
                                                            parentEntity = (*trainE).second;
                                                            affinity = overlaped;
                                                            newEntity = false;
                                                        }
                                                    }   
                                                }

                                                auto t2 = std::chrono::high_resolution_clock::now();
                                                std::chrono::duration<float,std::milli> overlapTime = (t2 - t1);
                                                std::cout << "[BlockEntityDatabase]Time checking overlaping between cubes: " << overlapTime.count()/1000 << std::endl;
                                                // create new entity associated to the most related parent
                                                if(newEntity){
                                                    entities_[queryE->id()] = queryE;
                                                    newEntities.push_back(queryE);
                                                    std::cout << "Created new entity " << queryE->id() << std::endl;
                                                }
                                                else{
                                                    // update entity 
                                                    parentEntity->update(queryE);
                                                    //std::cout << "--------------Created new entity associated with " << parentEntity << " and overlaped% " << affinity << std::endl;
                                                }
                                            }
                                        }else{
                                            for(auto e: entities){
                                                entities_[e->id()] = e;
                                                newEntities.push_back(e);
                                                // check overlapping here maybe
                                            }
                                        }
                                        std::cout << "Number of entities: " << entities_.size() << std::endl;
                                        getPipe("Entities")->flush(newEntities);
                                        idle_ = true;
                                        #endif
                                    }
                                }
        );
    }

    // BlockEntityDatabase::~BlockEntityDatabase(){
    // } 

    bool BlockEntityDatabase::configure(std::unordered_map<std::string, std::string> _params){
        cjson::Json jParams;
        for(auto &param: _params){
            if(param.second == "")
                return false;
            if(param.first == "overlapScore"){
                std::istringstream istr(_params["overlapScore"]);
                istr >> overlapScore_;
                jParams["overlapScore"] = overlapScore_;
            }
            if(param.first == "compared_entities"){
                std::istringstream istr(_params["compared_entities"]);
                istr >> comparedEntities_;
                jParams["compared_entities"] = comparedEntities_;
            }
            
        }
        std::cout << "[BlockEntityDatabase]Score selected: " << overlapScore_ << std::endl;
        std::cout << "[BlockEntityDatabase]Entities compared: " << comparedEntities_ << std::endl;
    }
    
    std::vector<std::string> BlockEntityDatabase::parameters(){
        return {"overlapScore","compared_entities"};
    }
}
