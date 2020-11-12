/*
 * Copyright (c) 2010-2013, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <fstream>

#include <octomap_msgs/GetOctomap.h>
#include "mapper_dyn_huk/espeleoSaveOctomap.h"

using octomap_msgs::GetOctomap;

using namespace std;
using namespace octomap;

class MapSaver{
  public:
    MapSaver(const std::string& mapname, bool full){
      ros::NodeHandle n;
      std::string servname = "octomap_binary";
      if (full)
        servname = "octomap_full";
      ROS_INFO("Requesting the map from %s...", n.resolveName(servname).c_str());
      GetOctomap::Request req;
      GetOctomap::Response resp;
      while(n.ok() && !ros::service::call(servname, req, resp))
      {
        ROS_WARN("Request to %s failed; trying again...", n.resolveName(servname).c_str());
        usleep(1000000);
      }

      if (n.ok()){ // skip when CTRL-C

        AbstractOcTree* tree = octomap_msgs::msgToMap(resp.map);
        AbstractOccupancyOcTree* octree = NULL;
        if (tree){
          octree = dynamic_cast<AbstractOccupancyOcTree*>(tree);
        } else {
          ROS_ERROR("Error creating octree from received message");
          if (resp.map.id == "ColorOcTree")
            ROS_WARN("You requested a binary map for a ColorOcTree - this is currently not supported. Please add -f to request a full map");
        }

        if (octree){
          ROS_INFO("Map received (%zu nodes, %f m res), saving to %s", octree->size(), octree->getResolution(), mapname.c_str());
          
          std::string suffix = mapname.substr(mapname.length()-3, 3);
          if (suffix== ".bt"){ // write to binary file:
            if (!octree->writeBinary(mapname)){
              ROS_ERROR("Error writing to file %s", mapname.c_str());
            }
          } else if (suffix == ".ot"){ // write to full .ot file:
            if (!octree->write(mapname)){
              ROS_ERROR("Error writing to file %s", mapname.c_str());
            }
          } else{
            ROS_ERROR("Unknown file extension, must be either .bt or .ot");
          }


        } else{
          ROS_ERROR("Error reading OcTree from stream");
        }

        delete octree;

      }
    }
};

bool saveOctomapCallback(mapper_dyn_huk::espeleoSaveOctomap::Request  &req,
         mapper_dyn_huk::espeleoSaveOctomap::Response &res)
{

  try{
    MapSaver ms(req.output_file, req.is_full);
  }catch(std::runtime_error& e){
    ROS_ERROR("octomap_saver exception: %s", e.what());
    res.success = false;
    return false;
  }

  res.success = true;
  return true;
}

int main(int argc, char * argv[])
{
    //  Initial node
    ros::init(argc, argv, "octomap_saver_service");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ROS_INFO("Start octomap_saver_service node");

    ros::ServiceServer service = nh.advertiseService("/espeleo_octomap_saver", saveOctomapCallback);
    ros::spin();

    ROS_INFO("octomap_saver_service node stopped");

    return 0;
}
