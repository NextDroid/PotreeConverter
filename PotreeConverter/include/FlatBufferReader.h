
/*
 * @file  FlatBufferReader.h
 * @brief
 * @author Karthik Sivarama Krishnan
 * @date October 24, 2018, 10:36 AM
 */


#ifndef VERITAS_FLATBUFFERREADER_H
#define VERITAS_FLATBUFFERREADER_H

#include "AABB.h"
#include "Point.h"
#include "PointReader.h"
#include <memory>
#include <fstream>
#include <DataSchemas/LidarWorld_generated.h>
#include <DataSchemas/GroundTruth_generated.h>



using std::string;

using std::ifstream;
using std::cout;
using std::endl;
using std::vector;


namespace Potree{

    class FlatBufferReader : public PointReader{

    public:

        FlatBufferReader(string path, AABB aabb,  string flatBufferType);


        ~FlatBufferReader();

        bool readNextPoint();
        bool populatePointCloud();

        bool centroid();
        bool lanePoints();
        bool egoDimensions();

        Point getPoint();

        AABB getAABB();
        int64_t numPoints();

        string flatBufferFileType;
        int count,counter,laneCounter,detectionCounter,rtkCounter;
        int pointsLength,statesLength,rightLaneLength,leftLaneLength,spineLength, detectionLength,rtkLength;
        void close();

    private:
        AABB aabb;
        double scale;
        string path;
        vector<string> files;
        vector<string>::iterator currentFile;
        std::unique_ptr<ifstream>reader;

        Point point, p;
        uint64_t pointCount;
        const flatbuffers::Vector<const LIDARWORLD::Point *> *pos;
        const LIDARWORLD::PointCloud *pointcloud;
        const Flatbuffer::GroundTruth::State *states;
        const Flatbuffer::GroundTruth::Lane *Lane;
        const Flatbuffer::GroundTruth::Detections *Detection;
        const flatbuffers::Vector<const Flatbuffer::GroundTruth::Vec3 *> *bbox, *rightLane,*leftLane,*spine;
        const flatbuffers::Vector<flatbuffers::Offset<Flatbuffer::GroundTruth::Detection>> *center;
        const Flatbuffer::GroundTruth::Track *track;
        const flatbuffers::Vector<flatbuffers::Offset<Flatbuffer::GroundTruth::State>> *statesFb;

        const Flatbuffer::GroundTruth::Poses *rtk;

        const flatbuffers::Vector<flatbuffers::Offset<Flatbuffer::GroundTruth::Pose>> *rtkPose;

        std::vector<char> buf2;

        struct bboxPoints{
            double bbox_x;
            double bbox_y;
            double bbox_z;
        };


        struct LanePoints{
            double lane_x;
            double lane_y;
            double lane_z;
        };
        struct egoPoints{
            double ego_x;
            double ego_y;
            double ego_z;
        };
        std::vector<bboxPoints>Points;
        std::vector<LanePoints> LanePoints;
        std::vector<egoPoints>ego;

    };
}
#endif //VERITAS_FLATBUFFERREADER_H