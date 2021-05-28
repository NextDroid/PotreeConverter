
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
#include <DataSchemas/Lidar_generated.h>
#include <DataSchemas/GroundTruth_generated.h>

#include "functions/VTM_functions.h"

using std::string;

using std::ifstream;
using std::cout;
using std::endl;
using std::vector;


namespace Potree{

    class FlatBufferReader : public PointReader{

    public:

        FlatBufferReader(string path, AABB aabb,  string flatBufferType, string metadataProcessingFile);


        ~FlatBufferReader();
        bool readNextPoint() override;   // Declared Virtual in the base class PointReader
        Point getPoint() override;      // Declared Virtual in the base class PointReader
        AABB getAABB() override;       // Declared Virtual in the base class PointReader
        int64_t numPoints() override; // Declared Virtual in the base class PointReader
        void close()  ;
        ifstream *reader;    // Declared Virtual in the base class PointReader
        double test;
        unsigned char *buffer;
    private:
        AABB aabb;

        vector<string> files;
        vector<string>::iterator currentFile;


        Point point;
        uint64_t pointCount;
        const flatbuffers::Vector<const Flatbuffer::LIDAR::Point *> *points;
        const Flatbuffer::GroundTruth::Points *classifiedPoints;
        const flatbuffers::Vector<const Flatbuffer::GroundTruth::Vec3 *>  *rightLane,*leftLane,*spine;
        const flatbuffers::Vector<flatbuffers::Offset<Flatbuffer::GroundTruth::Detection>> *detectionCenter;
        const flatbuffers::Vector<flatbuffers::Offset<Flatbuffer::GroundTruth::State>> *statesFb;
        const Flatbuffer::GroundTruth::Lane *Lane;

        std::vector<char> readerBuffer;

        struct bboxPoints{
            double bbox_x;
            double bbox_y;
            double bbox_z;
        };


        struct LanePoints{
            double lane_x;
            double lane_y;
            double lane_z;
            double lane_gps;
        };
        struct egoPoints{
            double ego_x;
            double ego_y;
            double ego_z;
            double ego_time;
        };
        std::vector<bboxPoints>Points;
        std::vector<LanePoints> LanePoints;
        std::vector<egoPoints>ego;

        VTMmetadata vtmMetadata;

        bool prepareNextSegment();

        bool centroid();
        bool lanePoints();
        bool egoDimensions();


        string flatBufferFileType;
        int totalNumPoints;
        int numSegmentsRead;
        int pointsIdx,bboxPointsIdx,laneIdx,detectionIdx,rtkIdx;
        int pointsLength,statesLength,rightLaneLength,leftLaneLength,spineLength, detectionLength,rtkLength;

    };
}
#endif //VERITAS_FLATBUFFERREADER_H
