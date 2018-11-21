
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
#include "PointAttributes.hpp"


#include <memory>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
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

        FlatBufferReader(string path, AABB aabb,  string flat_buffer);


        ~FlatBufferReader();

        bool readNextPoint();
        bool populatePointCloud();

        bool centroid();


        Point getPoint();

        AABB getAABB();
        long long numPoints();

        string flatBufferFileType;
        int count,  counter, laneCounter;
        int pointsLength, statesLength,rightLaneLength;
        void close();

    private:
        AABB aabb;
        double scale;
        string path;
        vector<string> files;
        vector<string>::iterator currentFile;
        ifstream *reader;
        PointAttributes attributes;
        Point point, p;
        long long pointCount;
        bool endOfFile;
        const flatbuffers::Vector<const LIDARWORLD::Point *> *pos;
        const LIDARWORLD::PointCloud *pointcloud;
        const Flatbuffer::GroundTruth::State *states;
        const Flatbuffer::GroundTruth::Lane *Lane;
        const flatbuffers::Vector<const Flatbuffer::GroundTruth::Vec3 *> *bbox;
        const flatbuffers::Vector<const Flatbuffer::GroundTruth::Vec3 *> *rightLane;
        const Flatbuffer::GroundTruth::Track *track;
        const flatbuffers::Vector<flatbuffers::Offset<Flatbuffer::GroundTruth::State>> *statesFb;

        unsigned char *buffer;
        std::vector<char> buf2;

        Vector3<double>Vertices;
        struct bboxPoints{
            double bbox_x;
            double bbox_y;
            double bbox_z;
        };

        std::vector<bboxPoints>Points;


    };
}
#endif //VERITAS_FLATBUFFERREADER_H