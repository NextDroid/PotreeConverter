//
// Created by Karthik Sivarama Krishnan on 10/24/18.
//


#include <fstream>
#include <iostream>
#include <vector>

#include <experimental/filesystem>
#include <DataSchemas/LidarWorld_generated.h>
#include <DataSchemas/GroundTruth_generated.h>



#include "FlatBufferReader.hpp"
#include "stuff.h"

#include "Point.h"
#include "PointReader.h"
#include "PointAttributes.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>


using std::ifstream;
using std::cout;
using std::endl;
using std::vector;
using std::ios;



namespace Potree{


    FlatBufferReader::FlatBufferReader(string path, AABB aabb, double scale, PointAttributes pointAttributes, string flat_buffer ) : endOfFile(false), count(1), pointsLength(0), numberOfBytes(0), fileSize(0),  counter(0){
        this->path = path;
        this->aabb = aabb;
        this->scale = scale;
        this->attributes = pointAttributes;
        buffer = new unsigned char[4];
        flatBufferFileType= flat_buffer;
        std::cout<<"file type  points  for points and  bbox for bounding box points = " <<flatBufferFileType <<std::endl;
        std::cout << "Filepath = " << path << std::endl;

        if(fs::is_directory(path)){
            for(fs::directory_iterator it(path); it != fs::directory_iterator(); it++){
                fs::path filepath = it->path();
                if(fs::is_regular_file(filepath)){
                    files.push_back(filepath.string());
                }
            }
        }else{
            files.push_back(path);
        }
        currentFile = files.begin();
        reader = new ifstream(*currentFile, ios::in | ios::binary);
        bool firstCloudPopulated = populatePointCloud();
        if (!firstCloudPopulated) {
            std::cerr << "Could not populate first cloud" << std::endl;
        }
        // Calculate AABB:
        if (true ) {
            pointCount = 0;
            while(readNextPoint()) {

                p = getPoint();
                if (pointCount == 0) {
                    this->aabb = AABB(p.position);
                } else {
                    this->aabb.update(p.position);
                }
                pointCount++;
            }}
        reader->clear();
        reader->seekg(0, reader->beg);
        currentFile = files.begin();
        reader = new ifstream(*currentFile, ios::in | ios::binary);

    }

    FlatBufferReader::~FlatBufferReader(){

        close();

    }

    void FlatBufferReader::close(){
        if(reader != NULL){
            reader->close();
            delete[] buffer;
            delete reader;
            reader = NULL;

        }
    }

    long long FlatBufferReader::numPoints(){
        return pointCount;
    }

    bool FlatBufferReader::populatePointCloud()  {

        /* Reads 4 bytes everytime when the code reaches the end of segment  */

        try{
            std::cout.precision(std::numeric_limits<double>::max_digits10);
            reader->read(reinterpret_cast<char *>(buffer), 4);
            numberOfBytes = (uint32_t) buffer[3] << 24 |
                            (uint32_t) buffer[2] << 16 |
                            (uint32_t) buffer[1] << 8 |
                            (uint32_t) buffer[0];
            fileSize+= numberOfBytes;
            if (numberOfBytes==0){
                endOfFile = false;

                std::cout << "END OF FILE REACHED" << std::endl;
                return false;
            }
            else{
                buf2.clear();
                buf2.reserve(numberOfBytes);
                if (reader->eof()){
                    std::cerr << "Reader is at end of file" << std::endl;
                    endOfFile = false;
                    return false;
                }
                reader->read(&buf2[0], numberOfBytes);
                if (flatBufferFileType== "point")
                {
                    pointcloud = LIDARWORLD::GetPointCloud(&buf2[0]);
                    pos = pointcloud->points();
                    pointsLength = pos->Length();
                    if(pointsLength == 0)
                        return true;
                }
                else if (flatBufferFileType=="bbox"){
                    track = flatbuffers::GetRoot<Flatbuffer::GroundTruth::Track>(&buf2[0]);
                    statesFb = track->states();
                    statesLength = statesFb->Length();
                    return  true;}
            }
        }
        catch (std::exception& e) {
            endOfFile = false;
            std::cout << "No More Points Left" << std::endl;
            return false;
        }

    }



    bool FlatBufferReader::centroid(){
        /*Function used to calculate the centroid and edges of every bounding box passed and populates the points to the AABB  */

        if(counter==0){
            auto &state = *statesFb;
            for(int stateIdx=0;stateIdx<statesLength;stateIdx++) {
                bbox = state[stateIdx]->bbox();
                auto bbox_len = bbox->Length();
                timeStamps = state[stateIdx]->timestamps();
                Yaw = state[stateIdx]->yaw();

                //Reads all the vertices points
                for (int bboxIdx = 0; bboxIdx < bbox_len; bboxIdx++) {
                    Vertices.x = bbox->Get(bboxIdx)->x();
                    Vertices.y = bbox->Get(bboxIdx)->y();
                    Vertices.z = bbox->Get(bboxIdx)->z();
                    Points.push_back({Vertices.x, Vertices.y, Vertices.z});

                }

                //Reads all the face centers
                for (int face_Idx = 0; face_Idx < 3; face_Idx++) {


                    Points.push_back({((bbox->Get(face_Idx)->x()) + (bbox->Get(face_Idx + 5)->x())) / 2,
                                      ((bbox->Get(face_Idx)->y()) + (bbox->Get(face_Idx + 5)->y())) / 2,
                                      ((bbox->Get(face_Idx)->z()) + (bbox->Get(face_Idx + 5)->z())) / 2});
                }
                Points.push_back({((bbox->Get(3)->x()) + (bbox->Get(4)->x())) / 2,
                                  ((bbox->Get(3)->y()) + (bbox->Get(4)->y())) / 2,
                                  ((bbox->Get(3)->z()) + (bbox->Get(4)->z())) / 2});
            }
            point.position.x = Points[counter].bbox_x;
            point.position.y = Points[counter].bbox_y;
            point.position.z = Points[counter].bbox_z;
            point.gpsTime = timeStamps;
            counter++;
            return true;

        }
        else{ if (counter < Points.size()) {

                point.position.x = Points[counter].bbox_x;
                point.position.y = Points[counter].bbox_y;
                point.position.z = Points[counter].bbox_z;
                point.gpsTime = timeStamps;
                counter++;
                return true;

            }
            else{
                Points.clear();
                counter=0;
                populatePointCloud();
                centroid();

            }
        }


    }

    bool FlatBufferReader::readNextPoint() {
        /* This is the main function which passes the read points to the AABB function everytime. */

        bool hasPoints = reader->good();


        if (!hasPoints) {
            currentFile++;

            if (currentFile != files.end()) {
                reader->close();
                delete reader;
                reader = NULL;
                reader = new ifstream(*currentFile, ios::in | ios::binary);
                hasPoints = reader->good();
            }
        }

        if(hasPoints && !endOfFile) {


            if (count < pointsLength) {
                auto fbPoints = pos->Get(count);
                count++;
                point.position.x = fbPoints->x();
                point.position.y = fbPoints->y();
                point.position.z = fbPoints->z();
                point.gpsTime = fbPoints->timestamp();
                point.intensity = fbPoints->intensity();
                return true;
            } else if (count == pointsLength) {
                count = 0;
                if (populatePointCloud()) {
                    auto fbPoints = pos->Get(count);
                    count++;
                    point.position.x = fbPoints->x();
                    point.position.y = fbPoints->y();
                    point.position.z = fbPoints->z();
                    point.gpsTime = fbPoints->timestamp();
                    point.intensity = fbPoints->intensity();
                    return true;
                }
                else{endOfFile = false;
                    std::cout << "reader reached the  end of file" << std::endl;
                    return false;}
            }
            else if (flatBufferFileType=="bbox" ) {

                centroid();
            }
            else{endOfFile = false;
                std::cout << "end of file" << std::endl;
                return false;}
        }
        return hasPoints;
    }
    Point FlatBufferReader::getPoint() {
        return point;
    }
    AABB FlatBufferReader::getAABB() {
        return aabb;
    }
}


