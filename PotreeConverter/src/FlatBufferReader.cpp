/*
 * @file  FlatBufferReader.cpp
 *
 * @brief Reads a Flatbuffer file with extension ‘.lidar’
 * @param[in] FlatBuffer file path along with string -b “point” ||  -b “bbox” || -b "lanes" || -b "detections" || -b "rtk"
 *
 * @author Karthik Sivarama Krishnan
 * @date October 24, 2018, 10:36 AM
 */


#include <fstream>
#include <iostream>
#include <vector>

#include <experimental/filesystem>
#include <DataSchemas/LidarWorld_generated.h>
#include <DataSchemas/GroundTruth_generated.h>

#include "FlatBufferReader.h"
#include "stuff.h"

#include "Point.h"
#include "PointReader.h"


using std::ifstream;
using std::cout;
using std::endl;
using std::vector;
using std::ios;



namespace Potree{


    FlatBufferReader::FlatBufferReader(string path, AABB aabb,  string flatBufferType ) :  count(1), pointsLength(0), counter(0), laneCounter(0), detectionCounter(0), rtkCounter(0) {
        this->path = path;
        this->aabb = aabb;
        flatBufferFileType = flatBufferType;
        std::cout<<" The FlatBuffer file type is  = " << flatBufferFileType << std::endl;
        std::cout << "Filepath = " << path << std::endl;

        if (fs::is_directory(path)) {
            for (fs::directory_iterator it(path); it != fs::directory_iterator(); it++) {

                fs::path filepath = it->path();

                if (fs::is_regular_file(filepath)) {
                    files.push_back(filepath.string());
                }
            }
        }
        else {
            files.push_back(path);
        }

        currentFile = files.begin();
        reader = std::make_unique<ifstream>(ifstream(*currentFile, ios::in | ios::binary));

        //     Check if there are any points.

        bool firstCloudPopulated = populatePointCloud();
        if (!firstCloudPopulated) {
            std::cerr << "Could not populate first cloud" << std::endl;
        }

        //      Calculate AABB: for every point available in the flatbuffer file

        pointCount = 0;
        while(readNextPoint()) {

            const Point p = getPoint();
            if (pointCount == 0) {
                this->aabb = AABB(p.position);
            }
            else {
                this->aabb.update(p.position);
            }
            pointCount++;
        }
        reader->clear();
        reader->seekg(0, reader->beg);
        currentFile = files.begin();
        reader = std::make_unique<ifstream>(ifstream(*currentFile, ios::in | ios::binary));
    }

    FlatBufferReader::~FlatBufferReader(){
       close();

    }

    void FlatBufferReader::close(){

     // Stub Function. close() function is declared as virtual in the abstract class "PointReader"
    }

    int64_t FlatBufferReader::numPoints(){
        return pointCount;
    }

    bool FlatBufferReader::populatePointCloud()  {
        /** @brief This function is called every time to read 4 bytes of data from flatbuffer file, when the code reaches the end of segment
         *  @param[in] reader to read the 4 bytes
         *  @return bool
         */

        try{
            uint8_t buffer[4];

            std::cout.precision(std::numeric_limits<double>::max_digits10);
            reader->read(reinterpret_cast<char *>(buffer), 4);
            if (reader->eof()) {
                std::cerr << "Reader is at end of file" << std::endl;

                return false;
            }


            const uint64_t numberOfBytes = buffer[3] << 24 |
                                           buffer[2] << 16 |
                                           buffer[1] << 8 |
                                           buffer[0];
            if (numberOfBytes == 0) {
                std::cout << "END OF FILE REACHED" << std::endl;
                return false;
            }
            else {
                buf2.clear();
                buf2.reserve(numberOfBytes);

                reader->read(&buf2[0], numberOfBytes);

                //    For the flatbuffer file of type pointcloud points using the Schema -> DataSchemas/schemas/LIDARWORLD.fbs

                if (flatBufferFileType == "point")
                {
                    auto pointcloud = LIDARWORLD::GetPointCloud(&buf2[0]);
                    pos             = pointcloud->points();
                    pointsLength    = pos->Length();
                    if (pointsLength != 0)
                        return true;
                }

                    //    For the flatbuffer file of type track bounding box pointcloud using the Schema -> DataSchemas/schemas/GroundTruth.fbs

                else if (flatBufferFileType == "bbox")
                {
                    auto track     = flatbuffers::GetRoot<Flatbuffer::GroundTruth::Track>(&buf2[0]);
                    statesFb       = track->states();
                    statesLength   = statesFb->Length();
                    return  true;
                }

                    //    For the flatbuffer file of type Lanes pointcloud using the Schema -> DataSchemas/schemas/GroundTruth.fbs

                else if (flatBufferFileType == "lanes")
                {
                    auto Lane       = flatbuffers::GetRoot<Flatbuffer::GroundTruth::Lane>(&buf2[0]);
                    rightLane       = Lane->right();
                    leftLane        = Lane->left();
                    spine           = Lane ->spine();
                    rightLaneLength = rightLane->Length();
                    leftLaneLength  = leftLane->Length();
                    spineLength     = spine->Length();
                    return  true;
                }

                    //    For the flatbuffer file of type Detections pointcloud using the Schema -> DataSchemas/schemas/GroundTruth.fbs

                else if (flatBufferFileType == "detections")
                {
                    auto Detection   = flatbuffers::GetRoot<Flatbuffer::GroundTruth::Detections>(&buf2[0]);
                    center           = Detection->detections();
                    detectionLength  = center->Length();
                    return true;
                }

                    //    For the flatbuffer file of type rtk pointcloud using the Schema -> DataSchemas/schemas/GroundTruth.fbs

                else if (flatBufferFileType == "rtk")
                {
                    auto  rtk   = flatbuffers::GetRoot<Flatbuffer::GroundTruth::Poses>(&buf2[0]);
                    rtkPose     = rtk->poses();
                    rtkLength   = rtkPose->Length();

                    return  true;
                }
            }
        }
        catch (std::exception& e) {

            std::cout << "No More Points Left" << std::endl;
            return false;
        }

    }

    bool FlatBufferReader::lanePoints() {

        /** @brief This function is used to combine all the lane points (Left,Right,Spine) into a vector of LanePoints for every 4 bytes of data
         *  @param[in] reader with 4 bytes of data
         *  @return bool
         */

        // Assign all the lane points into a vetor for every segment.

        for (int leftLaneidx = 0; leftLaneidx < leftLaneLength; leftLaneidx++) {
            LanePoints.push_back({leftLane->Get(leftLaneidx)->x(), leftLane->Get(leftLaneidx)->y(), leftLane->Get(leftLaneidx)->z()});
        }

        for (int rightLaneidx = 0; rightLaneidx < rightLaneLength; rightLaneidx++) {
            LanePoints.push_back({rightLane->Get(rightLaneidx)->x(), rightLane->Get(rightLaneidx)->y(), rightLane->Get(rightLaneidx)->z()});
        }

        for (int spineidx = 0; spineidx < spineLength; spineidx++) {
            LanePoints.push_back({spine->Get(spineidx)->x(), spine->Get(spineidx)->y(), spine->Get(spineidx)->z()});
        }
    }

    bool FlatBufferReader::centroid(){

        /** @brief This function is used to calculate the centroid and edges of every bounding box passed and creates a vector of struct “Points” then populates these points to the AABB function.
         *  @param[in] reader with 4 bytes of data
         *  @return bool
         */


        double timeStamps;
        if (counter == 0) {
            auto &state = *statesFb;

            for (int stateIdx = 0;stateIdx < statesLength;stateIdx++) {

                bbox          = state[stateIdx]->bbox();
                auto bbox_len = bbox->Length();
                timeStamps    = state[stateIdx]->timestamps();

                //Reads all the track bounding box vertices points

                for (int bboxIdx = 0; bboxIdx < bbox_len; bboxIdx++) {
                    Points.push_back({bbox->Get(bboxIdx)->x(), bbox->Get(bboxIdx)->y(), bbox->Get(bboxIdx)->z()});

                }

                //Reads all the track bounding box face centers

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
            point.gpsTime    = timeStamps - 1495189467.6400001;  //Hardcoded values should be fixed in the Potree Visualizer
            counter++;
            return true;

        }
        else{ if (counter < Points.size()) {

                point.position.x = Points[counter].bbox_x;
                point.position.y = Points[counter].bbox_y;
                point.position.z = Points[counter].bbox_z;
                point.gpsTime    = timeStamps - 1495189467.6400001; //Hardcoded values should be fixed in the Potree Visualizer
                counter++;
                return true;

            }
            else {
                Points.clear();
                counter = 0;
                populatePointCloud();
                centroid();
            }
        }


    }



    bool FlatBufferReader::egoDimensions() {
        /*    WORK IN PROGRESS
         *
         */

        for(int rtkcounter = 0; rtkcounter < rtkLength; rtkcounter++)
        { auto rtkPoints = rtkPose->Get(rtkcounter);
            ego.push_back({rtkPoints->pos()->x(),  rtkPoints->pos()->y(),  rtkPoints->pos()->z(), rtkPoints->timestamp()});
            ego.push_back({-rtkPoints->pos()->y(), rtkPoints->pos()->x()-1.165, rtkPoints->pos()->z()-10.553, rtkPoints->timestamp()});
            ego.push_back({rtkPoints->pos()->y(), -rtkPoints->pos()->x()+2.435, rtkPoints->pos()->z()-10.553, rtkPoints->timestamp()});

        }

    }





    bool FlatBufferReader::readNextPoint() {
        /** @brief This is the main driver function which checks for the points in the file and populate the points to the AABB function.
         *  @param[in] reader with 4 bytes of data
         *  @return bool
         */


        bool hasPoints = reader->good();
        if(hasPoints ) {
            if (flatBufferFileType == "point" ) {
                // check if the end of 4 bytes segment reached

                if (count < pointsLength) {
                    auto fbPoints    = pos->Get(count);
                    count++;
                    point.position.x = fbPoints->x();
                    point.position.y = fbPoints->y();
                    point.position.z = fbPoints->z();
                    point.gpsTime    = fbPoints->timestamp();
                    point.intensity  = fbPoints->intensity();
                    return true;
                }
                    //if end of 4 bytes reached, then read the next 4 bytes.

                else if (count == pointsLength) {
                    count = 0;
                    if (populatePointCloud()) {
                        auto fbPoints    = pos->Get(count);
                        count++;
                        point.position.x = fbPoints->x();
                        point.position.y = fbPoints->y();
                        point.position.z = fbPoints->z();
                        point.gpsTime    = fbPoints->timestamp();
                        point.intensity  = fbPoints->intensity();
                        return true;
                    }
                    else {

                        std::cout << "There are no More Pointcloud Points Left in the file" << std::endl;
                        return false;
                    }
                }
            }
            else if (flatBufferFileType == "bbox" ) {

                centroid();

            }
            else if (flatBufferFileType == "lanes") {

                if (laneCounter == 0) {
                    lanePoints();

                }

                if (laneCounter < LanePoints.size()) {

                    point.position.x = LanePoints[laneCounter].lane_x;
                    point.position.y = LanePoints[laneCounter].lane_y;
                    point.position.z = LanePoints[laneCounter].lane_z;
//                    point.gpsTime = Lane->timestamp()->Get(laneCounter);
                    laneCounter++;
                    return true;
                }
                else if (laneCounter == LanePoints.size()) {
                    laneCounter = 0;
                    LanePoints.clear();
                    if (populatePointCloud()) {
                        lanePoints();

                        point.position.x = LanePoints[laneCounter].lane_x;
                        point.position.y = LanePoints[laneCounter].lane_y;
                        point.position.z = LanePoints[laneCounter].lane_z;
//                        point.gpsTime = Lane->timestamp()->Get(laneCounter);
                        laneCounter++;
                        return true;
                    }
                    else {
                        std::cout << "There are no More Lane Points Left in the file" << std::endl;
                        return false;
                    }
                }
            }
            else if (flatBufferFileType == "detections" ) {

                if (detectionCounter < detectionLength) {
                    auto detectionPoints = center->Get(detectionCounter);

                    point.position.x = detectionPoints->centroid()->x();
                    point.position.y = detectionPoints->centroid()->y();
                    point.position.z = detectionPoints->centroid()->z();
                    point.gpsTime    = detectionPoints->timestamp() - 1495189467.6400001; //Hardcoded values should be fixed in the Potree Visualizer
                    std::cout<<point.gpsTime<<std::endl;
                    detectionCounter++;
                    return true;
                }
                    //if end of 4 bytes reached, then read the next 4 bytes.

                else if (detectionCounter == detectionLength) {
                    detectionCounter = 0;
                    if (populatePointCloud()) {
                        auto detectionPoints = center->Get(detectionCounter);

                        point.position.x = detectionPoints->centroid()->x();
                        point.position.y = detectionPoints->centroid()->y();
                        point.position.z = detectionPoints->centroid()->z();
                        point.gpsTime    = detectionPoints->timestamp() - 1495189467.6400001; //Hardcoded values should be fixed in the Potree Visualizer
//                        std::cout<<point.gpsTime<<std::endl;
                        detectionCounter++;
                        return true;
                    }
                }
            }
            else if (flatBufferFileType == "rtk" ) {

                // check if the end of 4 bytes segment reached
                if (rtkCounter < rtkLength && rtkCounter==0) {
                    egoDimensions();
                    point.position.x = ego[rtkCounter].ego_x;
                    point.position.y = ego[rtkCounter].ego_y;
                    point.position.z = ego[rtkCounter].ego_z;
                    point.gpsTime    = ego[rtkCounter].ego_time - 1495189467.6400001; //Hardcoded values should be fixed in the Potree Visualizer
                    rtkCounter++;
                    return true;
                }
                    //if end of 4 bytes reached, then read the next 4 bytes.
                else if (rtkCounter < ego.size()) {
                    point.position.x = ego[rtkCounter].ego_x;
                    point.position.y = ego[rtkCounter].ego_y;
                    point.position.z = ego[rtkCounter].ego_z;
                    point.gpsTime    = ego[rtkCounter].ego_time - 1495189467.6400001; //Hardcoded values should be fixed in the Potree Visualizer
                    rtkCounter++;
                    return true;
                }
                else if (rtkCounter == ego.size()) {
                    rtkCounter = 0;
                    if (populatePointCloud()) {
                        egoDimensions();
                        point.position.x = ego[rtkCounter].ego_x;
                        point.position.y = ego[rtkCounter].ego_y;
                        point.position.z = ego[rtkCounter].ego_z;
                        point.gpsTime    = ego[rtkCounter].ego_time - 1495189467.6400001; //Hardcoded values should be fixed in the Potree Visualizer
                        return true;
                    }
                    else {

                        std::cout << "There are no More Pointcloud Points Left in the file" << std::endl;
                        return false;
                    }
                }
            }
        }
        else  {
            currentFile++;

            if (currentFile != files.end()) {
                reader->close();
                reader = std::make_unique<ifstream>(ifstream(*currentFile, ios::in | ios::binary));
                hasPoints = reader->good();
            }
        }
        return hasPoints;
    }    Point FlatBufferReader::getPoint() {
        return point;
    }
    AABB FlatBufferReader::getAABB() {
        return aabb;
    }
}


