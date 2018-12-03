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
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>

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

        path                     = path;
        this->aabb               = aabb;
        this->flatBufferFileType = flatBufferType;

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
        fileReader  = std::make_unique<ifstream>(ifstream(*currentFile, ios::in | ios::binary));

        //     Check if there are any points.

        bool firstCloudPopulated = populatePointCloud();
        if (!firstCloudPopulated) {
            std::cerr << "Could not populate first cloud" << std::endl;
        }

        //      Calculate AABB: for every point available in the flatbuffer file override

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

        currentFile = files.begin();
        fileReader  = std::make_unique<ifstream>(ifstream(*currentFile, ios::in | ios::binary));
    }

    FlatBufferReader::~FlatBufferReader(){
        close() ;

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
            fileReader->read(reinterpret_cast<char *>(buffer), 4);
            if (fileReader->eof()) {
                std::cerr << "Reader is at end of file" << std::endl;

                return false;
            }


            const uint64_t numberOfBytes = buffer[3] << 24 |
                                           buffer[2] << 16 |
                                           buffer[1] << 8 |
                                           buffer[0];

            readerBuffer.clear();
            readerBuffer.reserve(numberOfBytes);

            fileReader->read(&readerBuffer[0], numberOfBytes);

            //    For the flatbuffer file of type pointcloud points using the Schema -> DataSchemas/schemas/LIDARWORLD.fbs

            if (flatBufferFileType == "point")
            {
                auto pointcloud = LIDARWORLD::GetPointCloud(&readerBuffer[0]);
                pos             = pointcloud->points();
                pointsLength    = pos->Length();
                if (pointsLength != 0)
                    return true;
            }

                //    For the flatbuffer file of type track bounding box pointcloud using the Schema -> DataSchemas/schemas/GroundTruth.fbs

            else if (flatBufferFileType == "bbox")
            {
                auto track     = flatbuffers::GetRoot<Flatbuffer::GroundTruth::Track>(&readerBuffer[0]);
                statesFb       = track->states();
                statesLength   = statesFb->Length();
                return  true;
            }

                //    For the flatbuffer file of type Lanes pointcloud using the Schema -> DataSchemas/schemas/GroundTruth.fbs

            else if (flatBufferFileType == "lanes")
            {
                Lane            = flatbuffers::GetRoot<Flatbuffer::GroundTruth::Lane>(&readerBuffer[0]);
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
                auto Detection   = flatbuffers::GetRoot<Flatbuffer::GroundTruth::Detections>(&readerBuffer[0]);
                detectionCenter  = Detection->detections();
                detectionLength  = detectionCenter->Length();
                return true;
            }

                //    For the flatbuffer file of type rtk pointcloud using the Schema -> DataSchemas/schemas/GroundTruth.fbs

            else if (flatBufferFileType == "rtk")
            {
                auto  rtk     = flatbuffers::GetRoot<Flatbuffer::GroundTruth::Poses>(&readerBuffer[0]);
                auto rtkPose  = rtk->poses();
                rtkLength     = rtkPose->Length();

                return  true;
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
            LanePoints.push_back({leftLane->Get(leftLaneidx)->x(), leftLane->Get(leftLaneidx)->y(), leftLane->Get(leftLaneidx)->z(),Lane->timestamp()->Get(leftLaneidx)});
        }

        for (int rightLaneidx = 0; rightLaneidx < rightLaneLength; rightLaneidx++) {
            LanePoints.push_back({rightLane->Get(rightLaneidx)->x(), rightLane->Get(rightLaneidx)->y(), rightLane->Get(rightLaneidx)->z(),Lane->timestamp()->Get(rightLaneidx)});
        }

        for (int spineidx = 0; spineidx < spineLength; spineidx++) {
            LanePoints.push_back({spine->Get(spineidx)->x(), spine->Get(spineidx)->y(), spine->Get(spineidx)->z(),Lane->timestamp()->Get(spineidx)});
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

                auto bbox     = state[stateIdx]->bbox();
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

/*    Should be Edited when we know the exact dimensions of the vehicle.
 *    This Function calculates the front and rear bumper dimensions and works on applying rotations to help be in the same plane.
 *
 *
// Performing Quaternion Rotation of the RTK points along the centroid of the ego vehicle.
    Eigen::Quaterniond
    euler2Quaternion( const double roll,
                      const double pitch,
                      const double yaw ) {
        // TODO check yaw pitch roll order required in use case
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

        Eigen::Quaterniond q = yawAngle *pitchAngle *rollAngle;
        return q;
    }

    Eigen::Matrix4d getTxMat(Eigen::Vector4d dx, Eigen::Vector3d dTheta){
        Eigen::Matrix3d rot_mat = euler2Quaternion(dTheta.x(),dTheta.y(),dTheta.z()).toRotationMatrix();
        Eigen::Matrix4d Trans; // Transformation matrix
        Trans.setIdentity();
        Trans.block<3,3>(0,0) = rot_mat;
        Trans.rightCols<1>() = dx;

        return Trans;
    }

    bool FlatBufferReader::egoDimensions() {

        Eigen::Vector4d  rtk,RotatedPoint,Rotated,centeroid, rearPoint, frontPoint;
        Eigen::Vector3d Yaw;

        for(int rtkcounter = 0; rtkcounter < rtkLength; rtkcounter++) {
            auto rtkPoints = rtkPose->Get(rtkcounter);

            //Defining the rotations around the yaw axis.
            Yaw(0) = 0;
            Yaw(1) = 0;
            Yaw(2) = rtkPoints->orientation()->z();

            //considering the rtk point in the world frame 0,0,0
            rtk(0) = rtk(1) = rtk(2) = 0;
            rtk(3) = 1;

            // Defining The centroid, front bumper (2.435,0,0) and the rear bumper points (-1.165,0,0)
            centeroid(0) = centeroid(1) = centeroid(2) = 0;

            centeroid(3) = 1;
            rearPoint(0) = -1.165;
//            frontPoint(0) = 2.435;


            rearPoint(1) = rearPoint(2) = frontPoint(2) = 0;
            rearPoint(3) = frontPoint(3) = 1;

            for (frontPoint(1) = 0; frontPoint(1) > -1; frontPoint(1) -= 0.2) {
                for (frontPoint(0)= 2.2;frontPoint(0)<3;frontPoint(0)+=0.2){
//                    std::cout<<frontPoint(0);
                centeroid = (rearPoint + frontPoint) / 2;

                //Subtracting the centroid
                auto centroidrearPoint = rearPoint - centeroid;
                auto centroidfrontPoint = frontPoint - centeroid;

                rtk = rtk - centeroid;

                //applying rotations
                auto rearRotationMatrix = getTxMat(centroidrearPoint, Yaw);
                auto rotatedfrontPoint = (rearRotationMatrix * centeroid);
                auto frontRotationMatrix = getTxMat(centroidfrontPoint, Yaw);
                auto rotatedrearPoint = (frontRotationMatrix * centeroid);
//                std::cout<<"rear rotations"<<rearPoint<<std::endl;

                auto rtkRotationMatrix = getTxMat(rtk, Yaw);
                auto rotatedrtk = (rtkRotationMatrix * centeroid);
                auto centroidRotationMatrix = getTxMat(centeroid, Yaw);
                auto rotatedcenteroid = (centroidRotationMatrix * centeroid);

                //subtract rtk

                auto rtksubtractedcenteroid = rotatedcenteroid - rotatedrtk;
                auto rtksubtracted = rotatedrtk - rotatedrtk;
                auto rtksubtractedfrontPoint = rotatedfrontPoint - rotatedrtk; //change here karthik
                auto rtksubtractedrearPoint = rotatedrearPoint - rotatedrtk;


                ego.push_back({rtksubtractedcenteroid(0) + rtkPoints->pos()->x(),
                               rtkPoints->pos()->y() + rtksubtractedcenteroid(1),
                               rtkPoints->pos()->z() + rtksubtractedcenteroid(2), rtkPoints->timestamp()});
                ego.push_back({rtksubtractedfrontPoint(0) + rtkPoints->pos()->x(),
                               rtkPoints->pos()->y() + rtksubtractedfrontPoint(1),
                               rtkPoints->pos()->z() + rtksubtractedfrontPoint(2), rtkPoints->timestamp()});
                ego.push_back({rtksubtractedrearPoint(0) + rtkPoints->pos()->x(),
                               rtkPoints->pos()->y() + rtksubtractedrearPoint(1),
                               rtkPoints->pos()->z() + rtksubtractedrearPoint(2), rtkPoints->timestamp()});
                ego.push_back(
                        {rtksubtracted(0) + rtkPoints->pos()->x(), rtkPoints->pos()->y() + rtksubtracted(1),
                         rtkPoints->pos()->z() + rtksubtracted(2),
                         rtkPoints->timestamp()});
            }
        }
        }


    }




*/
    bool FlatBufferReader::readNextPoint() {
        /** @brief This is the main driver function which checks for the points in the file and populate the points to the AABB function.
         *  @param[in] reader with 4 bytes of data
         *  @return bool
         */


        bool hasPoints = fileReader->good();
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
                    point.gpsTime    = LanePoints[laneCounter].lane_gps;
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
                        point.gpsTime    = LanePoints[laneCounter].lane_gps;
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
                    auto detectionPoints = detectionCenter->Get(detectionCounter);

                    point.position.x = detectionPoints->centroid()->x();
                    point.position.y = detectionPoints->centroid()->y();
                    point.position.z = detectionPoints->centroid()->z();
                    point.gpsTime    = detectionPoints->timestamp() - 1495189467.6400001; //Hardcoded values should be fixed in the Potree Visualizer
                    detectionCounter++;
                    return true;
                }
                    //if end of 4 bytes reached, then read the next 4 bytes.

                else if (detectionCounter == detectionLength) {
                    detectionCounter = 0;
                    if (populatePointCloud()) {
                        auto detectionPoints = detectionCenter->Get(detectionCounter);

                        point.position.x = detectionPoints->centroid()->x();
                        point.position.y = detectionPoints->centroid()->y();
                        point.position.z = detectionPoints->centroid()->z();
                        point.gpsTime    = detectionPoints->timestamp() - 1495189467.6400001; //Hardcoded values should be fixed in the Potree Visualizer
                        detectionCounter++;
                        return true;
                    }
                }
            }
            else if (flatBufferFileType == "rtk" ) {

                // check if the end of 4 bytes segment reached
                if (rtkCounter < rtkLength && rtkCounter==0) {
                    //egoDimensions();                             Should be Edited when we have the dimensions of the vehicle provided by the LG.
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
//                        egoDimensions();
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
                fileReader    = std::make_unique<ifstream>(ifstream(*currentFile, ios::in | ios::binary));
                hasPoints     = fileReader->good();
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


