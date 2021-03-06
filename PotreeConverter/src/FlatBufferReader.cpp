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
#include <DataSchemas/Lidar_generated.h>
#include <DataSchemas/GroundTruth_generated.h>
//#include <DataSchemas/VisualizationPrimitives_generated.h>
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

uint64_t totalPoints = 0;

namespace Potree{


    FlatBufferReader::FlatBufferReader(string path, AABB aabb,  string flatBufferType, string metadataProcessingFile ) :  pointsIdx(0), pointsLength(0), numSegmentsRead(0), totalNumPoints(0), bboxPointsIdx(0), laneIdx(0), detectionIdx(0), rtkIdx(0) {


        std::cout << "========================" << std::endl;
        std::cout << "FLATBUFFER READER AABB: " << aabb;
        std::cout << "========================" << std::endl;

        vtmMetadata = parseVTMmetadata(metadataProcessingFile);

        this->aabb               = aabb;
        this->flatBufferFileType = flatBufferType;
        buffer = new unsigned char[4];
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
        reader = new ifstream(*currentFile, ios::in | ios::binary);

        //     Check if there are any points.

        bool firstCloudPopulated = prepareNextSegment();
        if (!firstCloudPopulated) {
            std::cerr << "Could not populate first cloud" << std::endl;
        }

        //      Calculate AABB: for every point available in the flatbuffer file override

        if (!this->aabb.isInitialized) {
            std::cout << "Provided AABB is uninitialized - computing AABB" << std::endl;
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
            std::cout << "Total Number of Points in Cloud: " << totalNumPoints << std::endl;
        }

        currentFile = files.begin();
        reader = new ifstream(*currentFile, ios::in | ios::binary);
    }
    FlatBufferReader::~FlatBufferReader(){

          close();

      }

      void FlatBufferReader::close(){
          if(reader != NULL){
              reader->close();

              delete reader;
              reader = NULL;

          }
      }

    int64_t FlatBufferReader::numPoints(){
        return pointCount;
    }

    bool FlatBufferReader::prepareNextSegment()  {
        /** @brief This function is called every time to read 4 bytes of data from flatbuffer file, when the code reaches the end of segment
         *  @param[in] reader to read the 4 bytes
         *  @return bool
         */

        try{


            std::cout.precision(std::numeric_limits<double>::max_digits10);

            int numAdditionalPoints = 0;

            reader->read(reinterpret_cast<char *>(buffer), 4);
            if ((reader->eof())||(reader->bad())||(reader->peek() == 0)) {
                std::cerr << "Reader is at end of file (before preparing next segment)" << std::endl;
                return false;
            }

            auto numberOfBytes =
                            (uint32_t) buffer[3] << 24 |
                            (uint32_t) buffer[2] << 16 |
                            (uint32_t) buffer[1] << 8 |
                            (uint32_t) buffer[0];

//            std::cout << "Segment Size: " << numberOfBytes << std::endl;

            if (numberOfBytes==0) {
                std::cerr << "Reader is at end of file (after reading segment size)" << std::endl;
                return false;
            }
            readerBuffer.clear();
            readerBuffer.reserve(numberOfBytes);

            if ((reader->eof())||(reader->bad())||(reader->peek() == 0)) {
                std::cerr << "Reader is at end of file (after reading segment)" << std::endl;
                return false;
            }
            reader->read(&readerBuffer[0], numberOfBytes);
            //    For the flatbuffer file of type pointcloud points using the Schema -> DataSchemas/schemas/LIDARWORLD.fbs

            if (flatBufferFileType == "points")
            {
                auto pointcloud = Flatbuffer::LIDAR::GetPointCloud(&readerBuffer[0]);
                points          = pointcloud->points();
                pointsLength    = points->Length();
                numAdditionalPoints = pointsLength;
                totalPoints += numAdditionalPoints;
//                std::cout << "Prepared next segment with: " << pointsLength << " new points" << "\t[total: " << totalPoints << "]" << std::endl;
            }

            else if (flatBufferFileType == "classified-points")
            {
                classifiedPoints = flatbuffers::GetRoot<Flatbuffer::GroundTruth::Points>(&readerBuffer[0]);
                pointsLength = classifiedPoints->x()->size();
                numAdditionalPoints = pointsLength;
            }

                //    For the flatbuffer file of type track bounding box pointcloud using the Schema -> DataSchemas/schemas/GroundTruth.fbs

            else if (flatBufferFileType == "bbox")
            {
//
                auto track     = flatbuffers::GetRoot<Flatbuffer::GroundTruth::Track>(&readerBuffer[0]);
                statesFb       = track->states();
                statesLength   = statesFb->Length();
                totalNumPoints += 12*pointsLength;
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
                numAdditionalPoints = rightLaneLength;
            }

                //    For the flatbuffer file of type Detections pointcloud using the Schema -> DataSchemas/schemas/GroundTruth.fbs

            else if (flatBufferFileType == "detections")
            {
                auto Detection   = flatbuffers::GetRoot<Flatbuffer::GroundTruth::Detections>(&readerBuffer[0]);
                detectionCenter  = Detection->detections();
                detectionLength  = detectionCenter->Length();
                numAdditionalPoints = detectionLength;
            }

                //    For the flatbuffer file of type rtk pointcloud using the Schema -> DataSchemas/schemas/GroundTruth.fbs

            else if (flatBufferFileType == "rtk")
            {
                auto  rtk     = flatbuffers::GetRoot<Flatbuffer::GroundTruth::Poses>(&readerBuffer[0]);
                auto rtkPose  = rtk->poses();
                rtkLength     = rtkPose->Length();
                numAdditionalPoints = rtkLength;
            }
            else {
                std::cerr << "Unknown flatbuffer input type" << std::endl;
                return false;
            }
            if (numAdditionalPoints == 0) {
                std::cerr << "Reader is at end of file (No additional points after reading segment)" << std::endl;
                return false;
            }

//            std::cout << "Number of Additional Points: " << numAdditionalPoints << std::endl;
            totalNumPoints += numAdditionalPoints;
            return true;
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
        if (bboxPointsIdx == 0) {
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
            point.position.x = Points[bboxPointsIdx].bbox_x;
            point.position.y = Points[bboxPointsIdx].bbox_y;
            point.position.z = Points[bboxPointsIdx].bbox_z;
            point.gpsTime    = timeStamps ;
            bboxPointsIdx++;
            return true;

        }
        else{ if (bboxPointsIdx < Points.size()) {

                point.position.x = Points[bboxPointsIdx].bbox_x;
                point.position.y = Points[bboxPointsIdx].bbox_y;
                point.position.z = Points[bboxPointsIdx].bbox_z;
                point.gpsTime    = timeStamps;
                bboxPointsIdx++;
                return true;

            }
            else {
                Points.clear();
                bboxPointsIdx = 0;
                prepareNextSegment();
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
                centroid = (rearPoint + frontPoint) / 2;

                //Subtracting the centroid
                auto centroidRearPoint = rearPoint - centroid;
                auto centroidFrontPoint = frontPoint - centroid;

                rtk = rtk - centroid;

                //applying rotations
                auto rearRotationMatrix = getTxMat(centroidRearPoint, Yaw);
                auto rotatedFrontPoint = (rearRotationMatrix * centroid);
                auto frontRotationMatrix = getTxMat(centroidFrontPoint, Yaw);
                auto rotatedRearPoint = (frontRotationMatrix * centroid);


                auto rtkRotationMatrix = getTxMat(rtk, Yaw);
                auto rotatedRtk = (rtkRotationMatrix * centroid);
                auto centroidRotationMatrix = getTxMat(centroid, Yaw);
                auto rotatedCenteroid = (centroidRotationMatrix * centroid);

                //subtract rtk

                auto rtkSubtractedCenteroid = rotatedCenteroid - rotatedRtk;
                auto rtkSubtracted = rotatedRtk - rotatedRtk;
                auto rtkSubtractedFrontPoint = rotatedFrontPoint - rotatedRtk; //change here karthik
                auto rtkSubtractedRearPoint = rotatedRearPoint - rotatedRtk;


                ego.push_back({rtkSubtractedCenteroid(0) + rtkPoints->pos()->x(),
                               rtkPoints->pos()->y() + rtkSubtractedCenteroid(1),
                               rtkPoints->pos()->z() + rtkSubtractedCenteroid(2), rtkPoints->timestamp()});
                ego.push_back({rtkSubtractedFrontPoint(0) + rtkPoints->pos()->x(),
                               rtkPoints->pos()->y() + rtkSubtractedFrontPoint(1),
                               rtkPoints->pos()->z() + rtkSubtractedFrontPoint(2), rtkPoints->timestamp()});
                ego.push_back({rtkSubtractedRearPoint(0) + rtkPoints->pos()->x(),
                               rtkPoints->pos()->y() + rtkSubtractedRearPoint(1),
                               rtkPoints->pos()->z() + rtkSubtractedRearPoint(2), rtkPoints->timestamp()});
                ego.push_back(
                        {rtkSubtracted(0) + rtkPoints->pos()->x(), rtkPoints->pos()->y() + rtkSubtracted(1),
                         rtkPoints->pos()->z() + rtkSubtracted(2),
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


        bool hasPoints = reader->good();
        if(hasPoints ) {
            if (flatBufferFileType == "points" ) {
                // check if the end of 4 bytes segment reached

                if (pointsIdx < pointsLength) {
                    auto fbPoints    = points->Get(pointsIdx);
                    pointsIdx++;
                    point.position.x = fbPoints->x();
                    point.position.y = fbPoints->y();
                    point.position.z = fbPoints->z();
                    point.gpsTime    = fbPoints->timestamp();
                    point.intensity  = fbPoints->intensity();
                    point.rtk_pose.x = fbPoints->rtkPose().x();
                    point.rtk_pose.y = fbPoints->rtkPose().y();
                    point.rtk_pose.z = fbPoints->rtkPose().z();
                    point.rtk_orient.x = fbPoints->rtkPose().roll();
                    point.rtk_orient.y = fbPoints->rtkPose().pitch();
                    point.rtk_orient.z = fbPoints->rtkPose().yaw();
                    point.dualPlusConfidence = fbPoints->dualPlusConfidence();
                    auto latlon = VTMToLatLon(std::make_tuple(point.position.x, point.position.y, point.position.z),
                                              std::make_tuple(vtmMetadata.VTMoriginLatitude, vtmMetadata.VTMoriginLongitude),
                                              vtmMetadata.scaleFactor);
                    point.latitude = std::get<0>(latlon);
                    point.longitude = std::get<1>(latlon);
                    point.altitude = point.position.z;
                    return true;
                }
                    //if end of 4 bytes reached, then read the next 4 bytes.

                else if (pointsIdx == pointsLength) {
                    pointsIdx = 0;
                    if (prepareNextSegment()) {
                        auto fbPoints    = points->Get(pointsIdx);
                        pointsIdx++;
                        point.position.x = fbPoints->x();
                        point.position.y = fbPoints->y();
                        point.position.z = fbPoints->z();
                        point.gpsTime    = fbPoints->timestamp();
                        point.intensity  = fbPoints->intensity();
                        point.rtk_pose.x = fbPoints->rtkPose().x();
                        point.rtk_pose.y = fbPoints->rtkPose().y();
                        point.rtk_pose.z = fbPoints->rtkPose().z();
                        point.rtk_orient.x = fbPoints->rtkPose().roll();
                        point.rtk_orient.y = fbPoints->rtkPose().pitch();
                        point.rtk_orient.z = fbPoints->rtkPose().yaw();
                        point.dualPlusConfidence = fbPoints->dualPlusConfidence();
                        auto latlon = VTMToLatLon(std::make_tuple(point.position.x, point.position.y, point.position.z),
                                                  std::make_tuple(vtmMetadata.VTMoriginLatitude, vtmMetadata.VTMoriginLongitude),
                                                  vtmMetadata.scaleFactor);
                        point.latitude = std::get<0>(latlon);
                        point.longitude = std::get<1>(latlon);
                        point.altitude = point.position.z;
                        return true;
                    } else {

                        std::cout << "There are no More Pointcloud Points Left in the file" << std::endl;
                        return false;
                    }

                } else {std::cerr << "Why am I here???" << std::endl;}
            }
            else if (flatBufferFileType == "classified-points") {

                if (pointsIdx < pointsLength) {
                    point.position.x = classifiedPoints->x()->Get(pointsIdx);
                    point.position.y = classifiedPoints->y()->Get(pointsIdx);
                    point.position.z = classifiedPoints->z()->Get(pointsIdx);
                    point.intensity  = classifiedPoints->intensity()->Get(pointsIdx);
                    point.gpsTime = classifiedPoints->timestamp()->Get(pointsIdx);
                    point.classification = classifiedPoints->ptType()->Get(pointsIdx);
                    point.dualPlusConfidence = classifiedPoints->dualPlusConfidence()->Get(pointsIdx);
                    auto latlon = VTMToLatLon(std::make_tuple(point.position.x, point.position.y, point.position.z),
                                              std::make_tuple(vtmMetadata.VTMoriginLatitude, vtmMetadata.VTMoriginLongitude),
                                              vtmMetadata.scaleFactor);
                    point.latitude = std::get<0>(latlon);
                    point.longitude = std::get<1>(latlon);
                    point.altitude = point.position.z;

                    const auto rtkPoses = classifiedPoints->rtkPoses()->Get(pointsIdx);
                    point.rtk_pose.x = rtkPoses->x();
                    point.rtk_pose.y = rtkPoses->y();
                    point.rtk_pose.z = rtkPoses->z();
                    point.rtk_orient.x = rtkPoses->roll();
                    point.rtk_orient.y = rtkPoses->pitch();
                    point.rtk_orient.z = rtkPoses->yaw();

                    pointsIdx++;
                    return true;
                }
                    //if end of 4 bytes reached, then read the next 4 bytes.

                else if (pointsIdx == pointsLength) {
                    pointsIdx = 0;
                    if (prepareNextSegment()) {
                        point.position.x = classifiedPoints->x()->Get(pointsIdx);
                        point.position.y = classifiedPoints->y()->Get(pointsIdx);
                        point.position.z = classifiedPoints->z()->Get(pointsIdx);
                        point.intensity  = classifiedPoints->intensity()->Get(pointsIdx);
                        point.gpsTime = classifiedPoints->timestamp()->Get(pointsIdx);
                        point.classification = classifiedPoints->ptType()->Get(pointsIdx);
                        point.dualPlusConfidence = classifiedPoints->dualPlusConfidence()->Get(pointsIdx);
                        auto latlon = VTMToLatLon(std::make_tuple(point.position.x, point.position.y, point.position.z),
                                                  std::make_tuple(vtmMetadata.VTMoriginLatitude, vtmMetadata.VTMoriginLongitude),
                                                  vtmMetadata.scaleFactor);
                        point.latitude = std::get<0>(latlon);
                        point.longitude = std::get<1>(latlon);
                        point.altitude = point.position.z;

                        const auto rtkPoses = classifiedPoints->rtkPoses()->Get(pointsIdx);
                        point.rtk_pose.x = rtkPoses->x();
                        point.rtk_pose.y = rtkPoses->y();
                        point.rtk_pose.z = rtkPoses->z();
                        point.rtk_orient.x = rtkPoses->roll();
                        point.rtk_orient.y = rtkPoses->pitch();
                        point.rtk_orient.z = rtkPoses->yaw();

                        pointsIdx++;
                        return true;
                    }
                    else if (!prepareNextSegment()) {

                        std::cout << "There are no more Classified Points left in the file" << std::endl;
                        return false;
                    }

                }



            }
            else if (flatBufferFileType == "bbox" ) {

                centroid();

            }
            else if (flatBufferFileType == "lanes") {

                if (laneIdx == 0) {
                    lanePoints();

                }

                if (laneIdx < LanePoints.size()) {

                    point.position.x = LanePoints[laneIdx].lane_x;
                    point.position.y = LanePoints[laneIdx].lane_y;
                    point.position.z = LanePoints[laneIdx].lane_z;
                    point.gpsTime    = LanePoints[laneIdx].lane_gps;
                    laneIdx++;
                    return true;
                }
                else if (laneIdx == LanePoints.size()) {
                    laneIdx = 0;
                    LanePoints.clear();
                    if (prepareNextSegment()) {
                        lanePoints();

                        point.position.x = LanePoints[laneIdx].lane_x;
                        point.position.y = LanePoints[laneIdx].lane_y;
                        point.position.z = LanePoints[laneIdx].lane_z;
                        point.gpsTime    = LanePoints[laneIdx].lane_gps;
                        laneIdx++;
                        return true;
                    }
                    else {
                        std::cout << "There are no More Lane Points Left in the file" << std::endl;
                        return false;
                    }
                }
            }
            else if (flatBufferFileType == "detections" ) {

                if (detectionIdx < detectionLength) {
                    auto detectionPoints = detectionCenter->Get(detectionIdx);

                    point.position.x = detectionPoints->centroid()->x();
                    point.position.y = detectionPoints->centroid()->y();
                    point.position.z = detectionPoints->centroid()->z();
                    point.gpsTime    = detectionPoints->timestamp();
                    detectionIdx++;
                    return true;
                }
                    //if end of 4 bytes reached, then read the next 4 bytes.

                else if (detectionIdx == detectionLength) {
                    detectionIdx = 0;
                    if (prepareNextSegment()) {
                        auto detectionPoints = detectionCenter->Get(detectionIdx);

                        point.position.x = detectionPoints->centroid()->x();
                        point.position.y = detectionPoints->centroid()->y();
                        point.position.z = detectionPoints->centroid()->z();
                        point.gpsTime    = detectionPoints->timestamp();
                        detectionIdx++;
                        return true;
                    }
                }
            }
            else if (flatBufferFileType == "rtk" ) {

                // check if the end of 4 bytes segment reached
                if (rtkIdx < rtkLength && rtkIdx==0) {
                    //egoDimensions();                             TODO Should be Edited when we have the dimensions of the vehicle provided by the LG.
                    point.position.x = ego[rtkIdx].ego_x;
                    point.position.y = ego[rtkIdx].ego_y;
                    point.position.z = ego[rtkIdx].ego_z;
                    point.gpsTime    = ego[rtkIdx].ego_time;
                    rtkIdx++;
                    return true;
                }
                    //if end of 4 bytes reached, then read the next 4 bytes.
                else if (rtkIdx < ego.size()) {
                    point.position.x = ego[rtkIdx].ego_x;
                    point.position.y = ego[rtkIdx].ego_y;
                    point.position.z = ego[rtkIdx].ego_z;
                    point.gpsTime    = ego[rtkIdx].ego_time;
                    rtkIdx++;
                    return true;
                }
                else if (rtkIdx == ego.size()) {
                    rtkIdx = 0;
                    if (prepareNextSegment()) {
//                        egoDimensions();
                        point.position.x = ego[rtkIdx].ego_x;
                        point.position.y = ego[rtkIdx].ego_y;
                        point.position.z = ego[rtkIdx].ego_z;
                        point.gpsTime    = ego[rtkIdx].ego_time;
                        return true;
                    }
                    else {

                        std::cout << "There are no More Pointcloud Points Left in the file" << std::endl;
                        return false;
                    }
                }
            }
            else {
                std::cerr << "Unknown flatbuffer type specified: " << flatBufferFileType << std::endl;
                exit(1);
            }
        }
        else  {
            currentFile++;

            if (currentFile != files.end()) {
              reader->close();
                  delete reader;
                  reader = NULL;
                  reader = new ifstream(*currentFile, ios::in | ios::binary);
                  hasPoints = reader->good();
            }
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
