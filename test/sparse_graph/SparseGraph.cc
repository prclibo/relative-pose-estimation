#include "SparseGraph.h"

#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <sstream>

namespace vcharge
{

Pose::Pose()
{
    m_q.setIdentity();
    m_t.setZero();
}

Eigen::Quaterniond&
Pose::rotation(void)
{
    return m_q;
}

const Eigen::Quaterniond&
Pose::rotation(void) const
{
    return m_q;
}

double*
Pose::rotationData(void)
{
    return m_q.coeffs().data();
}

const double* const
Pose::rotationData(void) const
{
    return m_q.coeffs().data();
}

Eigen::Vector3d&
Pose::translation(void)
{
    return m_t;
}

const Eigen::Vector3d&
Pose::translation(void) const
{
    return m_t;
}

double*
Pose::translationData(void)
{
    return m_t.data();
}

const double* const
Pose::translationData(void) const
{
    return m_t.data();
}

Eigen::Matrix4d
Pose::pose(void) const
{
    Eigen::Matrix4d H;
    H.setIdentity();
    H.block<3,3>(0,0) = m_q.toRotationMatrix();
    H.block<3,1>(0,3) = m_t;

    return H;
}

Frame::Frame()
 : mId(0)
{

}

PosePtr&
Frame::camera(void)
{
    return mCamera;
}

PoseConstPtr
Frame::camera(void) const
{
    return mCamera;
}

OdometerPtr&
Frame::odometer(void)
{
    return mOdometer;
}

OdometerConstPtr
Frame::odometer(void) const
{
    return mOdometer;
}

std::vector<Point2DFeaturePtr>&
Frame::features2D(void)
{
    return mFeatures2D;
}

const std::vector<Point2DFeaturePtr>&
Frame::features2D(void) const
{
    return mFeatures2D;
}

std::vector<Point3DFeaturePtr>&
Frame::features3D(void)
{
    return mFeatures3D;
}

const std::vector<Point3DFeaturePtr>&
Frame::features3D(void) const
{
    return mFeatures3D;
}

unsigned int&
Frame::id(void)
{
    return mId;
}

unsigned int
Frame::id(void) const
{
    return mId;
}

cv::Mat&
Frame::image(void)
{
    return mImage;
}

const cv::Mat&
Frame::image(void) const
{
    return mImage;
}

Point2DFeature::Point2DFeature()
 : mIndex(0)
 , mBestPrevMatchIdx(-1)
 , mBestNextMatchIdx(-1)
{

}

cv::Mat&
Point2DFeature::descriptor(void)
{
    return mDtor;
}

const cv::Mat&
Point2DFeature::descriptor(void) const
{
    return mDtor;
}

cv::KeyPoint&
Point2DFeature::keypoint(void)
{
    return mKeypoint;
}

const cv::KeyPoint&
Point2DFeature::keypoint(void) const
{
    return mKeypoint;
}

unsigned int&
Point2DFeature::index(void)
{
    return mIndex;
}

unsigned int
Point2DFeature::index(void) const
{
    return mIndex;
}

Point2DFeaturePtr&
Point2DFeature::prevMatch(void)
{
    return mPrevMatches.at(mBestPrevMatchIdx);
}

Point2DFeatureConstPtr
Point2DFeature::prevMatch(void) const
{
    return mPrevMatches.at(mBestPrevMatchIdx);
}

std::vector<Point2DFeaturePtr>&
Point2DFeature::prevMatches(void)
{
    return mPrevMatches;
}

const std::vector<Point2DFeaturePtr>&
Point2DFeature::prevMatches(void) const
{
    return mPrevMatches;
}

int&
Point2DFeature::bestPrevMatchIdx(void)
{
    return mBestPrevMatchIdx;
}

int
Point2DFeature::bestPrevMatchIdx(void) const
{
    return mBestPrevMatchIdx;
}

Point2DFeaturePtr&
Point2DFeature::nextMatch(void)
{
    return mNextMatches.at(mBestNextMatchIdx);
}

Point2DFeatureConstPtr
Point2DFeature::nextMatch(void) const
{
    return mNextMatches.at(mBestNextMatchIdx);
}

std::vector<Point2DFeaturePtr>&
Point2DFeature::nextMatches(void)
{
    return mNextMatches;
}

const std::vector<Point2DFeaturePtr>&
Point2DFeature::nextMatches(void) const
{
    return mNextMatches;
}

int&
Point2DFeature::bestNextMatchIdx(void)
{
    return mBestNextMatchIdx;
}

int
Point2DFeature::bestNextMatchIdx(void) const
{
    return mBestNextMatchIdx;
}

Point3DFeaturePtr&
Point2DFeature::feature3D(void)
{
    return mFeature3D;
}

Point3DFeatureConstPtr
Point2DFeature::feature3D(void) const
{
    return mFeature3D;
}

FramePtr&
Point2DFeature::frame(void)
{
    return mFrame;
}

FrameConstPtr
Point2DFeature::frame(void) const
{
    return mFrame;
}

Point2DFeatureRightPtr&
Point2DFeatureLeft::rightCorrespondence(void)
{
    return mRightCorrespondence;
}

Point2DFeatureRightConstPtr
Point2DFeatureLeft::rightCorrespondence(void) const
{
    return mRightCorrespondence;
}

Point2DFeatureLeftPtr&
Point2DFeatureLeft::prevCorrespondence(void)
{
    return mPrevCorrespondence;
}

Point2DFeatureLeftConstPtr
Point2DFeatureLeft::prevCorrespondence(void) const
{
    return mPrevCorrespondence;
}

Point2DFeatureLeftPtr&
Point2DFeatureRight::leftCorrespondence(void)
{
    return mLeftCorrespondence;
}

Point2DFeatureLeftConstPtr
Point2DFeatureRight::leftCorrespondence(void) const
{
    return mLeftCorrespondence;
}

Point2DFeatureRightPtr&
Point2DFeatureRight::prevCorrespondence(void)
{
    return mPrevCorrespondence;
}

Point2DFeatureRightConstPtr
Point2DFeatureRight::prevCorrespondence(void) const
{
    return mPrevCorrespondence;
}

Point3DFeature::Point3DFeature(void)
{
    mPoint.setZero();
}

Eigen::Vector3d&
Point3DFeature::point(void)
{
    return mPoint;
}

const Eigen::Vector3d&
Point3DFeature::point(void) const
{
    return mPoint;
}

double*
Point3DFeature::pointData(void)
{
    return mPoint.data();
}

const double* const
Point3DFeature::pointData(void) const
{
    return mPoint.data();
}

std::vector<Point2DFeaturePtr>&
Point3DFeature::features2D(void)
{
    return mFeatures2D;
}

const std::vector<Point2DFeaturePtr>&
Point3DFeature::features2D(void) const
{
    return mFeatures2D;
}

SparseGraph::SparseGraph()
{

}

std::vector<FrameSegment>&
SparseGraph::frameSegments(int cameraIdx)
{
    if (cameraIdx >= mFrameSegments.size())
    {
        mFrameSegments.resize(cameraIdx + 1);
    }

    return mFrameSegments.at(cameraIdx);
}

const std::vector<FrameSegment>&
SparseGraph::frameSegments(int cameraIdx) const
{
    return mFrameSegments.at(cameraIdx);
}

bool
SparseGraph::readFromFile(const std::string& filename)
{
    boost::filesystem::path xmlPath(filename);

    boost::filesystem::path rootDir;
    if (xmlPath.has_parent_path())
    {
        rootDir = xmlPath.parent_path();
    }
    else
    {
        rootDir = boost::filesystem::path(".");
    }

    mFrameSegments.clear();

    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(filename.c_str()) != tinyxml2::XML_NO_ERROR)
    {
        std::cout << "# WARNING: Unable to open/parse " << filename << "." << std::endl;
        return false;
    }

    // parse xml file

    // odometer and 3D feature structures are in global scope as
    // multiple frames from multiple cameras reference these structures
    unsigned int nCameras;
    doc.RootElement()->QueryUnsignedAttribute("cameras_size", &nCameras);
    mFrameSegments.resize(nCameras);

    FrameSegment frameMap;
    std::vector<tinyxml2::XMLElement*> frameXMLMap;
    std::vector<PosePtr> poseMap;
    std::vector<OdometerPtr> odometerMap;
    std::vector<Point2DFeaturePtr> feature2DMap;
    std::vector<tinyxml2::XMLElement*> feature2DXMLMap;
    std::vector<Point3DFeaturePtr> feature3DMap;
    std::vector<tinyxml2::XMLElement*> feature3DXMLMap;

    tinyxml2::XMLElement* eFrames = doc.RootElement()->FirstChildElement("frames");
    tinyxml2::XMLElement* ePoses = doc.RootElement()->FirstChildElement("poses");
    tinyxml2::XMLElement* eOdometers = doc.RootElement()->FirstChildElement("odometers");
    tinyxml2::XMLElement* eFeatures2D = doc.RootElement()->FirstChildElement("features2D");
    tinyxml2::XMLElement* eFeatures3D = doc.RootElement()->FirstChildElement("features3D");

    unsigned int nFrames;
    eFrames->QueryUnsignedAttribute("size", &nFrames);
    XMLToFrames(eFrames, nFrames, frameMap, frameXMLMap, rootDir);

    unsigned int nPoses;
    ePoses->QueryUnsignedAttribute("size", &nPoses);
    XMLToPoses(ePoses, nPoses, poseMap);

    unsigned int nOdometers;
    eOdometers->QueryUnsignedAttribute("size", &nOdometers);
    XMLToOdometers(eOdometers, nOdometers, odometerMap);

    unsigned int nFeatures2D;
    eFeatures2D->QueryUnsignedAttribute("size", &nFeatures2D);
    XMLToPoint2DFeatures(eFeatures2D, nFeatures2D, feature2DMap, feature2DXMLMap);

    unsigned int nFeatures3D;
    eFeatures3D->QueryUnsignedAttribute("size", &nFeatures3D);
    XMLToPoint3DFeatures(eFeatures3D, nFeatures3D, feature3DMap, feature3DXMLMap);

    for (size_t cameraIdx = 0; cameraIdx < mFrameSegments.size(); ++cameraIdx)
    {
        char cameraName[255];
        sprintf(cameraName, "camera%lu", cameraIdx);
        tinyxml2::XMLElement* eCamera = doc.RootElement()->FirstChildElement(cameraName);

        unsigned int nSegments;
        eCamera->QueryUnsignedAttribute("segments_size", &nSegments);
        mFrameSegments.at(cameraIdx).resize(nSegments);

        for (size_t segmentIdx = 0; segmentIdx < mFrameSegments.at(cameraIdx).size(); ++segmentIdx)
        {
            char segmentName[255];
            sprintf(segmentName, "segment%lu", segmentIdx);
            tinyxml2::XMLElement* eSegment = eCamera->FirstChildElement(segmentName);

            unsigned int nFramesSegment;
            eSegment->QueryUnsignedAttribute("frames_size", &nFramesSegment);

            FrameSegment& segment = mFrameSegments.at(cameraIdx).at(segmentIdx);
            segment.resize(nFramesSegment);

            const tinyxml2::XMLAttribute* aFramesSegment = eSegment->FirstChildElement("frames")->FirstAttribute();
            for (size_t i = 0; i < segment.size(); ++i)
            {
                unsigned int key;
                sscanf(aFramesSegment->Name(), "frame%u", &key);

                unsigned int value;
                sscanf(aFramesSegment->Value(), "frame%u", &value);

                segment.at(key) = frameMap.at(value);

                aFramesSegment = aFramesSegment->Next();
            }
        }
    }

    // link all references
    for (size_t i = 0; i < frameXMLMap.size(); ++i)
    {
        FramePtr& frame = frameMap.at(i);
        tinyxml2::XMLElement* eFrame = frameXMLMap.at(i);

        const char* cameraName = eFrame->Attribute("camera");
        if (cameraName != 0)
        {
            size_t poseIdx;
            sscanf(cameraName, "pose%lu", &poseIdx);

            frame->camera() = poseMap.at(poseIdx);
        }

        const char* odometerName = eFrame->Attribute("odometer");
        if (odometerName != 0)
        {
            size_t odometerIdx;
            sscanf(odometerName, "odometer%lu", &odometerIdx);

            frame->odometer() = odometerMap.at(odometerIdx);
        }

        unsigned int nFeatures2D;
        eFrame->QueryUnsignedAttribute("features2D_size", &nFeatures2D);
        std::vector<Point2DFeaturePtr>& features2D = frame->features2D();
        features2D.resize(nFeatures2D);

        const tinyxml2::XMLAttribute* aFeatures2D = eFrame->FirstChildElement("features2D")->FirstAttribute();
        for (size_t i = 0; i < features2D.size(); ++i)
        {
            unsigned int key;
            sscanf(aFeatures2D->Name(), "features2D_%u", &key);

            unsigned int value;
            sscanf(aFeatures2D->Value(), "F2D-%u", &value);

            features2D.at(key) = feature2DMap.at(value);

            aFeatures2D = aFeatures2D->Next();
        }

        unsigned int nFeatures3D;
        eFrame->QueryUnsignedAttribute("features3D_size", &nFeatures3D);
        std::vector<Point3DFeaturePtr>& features3D = frame->features3D();
        features3D.resize(nFeatures3D);

        const tinyxml2::XMLAttribute* aFeatures3D = eFrame->FirstChildElement("features3D")->FirstAttribute();
        for (size_t i = 0; i < features3D.size(); ++i)
        {
            unsigned int key;
            sscanf(aFeatures3D->Name(), "features3D_%u", &key);

            unsigned int value;
            sscanf(aFeatures3D->Value(), "F3D-%u", &value);

            features3D.at(key) = feature3DMap.at(value);

            aFeatures3D = aFeatures3D->Next();
        }
    }

    for (size_t i = 0; i < feature2DXMLMap.size(); ++i)
    {
        Point2DFeaturePtr& feature2D = feature2DMap.at(i);
        tinyxml2::XMLElement* eFeature2D = feature2DXMLMap.at(i);

        unsigned int nPrevMatches;
        eFeature2D->QueryUnsignedAttribute("prev_matches_size", &nPrevMatches);
        feature2D->prevMatches().resize(nPrevMatches);

        const tinyxml2::XMLAttribute* aFeature2DPrevMatches = eFeature2D->FirstChildElement("prev_matches")->FirstAttribute();
        for (size_t j = 0; j < feature2D->prevMatches().size(); ++j)
        {
            unsigned int key;
            sscanf(aFeature2DPrevMatches->Name(), "prev_matches_%u", &key);

            unsigned int value;
            sscanf(aFeature2DPrevMatches->Value(), "F2D-%u", &value);

            feature2D->prevMatches().at(key) = feature2DMap.at(value);

            aFeature2DPrevMatches = aFeature2DPrevMatches->Next();
        }

        unsigned int nNextMatches;
        eFeature2D->QueryUnsignedAttribute("next_matches_size", &nNextMatches);
        feature2D->nextMatches().resize(nNextMatches);

        const tinyxml2::XMLAttribute* aFeature2DNextMatches = eFeature2D->FirstChildElement("next_matches")->FirstAttribute();
        for (size_t j = 0; j < feature2D->nextMatches().size(); ++j)
        {
            unsigned int key;
            sscanf(aFeature2DNextMatches->Name(), "next_matches_%u", &key);

            unsigned int value;
            sscanf(aFeature2DNextMatches->Value(), "F2D-%u", &value);

            feature2D->nextMatches().at(key) = feature2DMap.at(value);

            aFeature2DPrevMatches = aFeature2DNextMatches->Next();
        }

        const char* feature3DName = eFeature2D->Attribute("feature3D");
        if (feature3DName != 0)
        {
            size_t feature3DIdx;
            sscanf(feature3DName, "F3D-%lu", &feature3DIdx);

            feature2D->feature3D() = feature3DMap.at(feature3DIdx);
        }

        size_t frameIdx;
        sscanf(eFeature2D->Attribute("frame"), "frame%lu", &frameIdx);
        feature2D->frame() = frameMap.at(frameIdx);
    }

    for (size_t i = 0; i < feature3DXMLMap.size(); ++i)
    {
        Point3DFeaturePtr& feature3D = feature3DMap.at(i);
        tinyxml2::XMLElement* eFeature3D = feature3DXMLMap.at(i);

        unsigned int nFeatures2D;
        eFeature3D->QueryUnsignedAttribute("features2D_size", &nFeatures2D);
        feature3D->features2D().resize(nFeatures2D);

        const tinyxml2::XMLAttribute* aFeatures2DCorr = eFeature3D->FirstChildElement("features2D")->FirstAttribute();
        for (size_t j = 0; j < feature3D->features2D().size(); ++j)
        {
            unsigned int key;
            sscanf(aFeatures2DCorr->Name(), "features2D_%u", &key);

            unsigned int value;
            sscanf(aFeatures2DCorr->Value(), "F2D-%u", &value);

            char keyName[255];
            sprintf(keyName, "features2D_%lu", j);

            feature3D->features2D().at(key) = feature2DMap.at(value);

            aFeatures2DCorr = aFeatures2DCorr->Next();
        }
    }

    return true;
}

void
SparseGraph::writeToFile(const std::string& filename) const
{
    boost::filesystem::path xmlPath(filename);

    boost::filesystem::path imageDir;
    if (xmlPath.has_parent_path())
    {
        imageDir = xmlPath.parent_path();
        imageDir /= "images";
    }
    else
    {
        imageDir = boost::filesystem::path("images");
    }

    // create image directory if it does not exist
    if (!boost::filesystem::exists(imageDir))
    {
        boost::filesystem::create_directory(imageDir);
    }

    // write frame data to xml file
    tinyxml2::XMLDocument doc;

    tinyxml2::XMLElement* eRoot = doc.NewElement("root");
    doc.InsertEndChild(eRoot);
    eRoot->SetAttribute("cameras_size", static_cast<unsigned int>(mFrameSegments.size()));

    boost::unordered_map<Frame*,size_t> frameMap;
    boost::unordered_map<Frame*,tinyxml2::XMLElement*> frameXMLMap;
    boost::unordered_map<const Pose*,size_t> poseMap;
    boost::unordered_map<const Odometer*,size_t> odometerMap;
    boost::unordered_map<const Point2DFeature*,size_t> feature2DMap;
    boost::unordered_map<const Point2DFeature*,tinyxml2::XMLElement*> feature2DXMLMap;
    boost::unordered_map<const Point3DFeature*,size_t> feature3DMap;
    boost::unordered_map<const Point3DFeature*,tinyxml2::XMLElement*> feature3DXMLMap;

    tinyxml2::XMLElement* eFrames = doc.NewElement("frames");
    eRoot->InsertEndChild(eFrames);

    tinyxml2::XMLElement* ePoses = doc.NewElement("poses");
    eRoot->InsertEndChild(ePoses);

    tinyxml2::XMLElement* eOdometers = doc.NewElement("odometers");
    eRoot->InsertEndChild(eOdometers);

    tinyxml2::XMLElement* eFeatures2D = doc.NewElement("features2D");
    eRoot->InsertEndChild(eFeatures2D);

    tinyxml2::XMLElement* eFeatures3D = doc.NewElement("features3D");
    eRoot->InsertEndChild(eFeatures3D);

    for (size_t cameraIdx = 0; cameraIdx < mFrameSegments.size(); ++cameraIdx)
    {
        char cameraName[255];
        sprintf(cameraName, "camera%lu", cameraIdx);
        tinyxml2::XMLElement* eCamera = doc.NewElement(cameraName);
        eRoot->InsertEndChild(eCamera);

        eCamera->SetAttribute("segments_size", static_cast<unsigned int>(mFrameSegments.at(cameraIdx).size()));
        for (size_t segmentIdx = 0; segmentIdx < mFrameSegments.at(cameraIdx).size(); ++segmentIdx)
        {
            const FrameSegment& segment = mFrameSegments.at(cameraIdx).at(segmentIdx);

            char segmentName[255];
            sprintf(segmentName, "segment%lu", segmentIdx);
            tinyxml2::XMLElement* eSegment = doc.NewElement(segmentName);
            eCamera->InsertEndChild(eSegment);

            eSegment->SetAttribute("frames_size", static_cast<unsigned int>(segment.size()));

            tinyxml2::XMLElement* eFramesSegment = doc.NewElement("frames");
            eSegment->InsertEndChild(eFramesSegment);

            // index all structures
            for (size_t frameIdx = 0; frameIdx < segment.size(); ++frameIdx)
            {
                FramePtr frame = segment.at(frameIdx);

                tinyxml2::XMLElement* eFrame = frameToXML(frame, doc, eFrames, frameMap, frameXMLMap, imageDir);

                char key[255];
                sprintf(key, "frame%lu", frameIdx);

                eFramesSegment->SetAttribute(key, eFrame->Name());

                if (!frame->camera().empty())
                {
                    poseToXML(frame->camera(), doc, ePoses, poseMap);
                }
                if (!frame->odometer().empty())
                {
                    odometerToXML(frame->odometer(), doc, eOdometers, odometerMap);
                }

                std::vector<Point2DFeaturePtr>& features2D = frame->features2D();
                for (size_t i = 0; i < features2D.size(); ++i)
                {
                    Point2DFeaturePtr& feature2D = features2D.at(i);
                    if (feature2D.empty())
                    {
                        std::cout << "# WARNING: Empty Point2DFeaturePtr instance." << std::endl;
                        continue;
                    }

                    point2DFeatureToXML(feature2D, doc, eFeatures2D, feature2DMap, feature2DXMLMap);
                }

                std::vector<Point3DFeaturePtr>& features3D = frame->features3D();
                for (size_t i = 0; i < features3D.size(); ++i)
                {
                    Point3DFeaturePtr& feature3D = features3D.at(i);
                    if (feature3D.empty())
                    {
                        std::cout << "# WARNING: Empty Point3DFeaturePtr instance." << std::endl;
                        continue;
                    }

                    point3DFeatureToXML(feature3D, doc, eFeatures3D, feature3DMap, feature3DXMLMap);
                }
            }
        }
    }

    // link all references
    for (boost::unordered_map<Frame*,tinyxml2::XMLElement*>::iterator it = frameXMLMap.begin();
             it != frameXMLMap.end(); ++it)
    {
        Frame* frame = it->first;
        tinyxml2::XMLElement* eFrame = it->second;

        if (!frame->camera().empty())
        {
            char poseName[255];
            sprintf(poseName, "pose%lu", poseMap[frame->camera()]);
            eFrame->SetAttribute("camera", poseName);
        }

        if (!frame->odometer().empty())
        {
            char odometerName[255];
            sprintf(odometerName, "odometer%lu", odometerMap[frame->odometer()]);
            eFrame->SetAttribute("odometer", odometerName);
        }

        eFrame->SetAttribute("features2D_size", static_cast<unsigned int>(frame->features2D().size()));

        tinyxml2::XMLElement* eFeatures2DFrame = doc.NewElement("features2D");
        eFrame->InsertEndChild(eFeatures2DFrame);

        std::vector<Point2DFeaturePtr>& features2D = frame->features2D();
        for (size_t i = 0; i < features2D.size(); ++i)
        {
            Point2DFeaturePtr& feature2D = features2D.at(i);

            char feature2DName[255];
            sprintf(feature2DName, "F2D-%lu", feature2DMap[feature2D]);

            char keyName[255];
            sprintf(keyName, "features2D_%lu", i);
            eFeatures2DFrame->SetAttribute(keyName, feature2DName);
        }

        eFrame->SetAttribute("features3D_size", static_cast<unsigned int>(frame->features3D().size()));

        tinyxml2::XMLElement* eFeatures3DFrame = doc.NewElement("features3D");
        eFrame->InsertEndChild(eFeatures3DFrame);

        std::vector<Point3DFeaturePtr>& features3D = frame->features3D();
        for (size_t i = 0; i < features3D.size(); ++i)
        {
            Point3DFeaturePtr& feature3D = features3D.at(i);

            char feature3DName[255];
            sprintf(feature3DName, "F3D-%lu", feature3DMap[feature3D]);

            char keyName[255];
            sprintf(keyName, "features3D_%lu", i);
            eFeatures3DFrame->SetAttribute(keyName, feature3DName);
        }
    }

    for (boost::unordered_map<const Point2DFeature*,tinyxml2::XMLElement*>::iterator it = feature2DXMLMap.begin();
             it != feature2DXMLMap.end(); ++it)
    {
        const Point2DFeature* feature2D = it->first;
        tinyxml2::XMLElement* eFeature2D = it->second;

        eFeature2D->SetAttribute("prev_matches_size", static_cast<unsigned int>(feature2D->prevMatches().size()));

        tinyxml2::XMLElement* eFeature2DPrevMatches = doc.NewElement("prev_matches");
        eFeature2D->InsertEndChild(eFeature2DPrevMatches);

        for (size_t i = 0; i < feature2D->prevMatches().size(); ++i)
        {
            char keyName[255];
            sprintf(keyName, "prev_matches_%lu", i);

            if (feature2D->prevMatches().at(i).empty())
            {
                std::cout << "# WARNING: Empty Point2DFeaturePtr instance." << std::endl;
            }

            char feature2DName[255];
            sprintf(feature2DName, "F2D-%lu", feature2DMap[feature2D->prevMatches().at(i)]);
            eFeature2DPrevMatches->SetAttribute(keyName, feature2DName);
        }

        eFeature2D->SetAttribute("next_matches_size", static_cast<unsigned int>(feature2D->nextMatches().size()));

        tinyxml2::XMLElement* eFeature2DNextMatches = doc.NewElement("next_matches");
        eFeature2D->InsertEndChild(eFeature2DNextMatches);

        eFeature2D->SetAttribute("next_matches_size", static_cast<unsigned int>(feature2D->nextMatches().size()));
        for (size_t i = 0; i < feature2D->nextMatches().size(); ++i)
        {
            char keyName[255];
            sprintf(keyName, "next_matches_%lu", i);

            if (feature2D->nextMatches().at(i).empty())
            {
                std::cout << "# WARNING: Empty Point2DFeaturePtr instance." << std::endl;
            }

            char feature2DName[255];
            sprintf(feature2DName, "F2D-%lu", feature2DMap[feature2D->nextMatches().at(i)]);
            eFeature2DNextMatches->SetAttribute(keyName, feature2DName);
        }

        if (!feature2D->feature3D().empty())
        {
            char feature3DName[255];
            sprintf(feature3DName, "F3D-%lu", feature3DMap[feature2D->feature3D()]);
            eFeature2D->SetAttribute("feature3D", feature3DName);
        }

        const Frame* frame = feature2D->frame();
        char frameName[255];
        sprintf(frameName, "frame%lu", frameMap[const_cast<Frame*>(frame)]);
        eFeature2D->SetAttribute("frame", frameName);
    }

    for (boost::unordered_map<const Point3DFeature*,tinyxml2::XMLElement*>::iterator it = feature3DXMLMap.begin();
             it != feature3DXMLMap.end(); ++it)
    {
        const Point3DFeature* feature3D = it->first;
        tinyxml2::XMLElement* eFeature3D = it->second;

        eFeature3D->SetAttribute("features2D_size", static_cast<unsigned int>(feature3D->features2D().size()));

        tinyxml2::XMLElement* eFeature2DCorr = doc.NewElement("features2D");
        eFeature3D->InsertEndChild(eFeature2DCorr);

        for (size_t i = 0; i < feature3D->features2D().size(); ++i)
        {
            char keyName[255];
            sprintf(keyName, "features2D_%lu", i);

            if (feature3D->features2D().at(i).empty())
            {
                std::cout << "# WARNING: Empty Point2DFeaturePtr instance." << std::endl;
            }

            char feature2DName[255];
            sprintf(feature2DName, "F2D-%lu", feature2DMap[feature3D->features2D().at(i)]);
            eFeature2DCorr->SetAttribute(keyName, feature2DName);
        }
    }

    eFrames->SetAttribute("size", static_cast<unsigned int>(frameMap.size()));
    ePoses->SetAttribute("size", static_cast<unsigned int>(poseMap.size()));
    eOdometers->SetAttribute("size", static_cast<unsigned int>(odometerMap.size()));
    eFeatures2D->SetAttribute("size", static_cast<unsigned int>(feature2DMap.size()));
    eFeatures3D->SetAttribute("size", static_cast<unsigned int>(feature3DMap.size()));

    doc.SaveFile(filename.c_str());
}

tinyxml2::XMLElement*
SparseGraph::frameToXML(FramePtr& frame, tinyxml2::XMLDocument& doc,
                        tinyxml2::XMLElement* parent,
                        boost::unordered_map<Frame*,size_t>& map,
                        boost::unordered_map<Frame*,tinyxml2::XMLElement*>& xmlMap,
                        const boost::filesystem::path& imageDir) const
{
    char frameName[255];
    sprintf(frameName, "frame%lu", map.size());

    tinyxml2::XMLElement* eFrame = doc.NewElement(frameName);
    parent->InsertEndChild(eFrame);

    if (!frame->image().empty())
    {
        char imageFilename[1024];
        sprintf(imageFilename, "%s/%s.png",
                imageDir.string().c_str(), frameName);
        cv::imwrite(imageFilename, frame->image());

        memset(imageFilename, 0, 1024);
        sprintf(imageFilename, "images/%s.png", frameName);
        eFrame->SetAttribute("image", imageFilename);
    }

    eFrame->SetAttribute("id", frame->id());

    Frame* pFrame = frame;

    map.insert(std::make_pair(pFrame, map.size()));
    xmlMap.insert(std::make_pair(pFrame, eFrame));

    return eFrame;
}

void
SparseGraph::XMLToFrames(tinyxml2::XMLElement* parent, unsigned int count,
                         FrameSegment& map,
                         std::vector<tinyxml2::XMLElement*>& xmlMap,
                         const boost::filesystem::path& rootDir) const
{
    map.resize(count);
    xmlMap.resize(count);

    tinyxml2::XMLElement* eFrame = eFrame = parent->FirstChildElement();

    for (unsigned int i = 0; i < count; ++i)
    {
        unsigned int frameIdx;
        sscanf(eFrame->Name(), "frame%u", &frameIdx);

        FramePtr frame = new Frame;

        const char* imageFilename = eFrame->Attribute("image");
        if (imageFilename != 0)
        {
            boost::filesystem::path imagePath = rootDir;
            imagePath /= imageFilename;

            frame->image() = cv::imread(imagePath.string().c_str(), -1);
        }

        eFrame->QueryUnsignedAttribute("id", &(frame->id()));

        map.at(frameIdx) = frame;
        xmlMap.at(frameIdx) = eFrame;

        eFrame = eFrame->NextSiblingElement();
    }
}

void
SparseGraph::point2DFeatureToXML(Point2DFeaturePtr& feature2D, tinyxml2::XMLDocument& doc,
                                 tinyxml2::XMLElement* parent,
                                 boost::unordered_map<const Point2DFeature*,size_t>& map,
                                 boost::unordered_map<const Point2DFeature*,tinyxml2::XMLElement*>& xmlMap) const
{
    boost::unordered_map<const Point2DFeature*,size_t>::iterator it = map.find(feature2D);
    if (it != map.end())
    {
        return;
    }

    char feature2DName[255];
    sprintf(feature2DName, "F2D-%lu", map.size());
    tinyxml2::XMLElement* eFeature2D = doc.NewElement(feature2DName);
    parent->InsertEndChild(eFeature2D);

    tinyxml2::XMLElement* eDtor = doc.NewElement("dtor");
    eFeature2D->InsertEndChild(eDtor);

    const cv::Mat& dtor = feature2D->descriptor();

    eDtor->SetAttribute("type", dtor.type());
    eDtor->SetAttribute("rows", dtor.rows);
    eDtor->SetAttribute("cols", dtor.cols);

    tinyxml2::XMLElement* eMat = doc.NewElement("mat");
    eDtor->InsertEndChild(eMat);

    for (int r = 0; r < dtor.rows; ++r)
    {
        for (int c = 0; c < dtor.cols; ++c)
        {
            char elementName[255];
            sprintf(elementName, "m-%d-%d", r, c);

            switch (dtor.type())
            {
            case CV_8U:
                eMat->SetAttribute(elementName, static_cast<int>(dtor.at<unsigned char>(r,c)));
                break;
            case CV_8S:
                eMat->SetAttribute(elementName, static_cast<int>(dtor.at<char>(r,c)));
                break;
            case CV_16U:
                eMat->SetAttribute(elementName, static_cast<int>(dtor.at<unsigned short>(r,c)));
                break;
            case CV_16S:
                eMat->SetAttribute(elementName, static_cast<int>(dtor.at<short>(r,c)));
                break;
            case CV_32S:
                eMat->SetAttribute(elementName, dtor.at<int>(r,c));
                break;
            case CV_32F:
                eMat->SetAttribute(elementName, static_cast<double>(dtor.at<float>(r,c)));
                break;
            case CV_64F:
            default:
                eMat->SetAttribute(elementName, dtor.at<double>(r,c));
            }
        }
    }

    eFeature2D->SetAttribute("kp_angle", feature2D->keypoint().angle);
    eFeature2D->SetAttribute("kp_class_id", feature2D->keypoint().class_id);
    eFeature2D->SetAttribute("kp_octave", feature2D->keypoint().octave);
    eFeature2D->SetAttribute("kp_x", feature2D->keypoint().pt.x);
    eFeature2D->SetAttribute("kp_y", feature2D->keypoint().pt.y);
    eFeature2D->SetAttribute("kp_response", feature2D->keypoint().response);
    eFeature2D->SetAttribute("kp_size", feature2D->keypoint().size);
    eFeature2D->SetAttribute("index", feature2D->index());
    eFeature2D->SetAttribute("best_prev_match_idx", feature2D->bestPrevMatchIdx());
    eFeature2D->SetAttribute("best_next_match_idx", feature2D->bestNextMatchIdx());

    map.insert(std::make_pair(feature2D, map.size()));
    xmlMap.insert(std::make_pair(feature2D, eFeature2D));
}

void
SparseGraph::XMLToPoint2DFeatures(tinyxml2::XMLElement* parent, unsigned int count,
                                  std::vector<Point2DFeaturePtr>& map,
                                  std::vector<tinyxml2::XMLElement*>& xmlMap) const
{
    map.resize(count);
    xmlMap.resize(count);

    tinyxml2::XMLElement* eFeature2D = parent->FirstChildElement();

    for (unsigned int i = 0; i < count; ++i)
    {
        unsigned int featureIdx;
        sscanf(eFeature2D->Name(), "F2D-%u", &featureIdx);

        Point2DFeaturePtr feature2D = new Point2DFeature;

        tinyxml2::XMLElement* eDtor = eFeature2D->FirstChildElement("dtor");

        int type, rows, cols;
        eDtor->QueryIntAttribute("type", &type);
        eDtor->QueryIntAttribute("rows", &rows);
        eDtor->QueryIntAttribute("cols", &cols);

        feature2D->descriptor() = cv::Mat(rows, cols, type);

        cv::Mat& dtor = feature2D->descriptor();

        const tinyxml2::XMLAttribute* aMat = eDtor->FirstChildElement("mat")->FirstAttribute();
        int matSize = rows * cols;
        for (int j = 0; j < matSize; ++j)
        {
            int r, c;
            sscanf(aMat->Name(), "m-%d-%d", &r, &c);

            int iElement;
            double dElement;

            switch (dtor.type())
            {
            case CV_8U:
            case CV_8S:
            case CV_16U:
            case CV_16S:
            case CV_32S:
                iElement = aMat->IntValue();
                break;
            case CV_32F:
            case CV_64F:
            default:
                dElement = aMat->DoubleValue();
            }

            switch (dtor.type())
            {
            case CV_8U:
                dtor.at<unsigned char>(r,c) = static_cast<unsigned char>(iElement);
                break;
            case CV_8S:
                dtor.at<char>(r,c) = static_cast<char>(iElement);
                break;
            case CV_16U:
                dtor.at<unsigned short>(r,c) = static_cast<unsigned short>(iElement);
                break;
            case CV_16S:
                dtor.at<short>(r,c) = static_cast<short>(iElement);
                break;
            case CV_32S:
                dtor.at<int>(r,c) = iElement;
                break;
            case CV_32F:
                dtor.at<float>(r,c) = static_cast<float>(dElement);
                break;
            case CV_64F:
            default:
                dtor.at<double>(r,c) = dElement;
            }

            aMat = aMat->Next();
        }

        eFeature2D->QueryFloatAttribute("kp_angle", &feature2D->keypoint().angle);
        eFeature2D->QueryIntAttribute("kp_class_id", &feature2D->keypoint().class_id);
        eFeature2D->QueryIntAttribute("kp_octave", &feature2D->keypoint().octave);
        eFeature2D->QueryFloatAttribute("kp_x", &feature2D->keypoint().pt.x);
        eFeature2D->QueryFloatAttribute("kp_y", &feature2D->keypoint().pt.y);
        eFeature2D->QueryFloatAttribute("kp_response", &feature2D->keypoint().response);
        eFeature2D->QueryFloatAttribute("kp_size", &feature2D->keypoint().size);
        eFeature2D->QueryUnsignedAttribute("index", &feature2D->index());
        eFeature2D->QueryIntAttribute("best_prev_match_idx", &feature2D->bestPrevMatchIdx());
        eFeature2D->QueryIntAttribute("best_next_match_idx", &feature2D->bestNextMatchIdx());

        map.at(featureIdx) = feature2D;
        xmlMap.at(featureIdx) = eFeature2D;

        eFeature2D = eFeature2D->NextSiblingElement();
    }
}

void
SparseGraph::point3DFeatureToXML(Point3DFeaturePtr& feature3D, tinyxml2::XMLDocument& doc,
                                 tinyxml2::XMLElement* parent,
                                 boost::unordered_map<const Point3DFeature*,size_t>& map,
                                 boost::unordered_map<const Point3DFeature*,tinyxml2::XMLElement*>& xmlMap) const
{
    boost::unordered_map<const Point3DFeature*,size_t>::iterator it = map.find(feature3D);
    if (it != map.end())
    {
        return;
    }

    char feature3DName[255];
    sprintf(feature3DName, "F3D-%lu", map.size());
    tinyxml2::XMLElement* eFeature3D = doc.NewElement(feature3DName);
    parent->InsertEndChild(eFeature3D);

    Eigen::Vector3d& P = feature3D->point();
    eFeature3D->SetAttribute("x", P(0));
    eFeature3D->SetAttribute("y", P(1));
    eFeature3D->SetAttribute("z", P(2));

    map.insert(std::make_pair(feature3D, map.size()));
    xmlMap.insert(std::make_pair(feature3D, eFeature3D));
}

void
SparseGraph::XMLToPoint3DFeatures(tinyxml2::XMLElement* parent, unsigned int count,
                                  std::vector<Point3DFeaturePtr>& map,
                                  std::vector<tinyxml2::XMLElement*>& xmlMap) const
{
    map.resize(count);
    xmlMap.resize(count);

    tinyxml2::XMLElement* eFeature3D = parent->FirstChildElement();

    for (unsigned int i = 0; i < count; ++i)
    {
        unsigned int featureIdx;
        sscanf(eFeature3D->Name(), "F3D-%u", &featureIdx);

        Point3DFeaturePtr feature3D = new Point3DFeature;

        Eigen::Vector3d& P = feature3D->point();
        eFeature3D->QueryDoubleAttribute("x", &P(0));
        eFeature3D->QueryDoubleAttribute("y", &P(1));
        eFeature3D->QueryDoubleAttribute("z", &P(2));

        map.at(featureIdx) = feature3D;
        xmlMap.at(featureIdx) = eFeature3D;

        eFeature3D = eFeature3D->NextSiblingElement();
    }
}

void
SparseGraph::poseToXML(PosePtr& pose, tinyxml2::XMLDocument& doc,
                       tinyxml2::XMLElement* parent,
                       boost::unordered_map<const Pose*,size_t>& map) const
{
    boost::unordered_map<const Pose*,size_t>::iterator it = map.find(pose);
    if (it != map.end())
    {
        return;
    }

    char poseName[255];
    sprintf(poseName, "pose%lu", map.size());
    tinyxml2::XMLElement* ePose = doc.NewElement(poseName);
    parent->InsertEndChild(ePose);

    const double* const q = pose->rotationData();
    ePose->SetAttribute("q_x", q[0]);
    ePose->SetAttribute("q_y", q[1]);
    ePose->SetAttribute("q_z", q[2]);
    ePose->SetAttribute("q_w", q[3]);

    const double* const t = pose->translationData();
    ePose->SetAttribute("t_x", t[0]);
    ePose->SetAttribute("t_y", t[1]);
    ePose->SetAttribute("t_z", t[2]);

    map.insert(std::make_pair(pose, map.size()));
}

void
SparseGraph::XMLToPoses(tinyxml2::XMLElement* parent, unsigned int count,
                        std::vector<PosePtr>& map) const
{
    map.resize(count);

    tinyxml2::XMLElement* ePose = parent->FirstChildElement();

    for (unsigned int i = 0; i < count; ++i)
    {
        unsigned int poseIdx;
        sscanf(ePose->Name(), "pose%u", &poseIdx);

        PosePtr pose = new Pose;

        double q[4];
        ePose->QueryDoubleAttribute("q_x", &q[0]);
        ePose->QueryDoubleAttribute("q_y", &q[1]);
        ePose->QueryDoubleAttribute("q_z", &q[2]);
        ePose->QueryDoubleAttribute("q_w", &q[3]);

        memcpy(pose->rotationData(), q, sizeof(double) * 4);

        double t[3];
        ePose->QueryDoubleAttribute("t_x", &t[0]);
        ePose->QueryDoubleAttribute("t_y", &t[1]);
        ePose->QueryDoubleAttribute("t_z", &t[2]);

        memcpy(pose->translationData(), t, sizeof(double) * 3);

        map.at(poseIdx) = pose;

        ePose = ePose->NextSiblingElement();
    }
}

void
SparseGraph::odometerToXML(OdometerPtr& odometer, tinyxml2::XMLDocument& doc,
                           tinyxml2::XMLElement* parent,
                           boost::unordered_map<const Odometer*,size_t>& map) const
{
    boost::unordered_map<const Odometer*,size_t>::iterator it = map.find(odometer);
    if (it != map.end())
    {
        return;
    }

    char odometerName[255];
    sprintf(odometerName, "odometer%lu", map.size());
    tinyxml2::XMLElement* eOdometer = doc.NewElement(odometerName);
    parent->InsertEndChild(eOdometer);

    std::ostringstream oss;
    oss << odometer->timeStamp();
    eOdometer->SetAttribute("timestamp", oss.str().c_str());

    eOdometer->SetAttribute("x", odometer->x());
    eOdometer->SetAttribute("y", odometer->y());
    eOdometer->SetAttribute("yaw", odometer->yaw());

    map.insert(std::make_pair(odometer, map.size()));
}

void
SparseGraph::XMLToOdometers(tinyxml2::XMLElement* parent, unsigned int count,
                            std::vector<OdometerPtr>& map) const
{
    map.resize(count);

    tinyxml2::XMLElement* eOdometer = parent->FirstChildElement();

    for (unsigned int i = 0; i < count; ++i)
    {
        unsigned int odometerIdx;
        sscanf(eOdometer->Name(), "odometer%u", &odometerIdx);

        OdometerPtr odometer = new Odometer;

        std::istringstream iss(eOdometer->Attribute("timestamp"));
        iss >> odometer->timeStamp();

        eOdometer->QueryDoubleAttribute("x", &odometer->x());
        eOdometer->QueryDoubleAttribute("y", &odometer->y());
        eOdometer->QueryDoubleAttribute("yaw", &odometer->yaw());

        map.at(odometerIdx) = odometer;

        eOdometer = eOdometer->NextSiblingElement();
    }
}

}
