/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>

namespace ORB_SLAM2
{


MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

}

void MapDrawer::DrawMapPoints(const bool bDrawCurrentPoints)
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();
    const vector<MapPoint*> &vpCurrentMPs = mpMap->GetCurrentMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }

    glEnd();

    if (bDrawCurrentPoints)
    {
        // Define points
        glPointSize(5);
        glBegin(GL_POINTS);
        glColor3f(0.0, 1.0, 0.0);

        // All map points
        for (std::vector<MapPoint *>::const_iterator i = vpCurrentMPs.begin(); i != vpCurrentMPs.end(); i++)
        {

            if ((*i)->isBad())
                continue;
            cv::Mat pos = (*i)->GetWorldPos();
            glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
            //printf("x = %f, y = %f, z = %f ", pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
        }
        glEnd(); 

        std::vector<std::vector<cv::Mat>> coplanarMapPoints;

        for (std::vector<MapPoint *>::const_iterator i = vpCurrentMPs.begin(); i != vpCurrentMPs.end(); i++)
        {
            if ((*i)->isBad())
                    continue;

            cv::Mat posi = (*i)->GetWorldPos();
            std::vector<cv::Mat> temp;
            temp.push_back(posi);

            float coplanarThreshold = 0.02;

            for (std::vector<MapPoint *>::const_iterator j = vpCurrentMPs.begin(); j != vpCurrentMPs.end(); j++)
            {

                if ((*j)->isBad())
                    continue;
                cv::Mat posj = (*j)->GetWorldPos();
                if (posi.at<float>(2) <= (posj.at<float>(2) + coplanarThreshold) && posi.at<float>(2) >= (posj.at<float>(2) - coplanarThreshold))
                {
                    cv::Mat diff = posi != posj;
                    if (!(cv::countNonZero(diff) == 0))
                        temp.push_back(posj);
                }
            }
            coplanarMapPoints.push_back(temp);
        }

        std::sort(coplanarMapPoints.begin(), coplanarMapPoints.end(), [](const std::vector<cv::Mat> &a, const std::vector<cv::Mat> &b){ return a.size() > b.size(); });

        glPointSize(8);
        glBegin(GL_TRIANGLES);
        glColor3f(0.0, 0.0, 1.0);  

        std::vector<cv::Mat> extremes;

        if (!coplanarMapPoints.empty())
        {
            cv::Mat t(1, 3, 0.0f);
            extremes.push_back(t);
            extremes.push_back(t);
            extremes.push_back(t);
            extremes.push_back(t);
        }

        if (coplanarMapPoints.size() > 1)
        {
                
            for (long unsigned int col = 0; col < coplanarMapPoints.at(0).size(); col++)
            {
                if (coplanarMapPoints.at(0).at(col).at<float>(0) < extremes.at(0).at<float>(0))
                    extremes.at(0) = coplanarMapPoints.at(0).at(col);

                if (coplanarMapPoints.at(0).at(col).at<float>(0) > extremes.at(1).at<float>(0))
                    extremes.at(1) = coplanarMapPoints.at(0).at(col);

                if (coplanarMapPoints.at(0).at(col).at<float>(1) < extremes.at(2).at<float>(1))
                    extremes.at(2) = coplanarMapPoints.at(0).at(col);

                if (coplanarMapPoints.at(0).at(col).at<float>(1) > extremes.at(3).at<float>(1))
                    extremes.at(3) = coplanarMapPoints.at(0).at(col);
                //printf("x = %f, y = %f, z = %f ", coplanarMapPoints.at(row).at(col).at<float>(0), coplanarMapPoints.at(row).at(col).at<float>(1), coplanarMapPoints.at(row).at(col).at<float>(2));
            } 

            glVertex3f(extremes.at(0).at<float>(0), extremes.at(0).at<float>(1), extremes.at(0).at<float>(2));
            glVertex3f(extremes.at(2).at<float>(0), extremes.at(2).at<float>(1), extremes.at(2).at<float>(2));
            glVertex3f(extremes.at(3).at<float>(0), extremes.at(3).at<float>(1), extremes.at(3).at<float>(2));
            glVertex3f(extremes.at(1).at<float>(0), extremes.at(1).at<float>(1), extremes.at(1).at<float>(2));
            glVertex3f(extremes.at(2).at<float>(0), extremes.at(2).at<float>(1), extremes.at(2).at<float>(2));
            glVertex3f(extremes.at(3).at<float>(0), extremes.at(3).at<float>(1), extremes.at(3).at<float>(2));

            printf("Extremes\n");
            printf("x = %f, y = %f, z = %f\n", extremes.at(0).at<float>(0), extremes.at(0).at<float>(1), extremes.at(0).at<float>(2));
            printf("x = %f, y = %f, z = %f\n", extremes.at(1).at<float>(0), extremes.at(1).at<float>(1), extremes.at(1).at<float>(2));
            printf("x = %f, y = %f, z = %f\n", extremes.at(2).at<float>(0), extremes.at(2).at<float>(1), extremes.at(2).at<float>(2));
            printf("x = %f, y = %f, z = %f\n", extremes.at(3).at<float>(0), extremes.at(3).at<float>(1), extremes.at(3).at<float>(2));

            printf("\n");
        }

        glEnd();
        
    }
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

} //namespace ORB_SLAM
