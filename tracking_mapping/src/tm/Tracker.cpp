/* File:    Tracker.cpp
   Author:  Jiayi Liu
   Date:    Mar 17, 2013
   Description:


  */
#include <set>
#include <cstdio>

#include "tracking_mapping/tm/Tracker.h"
#include "tracking_mapping/tm/myutils.h"
#include "jlUtilities/opencv_helper.h"

using namespace myutils;

Tracker::Tracker()
{
    flag_first_image = true;
    current_state = TRCK_STATE_INITING;
    camera = initializer.pattern.camera;

    if(initializer.current_state == initializer.INIT_STATE_SUCCESS)
    {
        text_state = std::string("Tracker: Initializing the Initializer...");
        printf("Tracker: constructed.\n");
    }
    else
    {
        current_state = TRCK_STATE_FAILED;
        text_state = std::string("Tracker: NOT initialized; Init failed.");
    }

}
bool Tracker::init(int mode)
{
    if (!mat_image_current.data)	{
        printf("error - image is empty\n");
        return false;
    }

//    // Convert current image to grayscale
//    cv::Mat	imageInputGray;
//    cvtColor(mat_image_current,imageInputGray,CV_BGR2GRAY);


    // Dr. Hoff's pattern
    // !!! targets needs to be converted to cvPoints
    cv::Vec2d targets[5];	// CCC targets in the order UL-UM-UR-LL-LR 
    bool fFound;
    std::vector<cv::Point2d> targets2d;

    initializer.mode = mode;

    // !!! Later, member function .mode  needs to be integrated
    if(mode == 0)
    {
        fFound = initializer.processOnDataset(mat_image_current);
//        fFound = initializer.findCCC.process(imageInputGray, targets);
    }
    else if(mode == 1)
    {
        fFound = initializer.process(mat_image_current);
    }
    else if(mode == -1)
    {
        fFound = initializer.processOnDataset();
    }
//    drawKeypoints( mat_image_current, initializer.current_frame.vec_keypoints, mat_image_canvas,
//                   Scalar::all(-1), DrawMatchesFlags::DEFAULT );

//    for (int i=0; i<initializer.matches.size(); i++)
//    {
//        cv::circle(mat_image_canvas,
//                   initializer.vec_current_keypoints[initializer.matches[i].queryIdx].pt,	// center
//            3,							// radius
//            cv::Scalar(0,0,255),		// color
//            -1);						// negative thickness=filled

//        char szLabel[50];
//        sprintf(szLabel, "%d",
//                initializer.matches[i].trainIdx);
//        putText (mat_image_canvas, szLabel,
//                 initializer.vec_current_keypoints[initializer.matches[i].queryIdx].pt,
//            cv::FONT_HERSHEY_PLAIN, // font face
//            2.0,					// font scale
//            cv::Scalar(255,0,0),	// font color
//            2);						// thickness
//    }

//    drawMatches(mat_image_current, initializer.current_frame.vec_keypoints,
//                initializer.pattern.mat_image, initializer.pattern.vec_keypoints,
//                initializer.matches, mat_image_canvas);

//    // Draw targets and label them
//    if (fFound)
//    {

//        for (int i=0; i<5; i++)	{
//            cv::circle(mat_image_canvas,
//                cv::Point2d(targets[i]),	// center
//                3,							// radius
//                cv::Scalar(0,0,255),		// color
//                -1);						// negative thickness=filled

//            char szLabel[50];
//            sprintf(szLabel, "%d", i);
//            putText (mat_image_canvas, szLabel, cv::Point2d(targets[i]),
//                cv::FONT_HERSHEY_PLAIN, // font face
//                2.0,					// font scale
//                cv::Scalar(255,0,0),	// font color
//                2);						// thickness
//        }

    if(fFound)
    {
//        jlUtilities::draw_pointCoord(mat_image_canvas, initializer.pattern.vec_corners_currentImg);
//        // check
        jlUtilities::draw_2dContour(mat_image_canvas,
                                    initializer.pattern.vec_corners_currentImg,
                                    CV_RGB(200,0,0));

        camera = initializer.pattern.camera;
//        //-- Localize the object
//        std::vector<Point2f> obj;
//        std::vector<Point2f> scene;

//        for( int i = 0; i < initializer.matches.size(); i++ )
//        {
//            //-- Get the keypoints from the good matches
//            obj.push_back( initializer.vec_current_keypoints[ initializer.matches[i].queryIdx ].pt );
//            scene.push_back( initializer.vec_keypoints[ initializer.matches[i].trainIdx ].pt );
//        }
//        //    printf("pushed: %d obj and %d scene.", obj.size(), scene.size());

//        Mat H = findHomography( obj, scene, CV_RANSAC );

//        std::cout << H << std::endl;



//        std::vector<Point2f> points2d;
//        std::vector<Point3d> points2dHomo;

//        for(int i=0; i<initializer.vec_current_keypoints.size();i++)
//        {
//            Point3d p;
//            p.x = initializer.vec_current_keypoints[i].pt.x;
//            p.y = initializer.vec_current_keypoints[i].pt.y;
//            p.z = 1;
//            points2dHomo.push_back(p);
////            points2d.push_back(initializer.vec_current_keypoints[i].pt);
//        }



////        convertPointsToHomogeneous(points2d, points2dHomo);
//        printf("hi\n");
//        Mat myMatPoints2dHomo = Mat(points2dHomo).reshape(1);
////        Mat myMatPoints2dHomo_t(points2dHomo,true);
////         = myMatPoints2dHomo_t.reshape(0, points2dHomo.size());
//        std::cout<< myMatPoints2dHomo.rows << " " << myMatPoints2dHomo.cols << std::endl;
//        Mat mat_points3d = initializer.mat_intrinsics * myMatPoints2dHomo;

//        printf("hi1\n");

//        solvePnPRansac(mat_points3d, points2d,
//                       camM, NULL, vec_rot, vec_trans  );

//        printf("hi2\n");







//    // Get the pose of the target, from the image points

//        Pose pose_m_c = initializer.poseCCC.getPose(camera.K, camera.Kinv, targets);

        // Show the representation of the pose in terms of (ax,ay,az,tx,ty,tz)
//        double ax, ay, az, tx, ty, tz;
//        pose_m_c.getXYZangles(ax, ay, az);
//        pose_m_c.getTranslation(tx, ty, tz);
//        char sz[80];
//        sprintf(sz, "ax=%.1f ay=%.1f az=%.1f", ax, ay, az);
//        putText (mat_image_canvas, sz, cv::Point(40,400),
//            cv::FONT_HERSHEY_PLAIN, // font face
//            2.0,					// font scale
//            cv::Scalar(255,0,0),	// font color
//            2);
//        sprintf(sz, "tx=%.1f ty=%.1f tz=%.1f", tx, ty, tz);
//        putText (mat_image_canvas, sz, cv::Point(40,440),
//            cv::FONT_HERSHEY_PLAIN, // font face
//            2.0,					// font scale
//            cv::Scalar(255,0,0),	// font color
//            2);

        jlUtilities::draw_axes(mat_image_canvas, camera.mat_intrinsics, camera.mat_transform, 30);	// Draw model coordinate axes

//        char sz[80];
//        sprintf(sz, "Matches: %d | FPS: %d", initializer.matches.size());
//        jlUtilities::draw_text(mat_image_canvas, sz)
//        return true;
    }
//    else
//    {
//        return false;
//    }

//    // Find interest points
//    std::vector<cv::KeyPoint> keypoints;
//    initializer.interestPoints.process(imageInputGray, keypoints);


    return false;



}

int Tracker::updateFrame()
{
    of_matches.clear();
//    vecKeypointsPrevious.clear();
    vecKeypointsCurrent.clear();

    // Detect keypoints in the previous and current images
    FastFeatureDetector ffd;
//    ffd.detect(mat_image_previous, vecKeypointsPrevious);
    ffd.detect(mat_image_current, vecKeypointsCurrent);

    std::vector<Point2f> pts2fPrevious;
    KeyPoint::convert(vecKeypointsPrevious, pts2fPrevious);
    std::vector<Point2f> pts2fCurrent(pts2fPrevious.size());

    // making sure images are grayscale
    Mat matImagePrevious_gray, matImageCurrent_gray;
    if (mat_image_previous.channels() == 3)
    {
        cvtColor(mat_image_previous, matImagePrevious_gray, CV_BGR2GRAY);
        cvtColor(mat_image_current, matImageCurrent_gray, CV_BGR2GRAY);
    }
    else
    {
        matImagePrevious_gray = mat_image_previous.clone();
        matImageCurrent_gray = mat_image_current.clone();
    }

    // Calculate the optical flow field:
    // how each previous_point moved across the 2 images
    std::vector<uchar> ofStatus;
    std::vector<float> ofError;
//    calcOpticalFlowPyrLK(matImagePrevious_gray,
//                         matImageCurrent_gray,
//                         pts2fPrevious,
//                         pts2fCurrent,
//                         of_status,
//                         of_error);

    if(1)
    {

        try{
        calcOpticalFlowPyrLK(matImagePrevious_gray,
                             matImageCurrent_gray,
                             pts2fPrevious,
                             pts2fCurrent,
                             ofStatus,
                             ofError);
        }
        catch(cv::Exception &e)
        {
            std::cout<<"!!! OF failed: no feature" << std::endl;
            return -1;

        }
    // First, filter out the points with high error
    std::vector<Point2f> current_points_to_find;
    std::vector<int> current_points_to_find_back_index;
    for (unsigned int i=0; i<ofStatus.size(); i++)
    {
        if (ofStatus[i] && ofError[i] < 12.0)
        {
            // Keep the original index of the point in the
            // optical flow array, for future use
            current_points_to_find_back_index.push_back(i);
            // Keep the feature point itself
            current_points_to_find.push_back(pts2fCurrent[i]);
        }
    }

    // for each current_point see which detected feature it belongs to
    Mat current_points_to_find_flat =
            Mat(current_points_to_find).reshape(1, current_points_to_find.size()); //flatten array
    std::vector<Point2f> current_features; // detected features
    KeyPoint::convert(vecKeypointsCurrent, current_features);
    Mat current_features_flat =
            Mat(current_features).reshape(1, current_features.size());

    // Look around each OF point in the right image
    // for any features that were detected in its area
    // and make a match.
    BFMatcher matcher(CV_L2);
    std::vector<std::vector<DMatch> > nearest_neighbors;
    matcher.radiusMatch(current_points_to_find_flat,
                        current_features_flat,
                        nearest_neighbors,
                        2.0f);
    // Check that the found neighbors are unique (throw away neighbors
    // that are too close together, as they may be confusing)
    std::set<int> found_in_current_points; // for duplicate prevention
    for(int i=0; i<nearest_neighbors.size(); i++)
    {
        DMatch _m;
        if(nearest_neighbors[i].size() == 1)
        {
            _m = nearest_neighbors[i][0]; // only one neighbor
        }
        else if(nearest_neighbors[i].size() > 1)
        {
            // 2 neighbors – check how close they are
            double ratio = nearest_neighbors[i][0].distance /
                    nearest_neighbors[i][1].distance;
            if(ratio < 0.7)
            { // not too close
                // take the closest (first) one
                _m = nearest_neighbors[i][0];
            }
            else
            { // too close – we cannot tell which is better
                continue; // did not pass ratio test – throw away
            }
        }
        else
        {
            continue; // no neighbors... :(
        }
        // prevent duplicates
        if (found_in_current_points.find(_m.trainIdx) == found_in_current_points.end())
        {
            // The found neighbor was not yet used:
            // We should match it with the original indexing
            // of the left point
            _m.queryIdx = current_points_to_find_back_index[_m.queryIdx];
            of_matches.push_back(_m); // add this match
            found_in_current_points.insert(_m.trainIdx);
        }
    }






    std::cout<<"pruned "<< of_matches.size() <<" / "<< nearest_neighbors.size() <<" matches"<< std::endl;
}
    return 0;
}

void Tracker::drawOpticalFlowWithCurrent()
{


    for(int i=0; i<of_matches.size(); i++)
    {
        int line_thickness; line_thickness = 1;
        /* CV_RGB(red, green, blue) is the red, green, and blue components
         * of the color you want, each out of 255.
         */
         CvScalar line_color; line_color = CV_RGB(255,0,0);
        /* Let's make the flow field look nice with arrows. */
        /* The arrows will be a bit too short for a nice visualization because of the
        high framerate
         * (ie: there's not much motion between the frames). So let's lengthen them
        by a factor of 3.
         */
         CvPoint p,q;
         p.x = (int) vecKeypointsPrevious[of_matches[i].queryIdx].pt.x;
         p.y = (int) vecKeypointsPrevious[of_matches[i].queryIdx].pt.y;
         q.x = (int) vecKeypointsCurrent[of_matches[i].trainIdx].pt.x;
         q.y = (int) vecKeypointsCurrent[of_matches[i].trainIdx].pt.y;
        double angle; angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
        double hypotenuse; hypotenuse = sqrt( square(p.y - q.y) + square(p.x - q.x) )
        ;
        /* Here we lengthen the arrow by a factor of three. */
         q.x = (int) (p.x -  hypotenuse * cos(angle));
         q.y = (int) (p.y -  hypotenuse * sin(angle));
        /* Now we draw the main line of the arrow. */


         /* "frame1" is the frame to draw on.
          * "p" is the point where the line begins.
          * "q" is the point where the line stops.
          * "CV_AA" means antialiased drawing.
          * "0" means no fractional bits in the center cooridinate or radius.
          */
          line( mat_image_canvas, p, q, line_color, line_thickness, CV_AA, 0 );
         /* Now draw the tips of the arrow. I do some scaling so that the
          * tips look proportional to the main line of the arrow.
          */
          p.x = (int) (q.x + 9 * cos(angle + pi / 4));
          p.y = (int) (q.y + 9 * sin(angle + pi / 4));
          line( mat_image_canvas, p, q, line_color, line_thickness, CV_AA, 0 );
          p.x = (int) (q.x + 9 * cos(angle - pi / 4));
          p.y = (int) (q.y + 9 * sin(angle - pi / 4));
          line( mat_image_canvas, p, q, line_color, line_thickness, CV_AA, 0 );








//        line(mat_image_canvas,
//             vecKeypointsPrevious[of_matches[i].trainIdx].pt,
//             vecKeypointsCurrent[of_matches[i].queryIdx].pt,
//             Scalar(0, 0, 255));
//        circle(mat_image_canvas, vecKeypointsCurrent[of_matches[i].queryIdx].pt, 3, Scalar(0, 255, 0), 3);
//        circle(mat_image_canvas, vecKeypointsPrevious[of_matches[i].trainIdx].pt, 3, Scalar(255, 0, 0), 3);

    }
}

void Tracker::getFundamentalMat(Mat &F,
                                std::vector<cv::KeyPoint> keypoints1,
                                std::vector<cv::KeyPoint> keypoints2,
                                std::vector<DMatch> matches)
{
    std::vector<Point2f> imgpts1, imgpts2;
    for( unsigned int i = 0; i<matches.size(); i++ )
    {
        // queryIdx is the "left" image
        imgpts1.push_back(keypoints1[matches[i].queryIdx].pt);
        // trainIdx is the "right" image
        imgpts2.push_back(keypoints2[matches[i].trainIdx].pt);
    }
    F = findFundamentalMat(imgpts1, imgpts2, FM_RANSAC, 0.1, 0.99);
}

void Tracker::getCameraMat(const Mat& K,
//                        const Mat& Kinv,
                        const vector<KeyPoint>& imgpts1,
                        const vector<KeyPoint>& imgpts2,
//                        Matx34d& P,
                        Matx34d& P1,
                        vector<DMatch>& matches
//                        vector<CloudPoint>& outCloud
                        )
{
    //Find camera matrices
    //Get Fundamental Matrix
    Mat F;
    getFundamentalMat(F, imgpts1, imgpts2, matches);
    //Essential matrix: compute then extract cameras [R|t]
    Mat_<double> E = K.t() * F * K; //according to HZ (9.12)
    //decompose E to P' , HZ (9.19)
    SVD svd(E,SVD::MODIFY_A);
    Mat svd_u = svd.u;
    Mat svd_vt = svd.vt;
    Mat svd_w = svd.w;
    Matx33d W(0,-1,0,//HZ 9.13
              1,0,0,
              0,0,1);
    Mat_<double> R = svd_u * Mat(W) * svd_vt; //HZ 9.19
    Mat_<double> t = svd_u.col(2); //u3
//    if (!CheckCoherentRotation(R)) {
//        cout<<"resulting rotation is not coherent\n";
//        P1 = 0;
//        return;
//    }
    P1 = Matx34d(R(0,0),R(0,1),R(0,2),t(0),
                 R(1,0),R(1,1),R(1,2),t(1),
                 R(2,0),R(2,1),R(2,2),t(2));
}
