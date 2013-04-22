#include <stdio.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <vtkCubeSource.h>
#include <vtkSphereSource.h>
#include <vtkArrowSource.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkMath.h>
#include <vtkSphereSource.h>
#include <vtkProperty.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <time.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "show_vtk");
    ros::NodeHandle node;
    tf::TransformListener listener;
    tf::StampedTransform tfCam;

    // Create the origin
    vtkSmartPointer<vtkAxesActor> axes =
      vtkSmartPointer<vtkAxesActor>::New();
    vtkSmartPointer<vtkTransform> transform =
      vtkSmartPointer<vtkTransform>::New();
    transform->Translate(0.0, 0.0, 0.0);
    axes->SetUserTransform(transform);


    // Create a sphere
    vtkSmartPointer<vtkCubeSource> sphereSource =
            vtkSmartPointer<vtkCubeSource>::New();
    sphereSource->SetCenter(0.0, 0.0, 0.0);
    sphereSource->SetXLength(0.3);
    sphereSource->SetYLength(0.3);
    sphereSource->SetZLength(0.3);

    vtkSmartPointer<vtkPolyDataMapper> mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(sphereSource->GetOutputPort());

    vtkSmartPointer<vtkActor> actor =
            vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    vtkSmartPointer<vtkRenderer> renderer =
            vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renderWindow =
            vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    vtkSmartPointer<vtkRenderWindowInteractor> interactor =
            vtkSmartPointer<vtkRenderWindowInteractor>::New();
    interactor->SetRenderWindow(renderWindow);

    renderer->AddActor(actor);
    renderer->AddActor(axes);

    renderWindow->Render();
    interactor->Start();

    printf("enter loop\n");
    while(ros::ok())
    {
        ros::spinOnce();
        try
        {
            listener.waitForTransform("world", "cam",
                                    ros::Time(0), ros::Duration(1.0));
            listener.lookupTransform("world", "cam",
                                   ros::Time(0), tfCam);
//            tf::Transform t = tfCam.Transform.inverse();
            tfCam.setData(tfCam.inverse());

            tf::Vector3 orig = tfCam.getOrigin();
//            orig.setX(orig.x()*1.3);
//            orig.setY(orig.y()*1.3);
//            orig.setZ(-orig.z()*1.3);
            tf::Quaternion rot = tfCam.getRotation();

            printf("%f, %f, %f\n", orig.x(), orig.y(), orig.z());

            actor->SetPosition(orig.x(), orig.y(), orig.z());
            actor->RotateWXYZ(rot.w(), rot.x(), rot.y(), rot.z());
            renderer->Render();
            interactor->Render();

        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
        }
    }




    return EXIT_SUCCESS;
}
