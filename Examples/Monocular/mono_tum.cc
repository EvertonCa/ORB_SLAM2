#include <iostream>
#include <algorithm> 
#include <fstream>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <System.h>

#include <semaphore.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <fcntl.h>

#define HEIGHT 480
#define WIDTH 640
#define CHANNELS 3
#define CAMERA_BLOCK_SIZE (WIDTH*HEIGHT*CHANNELS)
#define IPC_RESULT_ERROR (-1)

using namespace std;
int main(int argc, char **argv) 
{
   if(argc != 3) 
   {
     cerr << endl << "Usage: ./path_to_PF_ORB path_to_vocabulary path_to_settings path_to_dev_video" << endl;
     return 1;
   }
   // Create SLAM system. It initializes all system threads and gets ready to process frames.
   ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

   cout << endl << "-------" << endl;
   cout << "Start processing sequence ..." << endl;

   sem_t *sem_prod_cam = sem_open("/slamcamproducer", 0);
    if (sem_prod_cam == SEM_FAILED) {
        perror("sem_open/slamcamproducer");
        exit(EXIT_FAILURE);
    }

    sem_t *sem_cons_cam = sem_open("/slamcamconsumer", 1);
    if (sem_cons_cam == SEM_FAILED) {
        perror("sem_open/slamcamconsumer");
        exit(EXIT_FAILURE);
    }

    key_t key;

    // request a key
    // the key is linked to a filename, so that other programs can access it
    if (-1 != open("/tmp/blockcam", O_CREAT, 0777)) {
        key = key = ftok("/tmp/blockcam", 0);
    } else {
        perror("open");
        exit(1);
    }

    // get shared block --- create it if it doesn't exist
    int shared_block_id = shmget(key, CAMERA_BLOCK_SIZE, IPC_CREAT | SHM_R | SHM_W );

    char *result;

    if (shared_block_id == IPC_RESULT_ERROR) {
        result = NULL;
    }

    //map the shared block int this process's memory and give me a pointer to it
    result = (char*) shmat(shared_block_id, NULL, 0);
    if (result == (char *)IPC_RESULT_ERROR) {
        result = NULL;
    }

   // Main loop
   int timeStamps=0;
   for(;;timeStamps++)
   {

        sem_wait(sem_prod_cam); // wait for the producer to have an open slot
        cv::Mat temp = cv::Mat(HEIGHT, WIDTH, 16, result, CHANNELS * WIDTH); // creates a frame from memory
        cv::Mat mat;
        cv::cvtColor(temp, mat, cv::COLOR_BGR2RGB);
        // Pass the image to the SLAM system
        cv::Mat Tcw = SLAM.TrackMonocular(mat, timeStamps);
        sem_post(sem_cons_cam); // signal that data was acquired
    }

   // Stop all threads
   SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    return 0;
}