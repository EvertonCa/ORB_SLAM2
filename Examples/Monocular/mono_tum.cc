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

#define HEIGHT 720
#define WIDTH 1280
#define CHANNELS 3
#define CAMERA_BLOCK_SIZE (WIDTH*HEIGHT*CHANNELS)
#define IPC_RESULT_ERROR (-1)
#define MESSAGE_BLOCK_SIZE 4096
#define FILENAME_CAM "/tmp/blockcam"
#define FILENAME_MESSAGE_SLAM "/tmp/blockslam"
#define SLAM_SEM_CAM_PRODUCER_FNAME "/slamcamproducer"
#define SLAM_SEM_CAM_CONSUMER_FNAME "/slamcamconsumer"
#define SLAM_SEM_MESSAGE_PRODUCER_FNAME "/slammesproducer"
#define SLAM_SEM_MESSAGE_CONSUMER_FNAME "/slammesconsumer"

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

    sem_t *sem_prod_cam = sem_open(SLAM_SEM_CAM_PRODUCER_FNAME, 0);
    if (sem_prod_cam == SEM_FAILED) {
        perror("sem_open/slamcamproducer");
        exit(EXIT_FAILURE);
    }

    sem_t *sem_cons_cam = sem_open(SLAM_SEM_CAM_CONSUMER_FNAME, 1);
    if (sem_cons_cam == SEM_FAILED) {
        perror("sem_open/slamcamconsumer");
        exit(EXIT_FAILURE);
    }

    sem_t *sem_prod_message = sem_open(SLAM_SEM_MESSAGE_PRODUCER_FNAME, 0);
    if (sem_prod_message == SEM_FAILED) {
        perror("sem_open/slamcamproducer");
        exit(EXIT_FAILURE);
    }

    sem_t *sem_cons_message = sem_open(SLAM_SEM_MESSAGE_CONSUMER_FNAME, 1);
    if (sem_cons_message == SEM_FAILED) {
        perror("sem_open/slamcamconsumer");
        exit(EXIT_FAILURE);
    }

    // CAMERA SHARED MEMORY

    key_t keyCam;

    // request a key
    // the key is linked to a filename, so that other programs can access it
    if (-1 != open(FILENAME_CAM, O_CREAT, 0777)) {
        keyCam = ftok(FILENAME_CAM, 0);
    } else {
        perror("open");
        exit(1);
    }

    // get shared block --- create it if it doesn't exist
    int shared_cam_block_id = shmget(keyCam, CAMERA_BLOCK_SIZE, IPC_CREAT | SHM_R | SHM_W );

    char *result_cam;

    if (shared_cam_block_id == IPC_RESULT_ERROR) {
        result_cam = NULL;
        printf("shared_cam_block_id failed\n");
        exit(1);
    }

    //map the shared block int this process's memory and give me a pointer to it
    result_cam = (char*) shmat(shared_cam_block_id, NULL, 0);
    if (result_cam == (char *)IPC_RESULT_ERROR) {
        result_cam = NULL;
        printf("result_cam failed\n");
        exit(1);
    }

    // MESSAGE SHARED MEMORY

    key_t keyMes;

    // request a key
    // the key is linked to a filename, so that other programs can access it
    if (-1 != open(FILENAME_MESSAGE_SLAM, O_CREAT, 0777)) {
        keyMes = ftok(FILENAME_MESSAGE_SLAM, 0);
    } else {
        perror("open");
        exit(1);
    }

    // get shared block --- create it if it doesn't exist
    int shared_message_block_id = shmget(keyMes, MESSAGE_BLOCK_SIZE, IPC_CREAT | SHM_R | SHM_W );

    char *result_message;

    if (shared_message_block_id == IPC_RESULT_ERROR) {
        result_message = NULL;
    }

    //map the shared block int this process's memory and give me a pointer to it
    result_message = (char*) shmat(shared_message_block_id, NULL, 0);
    if (result_message == (char *)IPC_RESULT_ERROR) {
        result_message = NULL;
    }

    // Main loop
    int timeStamps=0;
    for(;;timeStamps++)
    {

        sem_wait(sem_prod_cam); // wait for the producer to have an open slot
        
        cv::Mat temp = cv::Mat(HEIGHT, WIDTH, 16, result_cam, CHANNELS * WIDTH); // creates a frame from memory
        cv::Mat mat;
        cv::cvtColor(temp, mat, cv::COLOR_BGR2RGB);
        // Pass the image to the SLAM system
        cv::Mat Tcw = SLAM.TrackMonocularTCC(mat, timeStamps, sem_cons_message, sem_prod_message, result_message);
        sem_post(sem_cons_cam); // signal that data was acquired
    }

    // Stop all threads
    SLAM.Shutdown();
    return 0;
}