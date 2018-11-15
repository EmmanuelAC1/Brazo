#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdio.h>
#include <math.h>
#include "dynamixel_sdk.h"
using namespace std;

// Control table address
#define ADDR_RX_CW_ANGLE                6
#define ADDR_RX_CCW_ANGLE               8
#define ADDR_RX_TORQUE_ENABLE           24
#define ADDR_RX_GOAL_POSITION           30
#define ADDR_RX_PRESENT_SPEED           32
#define ADDR_RX_PRESENT_POSITION        36
#define ADDR_RX_MOVING                  46


// Protocol version
#define PROTOCOL_VERSION                1.0


// Default setting
//#define DXL_ID                          1                   // Dynamixel ID: 1
#define BAUDRATE                        9615
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      0                   // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      1023                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b
#define PI                              3.14159265358979323846

#define SPEED      97
#define NUM_MOTORS 7


int DXL_ID[] = {0, 1, 2, 3, 4, 5, 6};
int DXL_error[]  = {0, -62, 188, 0, 0, 0, 0};


void radians_to_range(double *new_position){
    
    for(int i=0; i<NUM_MOTORS; i++){
        new_position[i] += PI;

        if(new_position[i] > 2.0*PI)
            new_position[i] -= (2.0*PI);

        if(new_position[i] < PI/6.0){
            printf("ID %d out of limit value\n", DXL_ID[i]);

            new_position[i] = PI/6.0;
        } else if(new_position[i] > 11.0*PI/6.0){
            printf("ID %d out of limit value\n", DXL_ID[i]);

            new_position[i] = 11.0*PI/6.0;
        }
    }

    double cons_conv = (3.0*1023.0)/(5.0*PI);

    for(int i=0; i<NUM_MOTORS; i++){
        new_position[i] = ((new_position[i] - PI/6.0)*cons_conv);

        new_position[i] += DXL_error[i];
    }
}

double range_to_radians(double position, int idx_motor){
    double cons_conv = (3.0*1023.0)/(5.0*PI);

    position -= DXL_error[idx_motor];

    position = (position / cons_conv) + PI/6.0;

    return position;
}

void print_status(int dxl_comm_result, uint8_t dxl_error, dynamixel::PacketHandler *packetHandler, string message){
    if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        else
        {
            ////
            printf("message\n");
        }
}

bool initial_config(double *curr_position, dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler){
    uint8_t dxl_error = 0;                          // Dynamixel error
    uint16_t dxl_present_position = 0;              // Present position
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE))
    {
      printf("Succeeded to change the baudrate!\n");
    }
    else
    {
      printf("Failed to change the baudrate!\n");
      return false;
    }


    for(int i = 0; i<NUM_MOTORS; i++){
        // Enable Dynamixel Torque
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID[i], ADDR_RX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        else
        {
            printf("Torque enabled \n");
        }


        // Set speed
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID[i], ADDR_RX_PRESENT_SPEED, SPEED, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        else
        {
            printf("ID %d speed has been updated to %d\n", DXL_ID[i], SPEED);
        }


        // Set minimum angle
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID[i], ADDR_RX_CW_ANGLE, DXL_MINIMUM_POSITION_VALUE, &dxl_error);
        


        // Set maximum angle
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID[i], ADDR_RX_CCW_ANGLE, DXL_MAXIMUM_POSITION_VALUE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        else
        {
            printf("ID %d maximum position value has been updated to %d\n", DXL_ID[i], DXL_MAXIMUM_POSITION_VALUE);
        }


        // Get initial position
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID[i], ADDR_RX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        else
        {
            curr_position[i] = dxl_present_position;
            printf("ID %d Present position %d\n", DXL_ID[i], dxl_present_position);
        }
    }

    return true;
}



void set_arm_position(double *curr_position, double *new_position, dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler){
    int dxl_comm_result = COMM_TX_FAIL;   // Communication result
    uint8_t dxl_error = 0;                // Dynamixel error
    //uint16_t dxl_present_position = 0;    // Present position

    for(int i=0; i<NUM_MOTORS; i++){
        uint16_t new_pos_i = (uint16_t)round(new_position[DXL_ID[i]]);
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID[i], ADDR_RX_GOAL_POSITION, new_pos_i, &dxl_error);
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
    }

    bool moving = true;
    while(moving){
        moving = false;
        for(int i=0; i<NUM_MOTORS; i++){
            uint16_t move = 0;
            dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID[i], ADDR_RX_MOVING, &move, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
                printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            }
            else if (dxl_error != 0)
            {
                printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            }
            else{
                printf("ID %d Moving status %u\n", DXL_ID[i], move);
            }

            if(move)
                moving = true;
        }
    }

    for(int i=0; i<NUM_MOTORS; i++){
        uint16_t dxl_present_position;
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID[i], ADDR_RX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        else
        {
            curr_position[i] = dxl_present_position;

            printf("ID %d Present position -in range %d   -in radians %lf\n", DXL_ID[i], dxl_present_position, range_to_radians((double)dxl_present_position, DXL_ID[i]));
        }
    }
}


bool OpenPort(dynamixel::PortHandler *portHandler){
    if (portHandler->openPort())
    {
        printf("Succeeded to open the port!\n");
        return true;
    }
    else
    {
        printf("Failed to open the port!\n");
        return false;
    }
}


int main(int narg, char *argv[]){
    // Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux or PortHandlerWindows
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Open port
    if(!OpenPort(portHandler)) 
    return 0;

    double *curr_position = new double[NUM_MOTORS]; 
    double *new_position  = new double[NUM_MOTORS];
    if(!initial_config(curr_position, portHandler, packetHandler))
    return 0;

    /*
    // Open file 
    FILE *F = fopen(argv[1], "r");

    if(F == NULL){
        printf("Error Opening File\n");
        return 0;
    }
    */
    bool running = true;

    while(running){
        /*
        if(feof(F)){
            printf("End of file reached\n");
            running = false;
            break;
        }
        

        for(int i=0; i<NUM_MOTORS; i++)
            fscanf(F, "%lf", &new_position[i]);
        */

        for(int i=0; i<NUM_MOTORS; i++)
            scanf("%lf", &new_position[i]);

        radians_to_range(new_position);
        set_arm_position(curr_position, new_position, portHandler, packetHandler);

    }

}