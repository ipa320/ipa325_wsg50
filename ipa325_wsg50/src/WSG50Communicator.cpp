#include "WSG50Communicator.h"
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <stdlib.h>
#include <ros/ros.h>

using namespace boost;
using namespace std;

#define DEBUG       0

#ifndef SER_MSG_NUM_HEADER_BYTES
#define SER_MSG_NUM_HEADER_BYTES    3       //!< number of header bytes
#endif

#ifndef SER_MSG_HEADER_BYTE
#define SER_MSG_HEADER_BYTE     0xAA        //!< header byte value
#endif

boost::asio::io_service io_service;
boost::asio::ip::tcp::resolver resolver(io_service);
boost::asio::ip::tcp::socket sock(io_service);
std::array<char, 512> buffer;
std::array<std::string, 64> _splitMessages;

using boost::asio::ip::tcp;

// Checksum calculation table
//
const unsigned short CRC_TABLE[256] = {
    0x0000,  0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};


/*!
 * Commands
 */
// Connection Manager
static short _LOOP              = 0x06;     // Loop-Back
static short _DISCONNECT        = 0x07;     // announce Disconnect -> stops every movement

// Motion Control
static short _HOMING            = 0x20;     // homing movement
                                            // params: 0: default value; 1: homing in positive movement dir; 2: homing in negative movement dir.
static short _PREPFINGERS       = 0x21;     // Pre-position Fingers: move fingers to defined opening width
static short _STOP              = 0x22;     // Stop
static short _FASTSTOP          = 0x23;     // Issue Fast-Stop
static short _ACKFASTSTOP       = 0x24;     // Acknowledgement FastStop
static short _GRASPPART         = 0x25;     // Grasp part by passing normal width and grasping-speed
static short _RELEASEPART       = 0x26;     // Release a previously grasped part

// Motion Configuration
static short _SETACC            = 0x30;     // Set Acceleration
static short _GETACC            = 0x31;     // Get Acceleration
static short _SETFORCELIMIT     = 0x32;     // Set force-limit (float) in Newtons
static short _GETFORCELIMIT     = 0x33;     // Get force-limit
static short _SETSOFTLIMIT      = 0x34;     // Set soft limits for both the minus and plus direction (float minus, float plus)
static short _GETSOFTLIMIT      = 0x35;     // Get soft limits
static short _CLRSOFTLIMIT      = 0x36;     // Clear any set soft limits
static short _TAREFORCESENSOR   = 0x38;     // zeroes the connected force sensor used for the force control loop

// system state commands
static short _GETSYSTSTATE      = 0x40;     // Get the current system state
static short _GETGRASPSTATE     = 0x41;     // Get the current grasping state
                                            // idle = 0; grasping = 1; no part found = 2; part lost = 3; holding = 4
                                            // releasing = 5; positioning = 6; error = 7
static short _GETGRASPSTATS     = 0x42;     // get current grasping statistics for the number of executed grasps
static short _GETWIDTH          = 0x43;     // Get opening width
static short _GETSPEED          = 0x44;     // Get the current finger speed
static short _GETFORCE          = 0x45;     // Get the current grasping force
static short _GETTEMP           = 0x46;     // Get the current device temperature

// system configuration
static short _GETSYSTINFO       = 0x50;     // Get information about the connected device
static short _SETDEVTAG         = 0x51;     // Set the device tag (string)
static short _GETDEVTAG         = 0x52;     // Get the device tag
static short _GETSYSTLIMITS     = 0x53;     // Get the gripper's physical limits for stroke, speed, acceleration and force

// finger interface
static short _GETFINGERINFO     = 0x60;     // Return information about the connected fingers
static short _GETFINGERFLAGS    = 0x61;     // Return the state flags for the selected finger (params: int fingerindex)
static short _FINGERPOWERCTRL   = 0x62;     // enables or disables the power supply for the selected finter (params: int fingerindex, enum ON/OFF)
static short _GETFINGERDATA     = 0x63;     // return the current finger data for predefined finger types (params: int fingerindex)




/** ********************************
 * CLASS WSG50Communicator
 ***********************************/

WSG50Communicator::WSG50Communicator(std::string ip, std::string port)
{
    ROS_INFO_STREAM("WSG50Communicator(): IP = " << ip << ", PORT = " << port);
    // set ip and port
    //
    this->_IP = ip;
    this->_PORT = port;

    // initialize startup variables
    //
    this->_checkingConnection   = false;
    this->_respMsgDataAllocated = false;
    this->_respTCPBuffAllocated = false;
    this->clearIMsgBuffer();
}

/*
 *Destructor
 *stop running connections
 */
WSG50Communicator::~WSG50Communicator(void)
{
    ROS_DEBUG("Closing down");

    // stop and release connection
    //
    stopConnection();
}

/*
 *start a connection in a separate thread
 */
void WSG50Communicator::startConnection(void)
{
    ROS_DEBUG("Starting connection");

    // check if connection is already set
    //
    if(_connection != nullptr) {
        if(DEBUG) {
            ROS_WARN("It seems that already a connection is established!");
        }
        return;
    }

    // start separate thread for the connection
    //
    this->_keep_alive = true;
    _connection = std::make_shared<std::thread>(std::bind(&WSG50Communicator::connect, this));
}

/*
 *send disconnect-message
 *stop connection and wait for connection-thread to end
 */
void WSG50Communicator::stopConnection(void)
{
    if(!this->_keep_alive) {
        ROS_INFO("connection seems already closed.");
        return;
    }

    // send Disconnect message
    //
    TMESSAGE msg;
    createDisconnectMessage(&msg);

    msg_send(&msg);

    // set _keep_alive to false
    //
    this->_keep_alive = false;

    // wait until thread is closed
    //
    this->_connection->join();

    ROS_INFO("Connection closed.");
}



void WSG50Communicator::read_handler(const boost::system::error_code &ec,
                  std::size_t len)
{
    TRESPONSE       responseMsg;
    unsigned char*  responseTCPBuffer;
    unsigned char*  completeResponse = (unsigned char * ) buffer.data();
    unsigned char*  partResponse;
    int             bufflength,
                    index, i,
                    endPos,
                    count = 0,
                    pos1 = 0, pos2 = 0, searchPos = 0;
    unsigned char   delimiter[] = {0xAA, 0xAA, 0xAA};

    std::stringstream err;

//    if(DEBUG) ROS_INFO("read_handler called... (length: %d)", (int) len);


    // if imsgBuffer is not set, make buffer empty
    if(_iMsgBufferSize <= 0)
        this->clearIMsgBuffer();


    if(!ec)
    {
        logmsg.str("");

        // ***************************************************
        // read next data packet from tcp socket
        //
        sock.async_read_some(boost::asio::buffer( buffer ),
                             boost::bind(&WSG50Communicator::read_handler, this, _1, _2));

        // convert message into array
        //
        responseTCPBuffer = (unsigned char * ) buffer.data();
        this->_respTCPBuffAllocated = true;
//        if(DEBUG) this->printHexArray(responseTCPBuffer, len);

//        logmsg << std::hex << std::string((char *) responseTCPBuffer, 0, len).c_str() << std::dec << endl;

        // *****************************************************
        // split response into single messages
        // this is necessary, because the tcp-socket may deliver concatenated messages instead of single messages
        //
        pos1 = findOccurence(completeResponse, len, delimiter, 3, searchPos);

        // *************************************************************
        // EXCEPTION: if there is a message split between two tcp-data packets
        // if first position is not 0, then the messages have been split
        //
        if(pos1 != 0) {
            // check if there is another part inside the buffer
            //
            if(_iMsgBufferSize > 0) {
                // add the other parts to the buffer
                //
                int fin = _iMsgBufferSize + pos1;
                int count = 0;
                for(i=_iMsgBufferSize;i<fin;i++) {
                    _iMsgBuffer[i] = completeResponse[count];
                    _iMsgBufferSize++;
                    count++;
                }
                if(DEBUG) {
                    ROS_WARN("Message has been split. try concatenate messages");
                    printHexArray(_iMsgBuffer, _iMsgBufferSize);
                }

                // create tresponse of this first message
                //
                responseMsg = createTRESPONSE(_iMsgBuffer, _iMsgBufferSize);

                // check for disconnect response
                //
                if(responseMsg.id == 0x07) {
                    io_service.stop();
                }

                // send update response
                //
                _observer->update(&responseMsg);

                // clear the buffer again
                //
                clearIMsgBuffer();

            } else {
                if(DEBUG) ROS_ERROR("missing the first part of the message!");
            }
        }

        // ***********************************************************************
        // Loop through the response buffer
        // * extracting each response message from the devide (delimited by AA AA AA preamble)
        // * create TRESPONSE
        // * update observer
        //
        count = 0;
        while(pos1 != -1 && count < 200)
        {
            searchPos = pos1 + 4;
            pos2 = findOccurence(completeResponse, len, delimiter, 3, searchPos);

            // check if this is the last occurence
            //
            if((pos2 <= pos1 || pos2 <= 0) && len > pos1 && pos1 >= 0) {
                // get the length of the message
                //
                bufflength = len-pos1;
            }
            else if(pos1 >= 0 && pos2 > pos1)    // this is not the last occurence
            {
                // get the length of the message
                //
                bufflength = pos2-pos1;
            } else {
                bufflength = 0;
                return;
            }

//            if(DEBUG) ROS_INFO("loop-count = %d; found second occurence at index = %d; msg length = %d", count, pos2, bufflength);


            // *****************************************************
            // write this part of the message into an separate array
            // allocate char array
            //
            partResponse = new unsigned char[bufflength];

            // copy the message content
            //
            endPos = pos1 + bufflength;
            index = 0;
            for(i = pos1; i < endPos; i++) {
                partResponse[index] = completeResponse[i];
                index++;
            }

            // *****************************************************
            // set new pos1
            // new position 1 is the position of the next occurence, e.g. pos2
            //
            if(pos2 > 0 && pos2 > pos1 && pos1 != -1 ){
                pos1 = pos2;
            } else if(pos1 != -1 && pos2 == -1) {
                // stop condition if pos1 == -1
                //
                pos1 = -1;
                pos2 = -1;
            } else {
                pos2 = -1;
            }

            count++;


            // *****************************************************
            // create TRESPONSE Message
            //
            responseMsg = createTRESPONSE(partResponse, bufflength);
//            if(DEBUG) printTRESPONSE(responseMsg);

//            ROS_ERROR("response message length = %d", responseMsg.length);
            // check if response-message-length == -1, then there was an error creating the message
            if(responseMsg.length == -1) {
                ROS_DEBUG("could not create TRESPONSE. continue");

                // clear buffer
                //
                clearIMsgBuffer();

                // copy the rest of the message into the buffer;
                //
                for(i=0; i<bufflength; i++) {
                    _iMsgBuffer[i] = partResponse[i];
                }

                // set buffersize
                //
                _iMsgBufferSize = bufflength;

                delete[] partResponse;

                if(pos2 != -1) {
                    continue;
                } else {
                    break;
                }
            }


            // switch through different status codes and print error messages
            //
            if(responseMsg.status_code != E_SUCCESS && DEBUG) {
                printErrorCode(responseMsg.status_code);
            }

            // ******************************************************
            // if DISCONNECT response
            // if msg command id was 0x07 (disconnect message), then stop io_service
            // and delete communication
            //
            if(responseMsg.id == 0x07) {
                if(responseMsg.status_code == E_SUCCESS) {
                    ROS_WARN("Received successfull disconnect response!\n");
                    io_service.stop();
                } else {
                    printErrorCode(responseMsg.status_code);
                }

                // jump out of the loop
                //
                delete[] partResponse;
                break;
            }

            // ******************************************************
            // always call Observer and hand over response message
            //
            _observer->update(&responseMsg);


            // ******************************************************
            // free memory
            // and prepare for next message
            //
            delete[] partResponse;
        }
    } else {
        ROS_ERROR("an error occured when the read_handler was called");
        std::cout << ec.message() << std::endl;
    }
}

void WSG50Communicator::connect_handler(const boost::system::error_code &ec)
{
//    if(DEBUG) ROS_INFO("connect_handler called...");

    if(!ec)
    {
        // call read_handler
        // bind the async_read to the read_handler function
        //
        sock.async_read_some(boost::asio::buffer( buffer ),
                             boost::bind(&WSG50Communicator::read_handler, this, _1, _2));
    }
}


void WSG50Communicator::resolve_handler(const boost::system::error_code &ec,
                     boost::asio::ip::tcp::resolver::iterator it)
{
//    if(DEBUG) ROS_INFO("resolve_handler called...");

    if (!ec)
    {
        // call connection handler
        //
        sock.async_connect(*it, boost::bind(&WSG50Communicator::connect_handler, this, _1));
    }
}

/*
 * Asynchroneous thread to establish a connection and keep this connection alive
 * until the variable _keep_alive is set to false
 */
void WSG50Communicator::connect()
{
    // use asynchroneous connect and resolve / read handlers
    //
    boost::asio::ip::tcp::resolver::query query(this->_IP, this->_PORT);
    resolver.async_resolve(query, boost::bind(&WSG50Communicator::resolve_handler, this, _1, _2));

    // start service
    //
    io_service.run();
}


// attach new observer
// which will be updated once messages have been received
//
void WSG50Communicator::Attach(WSG50Observer *observer)
{
    this->_observer = observer;
}


// write message to socket
//
void WSG50Communicator::pushMessage(TMESSAGE *msg)
{
    if(false) {
        std::cout << "Push message: ";
        this->printHexArray(msg->data, (int) msg->length);
    }

    // lock
    _wsgBufferMutex.lock();

    // Convert message into byte sequence:
    WSGBUF = msg_build( msg, &WSGSIZE );
    if ( !WSGBUF )
    {
        printErrorCode(E_INSUFFICIENT_RESOURCES);
        return;
    }

//    if(DEBUG) {
//        std::cout << "Push TCP message: ";
//        this->printHexArray(WSGBUF, WSGSIZE);
//    }

    // write buffer
    //
    boost::asio::write(sock, boost::asio::buffer(WSGBUF, WSGSIZE));

    delete WSGBUF;

    // unlock
    _wsgBufferMutex.unlock();
}


// print array of hex-values bytewise
//
void WSG50Communicator::printHexArray(unsigned char *ar, int size)
{
    int i, iterator = 0, sh;

    sh = size/2;
    for(i=0; i<sh; i++) {
        iterator=i*2;
        printf("%02X %02X ", ar[iterator], ar[(iterator+1)]);
//        std::cout << std::hex << ar[iterator];
//        std::cout << " " << std::hex << ar[(iterator+1)] << std::dec;
    }

    if((size%2) != 0) // ungerade groesse
    {
        // print last position
        i = size - 1;
        printf("%02X", ar[i]);
    }

    std::cout << std::endl;
}



void WSG50Communicator::printErrorCode(TStat ec)
{
    switch (ec) {
    case E_NOT_AVAILABLE:
        ROS_ERROR("Received command error code: E_NOT_AVAILABLE");
        break;
    case E_NO_SENSOR:
        ROS_ERROR("Received command error code: E_NO_SENSOR");
        break;
    case E_NOT_INITIALIZED:
        ROS_ERROR("Received command error code: E_NOT_INITIALIZED");
        break;
    case E_ALREADY_RUNNING:
        ROS_ERROR("Received command error code: E_ALREADY_RUNNING");
        break;
    case E_FEATURE_NOT_SUPPORTED:
        ROS_ERROR("Received command error code: E_FEATURE_NOT_SUPPORTED");
        break;
    case E_INCONSISTENT_DATA:
        ROS_ERROR("Received command error code: E_INCONSISTENT_DATA");
        break;
    case E_TIMEOUT:
        ROS_ERROR("Received command error code: E_TIMEOUT");
        break;
    case E_READ_ERROR:
        ROS_ERROR("Received command error code: E_READ_ERROR");
        break;
    case E_WRITE_ERROR:
        ROS_ERROR("Received command error code: E_WRITE_ERROR");
        break;
    case E_INSUFFICIENT_RESOURCES:
        ROS_ERROR("Received command error code: E_INSUFFICIENT_RESOURCES");
        break;
    case E_CHECKSUM_ERROR:
        ROS_ERROR("Received command error code: E_CHECKSUM_ERROR");
        break;
    case E_NO_PARAM_EXPECTED:
        ROS_ERROR("Received command error code: E_NO_PARAM_EXPECTED");
        break;
    case E_NOT_ENOUGH_PARAMS:
        ROS_ERROR("Received command error code: E_NOT_ENOUGH_PARAMS");
        break;
    case E_CMD_UNKNOWN:
        ROS_ERROR("Received command error code: E_CMD_UNKNOWN");
        break;
    case E_CMD_FORMAT_ERROR:
        ROS_ERROR("Received command error code: E_CMD_FORMAT_ERROR");
        break;
    case E_ACCESS_DENIED:
        ROS_ERROR("Received command error code: E_ACCESS_DENIED");
        break;
    case E_ALREADY_OPEN:
        ROS_WARN("Received command error code: E_ALREADY_OPEN");
        break;
    case E_CMD_FAILED:
        ROS_ERROR("Received command error code: E_CMD_FAILED");
        break;
    case E_CMD_ABORTED:
        ROS_ERROR("Received command error code: E_CMD_ABORTED");
        break;
    case E_INVALID_HANDLE:
        ROS_ERROR("Received command error code: E_INVALID_HANDLE");
        break;
    case E_NOT_FOUND:
        ROS_ERROR("Received command error code: E_NOT_FOUND");
        break;
    case E_NOT_OPEN:
        ROS_ERROR("Received command error code: E_NOT_OPEN");
        break;
    case E_IO_ERROR:
        ROS_ERROR("Received command error code: E_IO_ERROR");
        break;
    case E_INVALID_PARAMETER:
        ROS_ERROR("Received command error code: E_INVALID_PARAMETER");
        break;
    case E_INDEX_OUT_OF_BOUNDS:
        ROS_ERROR("Received command error code: E_INDEX_OUT_OF_BOUNDS");
        break;
    case E_CMD_PENDING:
        ROS_WARN("Received command error code: E_CMD_PENDING");
        break;
    case E_OVERRUN:
        ROS_ERROR("Received command error code: E_OVERRUN");
        break;
    case E_RANGE_ERROR:
        ROS_ERROR("Received command error code: E_RANGE_ERROR");
        break;
    case E_AXIS_BLOCKED:
        ROS_ERROR("Received command error code: E_AXIS_BLOCKED");
        break;
    case E_FILE_EXISTS:
        ROS_ERROR("Received command error code: E_FILE_EXISTS");
        break;
    default:
        break;
    }
}




/*!
Send a message to an open file handle
@param *file Handle of an open file to which the message should be sent
@param *msg
Pointer to the message that should be sent
@return E_SUCCESS, if successful, otherwise error code
*/
bool WSG50Communicator::msg_send(TMESSAGE * msg )
{
    // Convert message into byte sequence:
    WSGBUF = msg_build( msg, &WSGSIZE );
    if ( !WSGBUF ) return( E_INSUFFICIENT_RESOURCES );

    // Transmit buffer:
    //
//    if(DEBUG) ROS_INFO("Send tcp message.");
    boost::asio::write(sock, boost::asio::buffer(WSGBUF, WSGSIZE));

    // Free allocated memory:
//    if(DEBUG) std::cout << "buffer size: " << WSGSIZE << std::endl;
    if(WSGSIZE > 0 && WSGBUF != nullptr) {
        delete[] WSGBUF;
        WSGBUF = nullptr;
        WSGSIZE = 0; // null pointer
    }
    return true;
}




/*********************************************************************/
/*!
Calculates the CRC checksum of an array by using a table.
The start value for calculating the CRC should be set to 0xFFFF.
@param *data points to the byte array from which checksum should
be calculated
@param size size of the byte array
@param crc
value calculated over another array and start value
of the crc16 calculation
@return CRC16 checksum
*/
/*********************************************************************/
unsigned short WSG50Communicator::checksum_update_crc16( unsigned char *data,
                                             unsigned int size, unsigned short crc )
{
    unsigned long c;
    /* process each byte prior to checksum field */
    for ( c=0; c < size; c++ ) {crc = CRC_TABLE[ ( crc ^ *( data ++ )) & 0x00FF ] ^ ( crc >> 8 );}
    return crc;
}


/*********************************************************************/
/*!
Builds a data packet from the given message.
You have to free the returned buffer, if you do not use it anymore.
@param *msg Pointer to the source message
@param *size
Returns the size of the created buffer
@return buffer containing the bytewise packet data or NULL in case
of an error.
*/
/*********************************************************************/
unsigned char * WSG50Communicator::msg_build( TMESSAGE * msg, unsigned int *size )
{
    unsigned char *buf;
    unsigned short chksum;
    unsigned int c, len;

    len = SER_MSG_NUM_HEADER_BYTES + 3 + 2 + msg->length;

    // allocate buffer (remember to delete buffer later!)
    //
    buf = new unsigned char[len];
    if ( !buf )
    {
        *size = 0;
        return nullptr;
    }

    // Assemble the message header:
    for ( c=0; c<SER_MSG_NUM_HEADER_BYTES; c++ ) buf[c] = SER_MSG_HEADER_BYTE;
    buf[ SER_MSG_NUM_HEADER_BYTES ] = msg->id; // Message ID
    buf[ SER_MSG_NUM_HEADER_BYTES + 1 ] = msg->length &0xFF; // Msg. length low byte
    buf[ SER_MSG_NUM_HEADER_BYTES + 2 ] = msg->length >> 8; // Msg. length high byte

    // Copy payload to buffer:
    if ( msg->length ) memcpy( &buf[ SER_MSG_NUM_HEADER_BYTES + 3 ], msg->data, msg->length );

    // Calculate the checksum over the header, include the preamble:
    chksum = checksum_update_crc16( buf, SER_MSG_NUM_HEADER_BYTES + 3 + msg->length, 0xFFFF );

    // Add checksum to message:
    buf[ SER_MSG_NUM_HEADER_BYTES + 3 + msg->length ] = chksum &0xFF;
    buf[ SER_MSG_NUM_HEADER_BYTES + 4 + msg->length ] = chksum >> 8;
    *size = len;
    return( buf );
}



void WSG50Communicator::createDisconnectMessage(TMESSAGE * msg)
{
    msg->id = _DISCONNECT;
    msg->data = nullptr;
    msg->length = 0;
}



/*
 *create the TRESPONSE message out of the received data stream.
 *
//! Response Messages
typedef struct
{
    unsigned short length;
    unsigned char id;
    unsigned short status_code;
    unsigned char *data;
} TRESPONSE;
 */
TRESPONSE WSG50Communicator::createTRESPONSE(unsigned char * data, size_t TCPPacketLength)
{
    TRESPONSE respMsg;
    unsigned short statusCode;
    unsigned char * _dat;
    int length;

    // check if message has minimum length
    //
    if(TCPPacketLength < 8) {
        // at least parts of the status-code is missing
        if(DEBUG) ROS_ERROR("response message incomplete! ignoring message");
        respMsg.length = -1;
        return respMsg;
    }

    // get Command ID
    //
    respMsg.id = data[3];

    // get data size
    //
    respMsg.length = (((data[5] & 0xff) << 8) | ((data[4] & 0xff))) - 2; // -2, because the statuscode takes the first two bytes

    // get Status Code
    //
    respMsg.status_code = (TStat) (((data[7] & 0xff) << 8) | ((data[6] & 0xff)));

    // reset data pointer
    //
    respMsg.data = nullptr;

    // check if given tcp length is less than length given in message
    //
    if((int) respMsg.length != ((int) (TCPPacketLength - 10)))
    {
        ROS_DEBUG("1 Length of response TCP-message does not match announced data-length!");
        ROS_DEBUG("2 actual length = %d, announced data-length = %d", ((int) TCPPacketLength -10), (int) respMsg.length);
        if(DEBUG) printHexArray(data, TCPPacketLength);
        respMsg.length = -1;
        return respMsg;
    }

    // get rest of the data if available
    //
    length = (int) respMsg.length;
    if(length > 0)
    {
        _dat = new unsigned char[length];
        for(int i=0; i<length; i++)
        {
            _dat[i] = data[(i+8)];
        }
        respMsg.data = _dat;
        this->_respMsgDataAllocated = true;
    }

    return respMsg;
}

/*
 * printf a TRESPONSE message dump
 */
void WSG50Communicator::printTRESPONSE(TRESPONSE msg)
{
    std::cout << "\t#####################" << std::endl << "\t# TRESPONSE Message:" << std::endl;
    std::cout << "\t# id: "; // << std::hex << std::uppercase << msg.id << std::endl;
    printf("%02X", msg.id);
    std::cout << std::endl << "\t# length: " << msg.length << std::endl;
    std::cout << "\t# status code: " << std::hex << std::uppercase << msg.status_code << std::endl;
    if(msg.length > 0)
    {
       std::cout << "\t# data: ";
        for(int i=0; i<msg.length; i++)
        {
            printf("%02X ", msg.data[i]);
//            std::cout << std::hex << msg.data[i] << " ";
        }
    } else {
        std::cout << "\t# no data!" << std::endl;
    }
    std::cout << std::endl << "\t#####################" << std::dec << std::endl;
}

// find the first occurence of a certain delimiter inside an unsigned char array
//
int WSG50Communicator::findOccurence(unsigned char *ar, int length, unsigned char delimiter, int startPos)
{
    int returnValue = -1;
    if(startPos >= length) return returnValue;
    for(int i=startPos; i<length; i++) {
        if(ar[i] == delimiter) {
            returnValue = i;
            break;}
    }
    return returnValue;
}

// find the first occurence of a certain delimiter inside an unsigned char array
//
int WSG50Communicator::findOccurence(unsigned char *ar,
                                     int length,
                                     unsigned char *delimiter,
                                     int delimiterLength,
                                     int startPos)
{
    int returnValue = -1, pos=0;
    bool failure=true;
    if(startPos >= length) return returnValue;
    for(int i=startPos; i<length; i++) {
        if(ar[i] == delimiter[0]) {
            // if the first delimiter matches, check if the other delimiters match too
            //
            failure=false;
            for(int j=0;j<delimiterLength;j++) {
                pos = i+j;
                if(delimiter[j] != ar[pos]) {
                    failure = true;
                    break;}
            }
            if(!failure) {
                returnValue = i;
            }
            break;
        }
    }
    return returnValue;
}

void WSG50Communicator::clearIMsgBuffer() {
    int i=0;
    for(i=0; i<MSGBUFFERSIZE; i++) {_iMsgBuffer[i]=0;}
    _iMsgBufferSize = 0;
}
