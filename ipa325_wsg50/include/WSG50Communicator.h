#pragma once
/**
 *
 *@brief
 * the WSG50Communicator encapsulates the TCP-connection and communication between
 * the WSG50Controller and the WSG50 Gripper
 *
 */

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <string>
#include <boost/thread.hpp>
//#include <WSG50Controller.h>
#include "WSG50Subject.h"
#include "WSG50Observer.h"



//! Typedef for Callback methods
//!
typedef boost::function1<void, TRESPONSE> callbackTRESPONSE;



/**
 * @brief The WSG50Communicator class
 * handle TCP-communication with the gripper
 *
 */
class WSG50Communicator : public WSG50Subject
{
public:

    /**
     * Constructor / Destructor
     */
//    WSG50Communicator(void);

    // class constructor, also establishes connectivity to the gripper
    //
    WSG50Communicator(std::string ip, std::string port);

    // destructor
    // release connection
    //
    ~WSG50Communicator(void);


    /**
     * Controls
     */

    // connect / reconnect to the gripper
    //
    void startConnection();

    // stop connection
    //
    void stopConnection();

    // write a message
    //
    void pushMessage(TMESSAGE *msg);


    /**
     * Observer register / unregister
     */
    void Attach(WSG50Observer *observer); // change to boost bind function

    // unused, since only one observer is registered
    //
    //void Detach();


    /**
     * Helpers
     */
    static void     printHexArray(unsigned char *ar, int size);

    void            printErrorCode(TStat ec);





    /*! #################################################################
     *  DEPRECATED!
     */

    /**
     * send given message to the device and return response (synchroneous messages)
     */
    void sendMessage(TMESSAGE *msg, TMESSAGE *response);
//    void sendAsyncMessage(TMESSAGE *msg, callbackTStat callbackResponse1, callbackTMESSAGE callbackResponse2);
    void sendAsyncMessage(TMESSAGE *msg, callbackTRESPONSE callbackResponse1, callbackTRESPONSE callbackResponse2);
    bool isCommunicationOK(void);




private:
    std::string _IP;
    std::string _PORT;

    bool        _checkingConnection;

    // Message
    TMESSAGE _msg;              // raw message
    unsigned char *WSGBUF;      // write-buffer
    unsigned int WSGSIZE;       // size of the write buffer


    // variables to check which data is allocated and what is freed
    bool        _respMsgDataAllocated,
                _respTCPBuffAllocated;


    /**
     * Networking
     */
    boost::asio::ip::tcp::resolver::iterator _endpoint_iterator;
    boost::thread *_connection;
    bool _keep_alive;
    void connect();

    void resolve_handler(const boost::system::error_code &ec,
                         boost::asio::ip::tcp::resolver::iterator it);
    void read_handler(const boost::system::error_code &ec,
                      std::size_t bytes_transferred);
    void connect_handler(const boost::system::error_code &ec);

    // Messaging-Methods
    void createDisconnectMessage(TMESSAGE * msg);


    /**
     * Helper methods
     */
    unsigned short  checksum_update_crc16( unsigned char *data,
                                           unsigned int size, unsigned short crc );
    TRESPONSE       createTRESPONSE(unsigned char * data, size_t len);

    // only internal use, e.g. to close the connection
    //
    bool            msg_send(TMESSAGE * msg);

    // create a buffer from TMESSAGE
    //
    unsigned char * msg_build(TMESSAGE * msg, unsigned int * size);

    void            printTRESPONSE(TRESPONSE msg);


    /**
     * threading
     */
    boost::mutex    _wsgBufferMutex;
};
