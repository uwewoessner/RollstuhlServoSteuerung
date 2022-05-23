/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#ifndef _UDPCOMM_H_
#define _UDPCOMM_H_

//#include <util/common.h>
#define UTILEXPORT
#include <assert.h>
#include <stdio.h>
#include <iostream>

#ifndef _WIN32
#include <netinet/in.h>
#else
#include <winsock2.h>
#endif
#ifndef NULL
#define NULL 0
#endif
#define UDP_COMM_MAX_SIZE 1024
/**
 * Class for a very simple UDP communication
 *
 */
class UTILEXPORT UDPComm
{
public:
    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // ++ Constructors / Destructor
    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    /** Constructor
       *  Prepare receiver only / port
       */
    UDPComm(int localPort);

    // send udp packets only
    UDPComm(int port, const char *hostname);

    /** Constructor send and receive
       *  Prepare sender to hostname / port
       */
    UDPComm(const char *hostname, int port, int localPort,
            const char *mcastif = NULL, int mcastttl = -1);

    /** Constructor send and receive
       *  Prepare sender to hostname/port given as "host:port"
       */
    UDPComm(const char *hostPort, int localPort,
            const char *mcastif = NULL, int mcastttl = -1);

    /// Destructor : virtual in case we derive objects
    virtual ~UDPComm();

    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // ++ Operations
    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    // send a UDP packet: return #bytes sent, error ret=-1, error string set
    int send(const void *buffer, int numsendBytes);

    // send \0-terminated string
    int send(const char *string);

    int receive(void *buffer, int numBytes, double timeout=2.0); // wait by default 2 seconds for data
    // read into buffer
    // returns number of bytes read into buffer or -1
    int readMessage(); // read a Datagram with a maximum size of UDP_COMM_MAX_SIZE;
    int getMessagePart(void *buf, int size); // get part of this message, returns the number of bytes copied to buf or -1

    int enableBroadcast(bool state);

    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // ++ Attribute request/set functions
    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    // return true if bad
    bool isBad();

    // get error string of last error
    const char *errorMessage();

    const char *rawBuffer()
    {
        return (const char *)msgBuf;
    };
    int messageSize()
    {
        return msgSize;
    };
    int getReceiveSocket()
    {
        return d_rsocket;
    };

    // Funktion für Ein-/Ausblenden der Netzwerkfehler
    static void errorStatus_SW();
    static bool getError_SW();
    static void setError_SW(bool);
    sockaddr_in& getRemoteAddess() { return remote_adr; };

protected:
    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // ++  Attributes
    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    // address info to send to
    struct sockaddr_in d_address;

    // error buffer: contains non-empty string if bad things happen
    char d_error[1024];
    char msgBuf[UDP_COMM_MAX_SIZE];
    char *currMsgPtr;
    int msgSize;

    // Socket number
    int d_socket;
    int d_rsocket; // receive socket

private:
    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // ++  Internally used functions
    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    // Try to get IP, use no nameserver if dot-notation
    unsigned long getIP(const char *hostname);

    // set up server data (used by constructors)
    void setup(const char *hostname, int port, int localPort,
               const char *mcastif = NULL, int mcastttl = -1);

    int openReceivePort(int portnumber);
    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // ++  prevent auto-generated bit copy routines by default
    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    /// Copy-Constructor: NOT IMPLEMENTED, checked by assert
    UDPComm(const UDPComm &);

    /// Assignment operator: NOT IMPLEMENTED, checked by assert
    UDPComm &operator=(const UDPComm &);

    /// Default constructor: NOT IMPLEMENTED, checked by assert
    UDPComm();

    // Variable für Ein-/Ausblenden der Netzwerkfehler
    static bool error_SW;

    sockaddr_in  remote_adr;
    socklen_t rlen;
};
#endif
