#pragma once

#include <memory>
#include <mutex>
#include <queue>
#include <string.h>

#ifdef __linux__
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <string>

#define SocketT socklen_t
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define SOCKADDR_IN sockaddr_in
#define SOCKADDR sockaddr
#define sprintfT sprintf
#elif _WIN32
#include <WS2tcpip.h>
#include <windows.h>
#include <winsock2.h>
#define SocketT SOCKET
#define sprintfT sprintf_s
#pragma comment(lib, "WS2_32.lib")
#endif

#define UDP_MAX_LENGTH 1600

namespace tanway {

#pragma pack(push, 1)
// pcap head 24byte
struct Pcap_FileHeader {
  Pcap_FileHeader()
      : magic(0xa1b2c3d4), version_major(0x0002), version_minor(0x0004),
        thiszone(0), sigfigs(0), snaplen(65536), linktype(1) {}
  u_int magic;                                // 0xa1b2c3d4
  u_short version_major;                      // 0x0002
  u_short version_minor;                      // 0x0004
  int thiszone; /* gmt to local correction */ // 0
  u_int sigfigs; /* accuracy of timestamps */ // 0
  u_int snaplen;
  /* max length saved portion of each pkt */        // 65535 0x000000ff
  u_int linktype; /* data link type (LINKTYPE_*) */ // 0x00000001
};
// pcap pack 16???
struct Pcap_PktHdr {
  Pcap_PktHdr() : tv_sec(0), tv_usec(0), caplen(0), len(0) {}
  u_int tv_sec;  /* seconds */
  u_int tv_usec; /* and microseconds */
  u_int caplen;  /* length of portion present data.size()+14+20+8 */
  u_int len;     /* length this packet (off wire) data.size()+14+20+8 */

  bool IsValid() {
    if (caplen > 0 && caplen < 1800 && caplen == len &&
        !(tv_sec == tv_usec && tv_sec == caplen)) {
      return true;
    } else
      return false;
  }
};

// net pack 14byte
struct NETHdr {
  NETHdr() : net_type(0x0008) {}
  void SetSourceMac(unsigned char *sourceMac) {
    memcpy(source_mac, sourceMac, 6);
  }
  unsigned char dest_mac[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
  unsigned char source_mac[6] = {0x1a, 0x2b, 0x3c, 0x4d, 0x5e, 0x6f};
  u_short net_type; // 0x0008   //IPv4
};

// IP pack  20byte,Big endian order
struct IPHdr {
  IPHdr()
      : version_and_header_len(0x45), services_codepoint(0x00), totalLength(0),
        identification(0x0000), flags(0x0040), time_live(0xff), protocol(17),
        header_checksum(0xffff) {}

  void SetLength(u_short length) {
    totalLength = 0x0000 | length << 8;
    totalLength = totalLength | length >> 8;
  };
  u_short GetLength() {
    u_short r_length = 0;
    r_length = r_length | totalLength << 8;
    r_length = r_length | totalLength >> 8;
    return r_length;
  }
  std::string GetSourceIP() {
    char str_ip[16];
    sprintfT(str_ip, "%u.%u.%u.%u", source_ip[0], source_ip[1], source_ip[2],
             source_ip[3]);
    return std::string(str_ip);
  }
  // private:
  unsigned char version_and_header_len; // 0x45
  unsigned char services_codepoint;     // 0x00
  u_short totalLength;                  // data.size()+20+8
  u_short identification;
  u_short flags;           // 0x0040
  unsigned char time_live; // 0xff
  unsigned char protocol;  // 17 UDP
  u_short header_checksum;
  unsigned char source_ip[4] = {0, 0, 0, 0};
  unsigned char dest_ip[4] = {0, 0, 0, 0};
};

// UDP pack
struct UDPHdr {
  UDPHdr() : sourcePort(0), destPort(0), length(0), check(0) {}
  void SetSourcePort(u_short port) {
    sourcePort = 0x0000 | port << 8;
    sourcePort = sourcePort | port >> 8;
  };
  void SetDestPort(u_short port) {
    destPort = 0x0000 | port << 8;
    destPort = destPort | port >> 8;
  };
  void SetLength(u_short len) {
    length = 0x0000 | len << 8;
    length = length | len >> 8;
  };
  u_short GetDestPort() {
    u_short port = 0;
    port = port | destPort << 8;
    port = port | destPort >> 8;
    return port;
  }
  u_short GetLength() {
    u_short r_length = 0;
    r_length = r_length | length << 8;
    r_length = r_length | length >> 8;
    return r_length;
  }
  // private:
  u_short sourcePort;
  u_short destPort;
  u_short length; // data.size()+8
  u_short check;
};
#pragma pack(pop)

struct UDPPackage {
  typedef std::shared_ptr<UDPPackage> Ptr;

  UDPPackage() : m_length(0) {
#ifdef __linux__
    timeval start;
    gettimeofday(&start, NULL);
    t_sec = start.tv_sec;
    t_usec = start.tv_usec;
#elif _WIN32
    time_t clock;
    struct tm tm;
    SYSTEMTIME wtm;
    GetLocalTime(&wtm);
    tm.tm_year = wtm.wYear - 1900;
    tm.tm_mon = wtm.wMonth - 1;
    tm.tm_mday = wtm.wDay;
    tm.tm_hour = wtm.wHour;
    tm.tm_min = wtm.wMinute;
    tm.tm_sec = wtm.wSecond;
    tm.tm_isdst = -1;
    clock = mktime(&tm);
    t_sec = (long)clock;
    t_usec = wtm.wMilliseconds * 1000;
#endif
  }
  char m_szData[UDP_MAX_LENGTH];
  int m_length;

  bool framed = false;
  uint64_t frameIndex;
  // time
  unsigned int t_sec;
  unsigned int t_usec;
};

class PackageList {
public:
  PackageList(int size = 200000);
  ~PackageList();

  void PushPackage(UDPPackage::Ptr udpPackage);
  UDPPackage::Ptr PopPackage();

  void PushFreePackage(UDPPackage::Ptr udpPackage);
  UDPPackage::Ptr PopFreePackage();

  int Size();
  void Clear();

private:
  std::queue<UDPPackage::Ptr> _queue;
  std::queue<UDPPackage::Ptr> _freeQueue;
  std::mutex _mutex;
};
} // namespace tanway
