#include "PackageList.h"
namespace tanway {
PackageList::PackageList(int size) {
  for (int var = 0; var < size; ++var) {
    UDPPackage::Ptr data(new UDPPackage);
    _freeQueue.push(data);
  }
}

PackageList::~PackageList() {
  std::queue<UDPPackage::Ptr> empty;
  swap(empty, _queue);
  swap(empty, _freeQueue);
}

void PackageList::PushPackage(UDPPackage::Ptr udpPackage) {
  std::lock_guard<std::mutex> lock(_mutex);
  _queue.push(udpPackage);
}

UDPPackage::Ptr PackageList::PopPackage() {
  std::lock_guard<std::mutex> lock(_mutex);
  if (_queue.size() == 0)
    return nullptr;
  UDPPackage::Ptr pack = _queue.front();
  _queue.pop();
  return pack;
}

void PackageList::PushFreePackage(UDPPackage::Ptr udpPackage) {
  std::lock_guard<std::mutex> lock(_mutex);
  udpPackage->frameIndex = 0;
  udpPackage->framed = false;
  _freeQueue.push(udpPackage);
}

UDPPackage::Ptr PackageList::PopFreePackage() {
  std::lock_guard<std::mutex> lock(_mutex);
  if (_freeQueue.size() == 0) {
    return nullptr;
  }
  UDPPackage::Ptr pack = _freeQueue.front();
  _freeQueue.pop();
  return pack;
}

int PackageList::Size() {
  std::lock_guard<std::mutex> lock(_mutex);
  return _queue.size();
}

void PackageList::Clear() {
  std::lock_guard<std::mutex> lock(_mutex);
  while (_queue.size() != 0) {
    UDPPackage::Ptr pack = _queue.front();
    _queue.pop();
    _freeQueue.push(pack);
  }
}

} // namespace tanway
