
#include <string.h>
#include "Storage.h"

using namespace Quan;

QuanStorage::QuanStorage()
{}

void QuanStorage::init(void*)
{}

void QuanStorage::read_block(void* dst, uint16_t src, size_t n) {
    memset(dst, 0, n);
}

void QuanStorage::write_block(uint16_t loc, const void* src, size_t n)
{}

