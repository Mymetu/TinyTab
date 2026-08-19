#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

#include "Stream.h"

namespace fs {

struct FileState;

class File : public Stream {
public:
    File();
    explicit File(std::shared_ptr<FileState> state);

    explicit operator bool() const;
    int available() const override;
    int read() override;
    size_t read(uint8_t *destination, size_t length);
    size_t write(const uint8_t *source, size_t length) override;
    size_t write(uint8_t value) override;
    bool seek(uint32_t position);
    size_t size() const;
    bool isDirectory() const;
    const char *name() const;
    File openNextFile();
    void close();

private:
    std::shared_ptr<FileState> state_;
};

class FS {
public:
    explicit FS(const char *base_path = "/meshcore");
    virtual ~FS() = default;

    File open(const char *path, const char *mode = "r", bool create_path = false);
    bool exists(const char *path) const;
    bool mkdir(const char *path) const;
    bool remove(const char *path) const;

protected:
    const char *base_path() const { return base_path_; }
    bool make_path(char *destination, size_t size, const char *path) const;

private:
    const char *base_path_;
};

class SPIFFSFS : public FS {
public:
    SPIFFSFS();
    bool format();
    size_t totalBytes() const;
    size_t usedBytes() const;
};

}  // namespace fs

using File = fs::File;
