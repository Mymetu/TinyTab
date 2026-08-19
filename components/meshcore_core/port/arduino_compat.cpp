#include "Arduino.h"
#include "SPIFFS.h"

#include <cerrno>
#include <cstdio>
#include <dirent.h>
#include <sys/stat.h>

#include "esp_random.h"
#include "esp_rom_sys.h"
#include "esp_spiffs.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace fs {

namespace {

bool copy_text(char *destination, size_t size, const char *source) {
    if(destination == nullptr || size == 0 || source == nullptr) return false;
    const size_t length = std::strlen(source);
    if(length >= size) return false;
    std::memcpy(destination, source, length + 1);
    return true;
}

bool join_path(char *destination, size_t size, const char *parent, const char *name) {
    if(destination == nullptr || size == 0 || parent == nullptr || name == nullptr) return false;
    const size_t parent_length = std::strlen(parent);
    const size_t name_length = std::strlen(name);
    if(parent_length + 1 + name_length >= size) return false;
    std::memcpy(destination, parent, parent_length);
    destination[parent_length] = '/';
    std::memcpy(destination + parent_length + 1, name, name_length + 1);
    return true;
}

}  // namespace

struct FileState {
    FILE *file = nullptr;
    DIR *directory = nullptr;
    char path[256] = {};
    char name[128] = {};

    ~FileState() {
        if(file != nullptr) std::fclose(file);
        if(directory != nullptr) closedir(directory);
    }
};

File::File() = default;
File::File(std::shared_ptr<FileState> state) : state_(std::move(state)) {}

File::operator bool() const {
    return state_ != nullptr && (state_->file != nullptr || state_->directory != nullptr);
}

int File::available() const {
    if(!state_ || !state_->file) return 0;
    const long current = std::ftell(state_->file);
    std::fseek(state_->file, 0, SEEK_END);
    const long end = std::ftell(state_->file);
    std::fseek(state_->file, current, SEEK_SET);
    return end > current ? static_cast<int>(end - current) : 0;
}

int File::read() {
    if(!state_ || !state_->file) return -1;
    return std::fgetc(state_->file);
}

size_t File::read(uint8_t *destination, size_t length) {
    if(!state_ || !state_->file || destination == nullptr) return 0;
    return std::fread(destination, 1, length, state_->file);
}

size_t File::write(const uint8_t *source, size_t length) {
    if(!state_ || !state_->file || source == nullptr) return 0;
    const size_t written = std::fwrite(source, 1, length, state_->file);
    std::fflush(state_->file);
    return written;
}

size_t File::write(uint8_t value) { return write(&value, 1); }

bool File::seek(uint32_t position) {
    return state_ && state_->file && std::fseek(state_->file, position, SEEK_SET) == 0;
}

size_t File::size() const {
    if(!state_ || !state_->file) return 0;
    const long current = std::ftell(state_->file);
    std::fseek(state_->file, 0, SEEK_END);
    const long end = std::ftell(state_->file);
    std::fseek(state_->file, current, SEEK_SET);
    return end > 0 ? static_cast<size_t>(end) : 0;
}

bool File::isDirectory() const { return state_ && state_->directory != nullptr; }
const char *File::name() const { return state_ ? state_->name : ""; }

File File::openNextFile() {
    if(!state_ || !state_->directory) return {};
    while(dirent *entry = readdir(state_->directory)) {
        if(std::strcmp(entry->d_name, ".") == 0 || std::strcmp(entry->d_name, "..") == 0) continue;
        char child_path[256];
        if(!join_path(child_path, sizeof(child_path), state_->path, entry->d_name)) continue;
        auto child = std::make_shared<FileState>();
        if(!copy_text(child->path, sizeof(child->path), child_path) ||
           !copy_text(child->name, sizeof(child->name), entry->d_name)) continue;
        struct stat info = {};
        if(stat(child_path, &info) == 0 && S_ISDIR(info.st_mode)) {
            child->directory = opendir(child_path);
        } else {
            child->file = std::fopen(child_path, "r");
        }
        if(child->file || child->directory) return File(child);
    }
    return {};
}

void File::close() { state_.reset(); }

FS::FS(const char *base_path) : base_path_(base_path) {}

bool FS::make_path(char *destination, size_t size, const char *path) const {
    if(!destination || size == 0 || !path || path[0] == '\0') return false;
    const char *relative = path[0] == '/' ? path + 1 : path;
    return join_path(destination, size, base_path_, relative);
}

File FS::open(const char *path, const char *mode, bool) {
    char full_path[256];
    if(!make_path(full_path, sizeof(full_path), path)) return {};

    struct stat info = {};
    auto state = std::make_shared<FileState>();
    if(!copy_text(state->path, sizeof(state->path), full_path)) return {};
    const char *slash = std::strrchr(full_path, '/');
    if(!copy_text(state->name, sizeof(state->name), slash ? slash + 1 : full_path)) return {};
    if(stat(full_path, &info) == 0 && S_ISDIR(info.st_mode)) {
        state->directory = opendir(full_path);
    } else {
        state->file = std::fopen(full_path, mode);
    }
    return File(state);
}

bool FS::exists(const char *path) const {
    char full_path[256];
    struct stat info = {};
    return make_path(full_path, sizeof(full_path), path) && stat(full_path, &info) == 0;
}

bool FS::mkdir(const char *path) const {
    char full_path[256];
    if(!make_path(full_path, sizeof(full_path), path)) return false;
    return ::mkdir(full_path, 0775) == 0 || errno == EEXIST;
}

bool FS::remove(const char *path) const {
    char full_path[256];
    return make_path(full_path, sizeof(full_path), path) && std::remove(full_path) == 0;
}

SPIFFSFS::SPIFFSFS() : FS("/meshcore") {}
bool SPIFFSFS::format() { return esp_spiffs_format("storage") == ESP_OK; }

size_t SPIFFSFS::totalBytes() const {
    size_t total = 0;
    size_t used = 0;
    return esp_spiffs_info("storage", &total, &used) == ESP_OK ? total : 0;
}

size_t SPIFFSFS::usedBytes() const {
    size_t total = 0;
    size_t used = 0;
    return esp_spiffs_info("storage", &total, &used) == ESP_OK ? used : 0;
}

}  // namespace fs

fs::SPIFFSFS SPIFFS;
IdfSerialCompat Serial;

unsigned long millis(void) {
    return static_cast<unsigned long>(esp_timer_get_time() / 1000ULL);
}

unsigned long micros(void) { return static_cast<unsigned long>(esp_timer_get_time()); }

void delay(unsigned long milliseconds) { vTaskDelay(pdMS_TO_TICKS(milliseconds)); }
void delayMicroseconds(unsigned int microseconds) { esp_rom_delay_us(microseconds); }
void randomSeed(unsigned long) {}

long random(long maximum) {
    if(maximum <= 0) return 0;
    return static_cast<long>(esp_random() % static_cast<uint32_t>(maximum));
}

long random(long minimum, long maximum) {
    if(maximum <= minimum) return minimum;
    return minimum + random(maximum - minimum);
}

char *ultoa(unsigned long value, char *buffer, int base) {
    if(!buffer || base < 2 || base > 36) return buffer;
    static constexpr char digits[] = "0123456789abcdefghijklmnopqrstuvwxyz";
    char reversed[sizeof(value) * 8 + 1];
    size_t length = 0;
    do {
        reversed[length++] = digits[value % static_cast<unsigned long>(base)];
        value /= static_cast<unsigned long>(base);
    } while(value != 0);
    for(size_t i = 0; i < length; ++i) buffer[i] = reversed[length - i - 1];
    buffer[length] = '\0';
    return buffer;
}

char *ltoa(long value, char *buffer, int base) {
    if(!buffer || base < 2 || base > 36) return buffer;
    if(value >= 0 || base != 10) return ultoa(static_cast<unsigned long>(value), buffer, base);
    buffer[0] = '-';
    const unsigned long magnitude = static_cast<unsigned long>(-(value + 1)) + 1UL;
    ultoa(magnitude, buffer + 1, base);
    return buffer;
}

size_t IdfSerialCompat::write(uint8_t value) {
    return std::fputc(value, stdout) == EOF ? 0 : 1;
}

size_t IdfSerialCompat::print(const char *value) {
    return value ? static_cast<size_t>(std::printf("%s", value)) : 0;
}
size_t IdfSerialCompat::print(char value) { return static_cast<size_t>(std::printf("%c", value)); }
size_t IdfSerialCompat::print(int value, int base) {
    return static_cast<size_t>(base == HEX ? std::printf("%X", value) : std::printf("%d", value));
}
size_t IdfSerialCompat::print(unsigned value, int base) {
    return static_cast<size_t>(base == HEX ? std::printf("%X", value) : std::printf("%u", value));
}
size_t IdfSerialCompat::println(const char *value) {
    return value ? static_cast<size_t>(std::printf("%s\n", value)) : 0;
}
size_t IdfSerialCompat::println(int value, int base) {
    return print(value, base) + static_cast<size_t>(std::printf("\n"));
}
