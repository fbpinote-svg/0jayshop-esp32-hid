#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <span>
#include <vector>
#include <windows.h>

namespace platf::esp32::detail {
  enum class overlapped_domain_e { serial, hid, test };
  enum class overlapped_wait_action_e { complete, fail, quarantine };

  constexpr overlapped_wait_action_e overlapped_wait_action(DWORD result) {
    if (result == WAIT_OBJECT_0) return overlapped_wait_action_e::complete;
    if (result == WAIT_TIMEOUT) return overlapped_wait_action_e::quarantine;
    return overlapped_wait_action_e::fail;
  }

  class overlapped_operation_t {
  public:
    static std::unique_ptr<overlapped_operation_t> create(HANDLE source, size_t size, overlapped_domain_e domain) {
      recover_completed(domain);
      if (!reserve(domain)) return {};
      HANDLE duplicate = INVALID_HANDLE_VALUE;
      if (!DuplicateHandle(GetCurrentProcess(), source, GetCurrentProcess(), &duplicate, 0, FALSE, DUPLICATE_SAME_ACCESS)) {
        release(domain);
        return {};
      }
      auto operation = std::unique_ptr<overlapped_operation_t>(new overlapped_operation_t(duplicate, size, domain));
      if (!operation->overlapped_.hEvent) return {};
      return operation;
    }

    ~overlapped_operation_t() {
      if (overlapped_.hEvent) CloseHandle(overlapped_.hEvent);
      if (handle_ != INVALID_HANDLE_VALUE) CloseHandle(handle_);
      if (reserved_) release(domain_);
    }

    bool write(std::span<const uint8_t> data, DWORD timeout_ms) {
      buffer_.assign(data.begin(), data.end());
      return finish(WriteFile(handle_, buffer_.data(), static_cast<DWORD>(buffer_.size()), &transferred_, &overlapped_), timeout_ms);
    }

    bool read(DWORD timeout_ms) {
      return finish(ReadFile(handle_, buffer_.data(), static_cast<DWORD>(buffer_.size()), &transferred_, &overlapped_), timeout_ms);
    }

    DWORD transferred() const { return transferred_; }
    std::span<const uint8_t> data() const { return buffer_; }
    bool deferred() const { return deferred_; }

    static bool poisoned(overlapped_domain_e domain) {
      std::lock_guard lock(registry_mutex());
      return registry()[index(domain)] != nullptr && registry()[index(domain)] != reserved_marker();
    }

    static bool take_poison_notice(overlapped_domain_e domain) {
      std::lock_guard lock(registry_mutex());
      const auto slot = registry()[index(domain)];
      auto &reported = poison_notices()[index(domain)];
      if (slot == nullptr || slot == reserved_marker() || reported) return false;
      reported = true;
      return true;
    }

    static void quarantine(std::unique_ptr<overlapped_operation_t> operation) {
      if (!operation || !operation->deferred_) return;
      auto *raw_operation = operation.release();
      std::lock_guard lock(registry_mutex());
      registry()[index(raw_operation->domain_)] = raw_operation;
      poison_notices()[index(raw_operation->domain_)] = false;
      raw_operation->reserved_ = false;
    }

  private:
    overlapped_operation_t(HANDLE handle, size_t size, overlapped_domain_e domain): handle_(handle), buffer_(size), domain_(domain) {
      overlapped_.hEvent = CreateEventW(nullptr, TRUE, FALSE, nullptr);
    }

    static size_t index(overlapped_domain_e domain) { return static_cast<size_t>(domain); }
    static void *reserved_marker() { return reinterpret_cast<void *>(1); }
    static std::array<void *, 3> &registry() {
      // ponytail: Process-lifetime storage caps an uncancellable driver at one quarantined operation per transport; restart releases it safely.
      static auto *slots = new std::array<void *, 3> {};
      return *slots;
    }
    static std::array<bool, 3> &poison_notices() {
      static auto *notices = new std::array<bool, 3> {};
      return *notices;
    }
    static std::mutex &registry_mutex() {
      static auto *mutex = new std::mutex;
      return *mutex;
    }
    static void recover_completed(overlapped_domain_e domain) {
      overlapped_operation_t *completed = nullptr;
      {
        std::lock_guard lock(registry_mutex());
        auto &slot = registry()[index(domain)];
        if (slot == nullptr || slot == reserved_marker()) return;
        auto *operation = static_cast<overlapped_operation_t *>(slot);
        if (WaitForSingleObject(operation->overlapped_.hEvent, 0) != WAIT_OBJECT_0) return;
        GetOverlappedResult(operation->handle_, &operation->overlapped_, &operation->transferred_, FALSE);
        operation->reserved_ = false;
        completed = operation;
        slot = nullptr;
        poison_notices()[index(domain)] = false;
      }
      delete completed;
    }
    static bool reserve(overlapped_domain_e domain) {
      std::lock_guard lock(registry_mutex());
      auto &slot = registry()[index(domain)];
      if (slot != nullptr) return false;
      slot = reserved_marker();
      return true;
    }
    static void release(overlapped_domain_e domain) {
      std::lock_guard lock(registry_mutex());
      auto &slot = registry()[index(domain)];
      if (slot == reserved_marker()) slot = nullptr;
    }

    bool finish(BOOL started, DWORD timeout_ms) {
      if (started) return true;
      if (GetLastError() != ERROR_IO_PENDING) return false;
      switch (overlapped_wait_action(WaitForSingleObject(overlapped_.hEvent, timeout_ms))) {
        case overlapped_wait_action_e::complete:
          return GetOverlappedResult(handle_, &overlapped_, &transferred_, FALSE);
        case overlapped_wait_action_e::quarantine:
        case overlapped_wait_action_e::fail:
          CancelIoEx(handle_, &overlapped_);
          deferred_ = true;
          return false;
      }
      return false;
    }

    HANDLE handle_ = INVALID_HANDLE_VALUE;
    OVERLAPPED overlapped_ {};
    std::vector<uint8_t> buffer_;
    DWORD transferred_ = 0;
    overlapped_domain_e domain_;
    bool deferred_ = false;
    bool reserved_ = true;
  };
}
