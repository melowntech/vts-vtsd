#ifndef libvadstena_http_lock_hpp_included_
#define libvadstena_http_lock_hpp_included_

#include <mutex>

class LockGuard
{
public:
    typedef boost::optional<std::mutex> OptionalMutex;

    explicit LockGuard(OptionalMutex &m)
        : m_(m)
    {
        if (m_) { m_->lock(); }
    }
    ~LockGuard() { if (m_) { m_->unlock(); } }

private:
    OptionalMutex &m_;
};

#endif // libvadstena_http_lock_hpp_included_
