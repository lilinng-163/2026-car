#pragma once

#ifdef __cplusplus
#include <cstdint>

extern "C" {
#endif

#ifdef __cplusplus
}
#endif

// ============================================================
// C++ 接口
// ============================================================
#ifdef __cplusplus
class Beep {
public:
    static Beep& instance();

    void on();
    void off();
    void toggle();

private:
    Beep() = default;
};
#endif // __cplusplus

// ============================================================
// C 兼容接口
// ============================================================
#ifdef __cplusplus
extern "C" {
#endif



#ifdef __cplusplus
}
#endif
